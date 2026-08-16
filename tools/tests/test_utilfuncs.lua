--[[
Tests for scripts/Utils/UtilFuncs.lua - the table/string helpers and the debug layer.

Covers findings:
  B9  table:f_remove skipped elements (removed; the guard test proves it is gone)
  B10 the helpers moved off the global table/string namespaces onto ADTable
  P3  debug calls must not evaluate their arguments when the channel is off
  B6  AutoDrive.debugMsg logged unconditionally
]]

lu = require('luaunit')
require('test-setup')

require('UtilFuncs')

TestTableHelpers = {}

function TestTableHelpers:setUp()
    TestSetup.reset()
end

------------------------------------------------------------------------------------------------------------------------
--- B10 - helpers live on ADTable, not on the global table
------------------------------------------------------------------------------------------------------------------------
function TestTableHelpers:testHelpersAreOnADTable()
    lu.assertNotNil(ADTable, 'ADTable namespace must exist')
    for _, name in ipairs({ 'contains', 'count', 'indexOf', 'removeValue',
                            'f_contains', 'f_indexOf', 'f_find', 'f_filter', 'f_count',
                            'concatNil' }) do
        lu.assertIsFunction(ADTable[name], 'ADTable.' .. name .. ' must exist')
    end
    lu.assertIsFunction(ADString.random, 'ADString.random must exist')
end

function TestTableHelpers:testGlobalTableIsNotExtended()
    -- The whole point of B10: another mod loaded after AutoDrive must not inherit these.
    for _, name in ipairs({ 'contains', 'indexOf', 'removeValue', 'f_remove',
                            'f_contains', 'f_indexOf', 'f_find', 'f_filter',
                            'f_count', 'concatNil' }) do
        lu.assertNil(rawget(table, name),
            'table.' .. name .. ' must not be defined on the global table namespace')
    end
    lu.assertNil(rawget(string, 'random'), 'string.random must not be global')
end

------------------------------------------------------------------------------------------------------------------------
--- B9 - the broken f_remove is gone, and what replaces it must not skip elements
------------------------------------------------------------------------------------------------------------------------
function TestTableHelpers:testBrokenFRemoveIsGone()
    lu.assertNil(rawget(table, 'f_remove'), 'the broken table:f_remove must not survive')
end

function TestTableHelpers:testRemoveValueRemovesExactlyOne()
    local t = { 'a', 'b', 'b', 'c' }
    lu.assertTrue(ADTable.removeValue(t, 'b'))
    lu.assertEquals(t, { 'a', 'b', 'c' })
    lu.assertFalse(ADTable.removeValue(t, 'zzz'))
end

--- If a bulk remove is offered at all it must not skip adjacent matches - that was the B9 defect.
function TestTableHelpers:testBulkRemoveDoesNotSkipAdjacentMatches()
    if ADTable.removeAll == nil then
        return -- no bulk remove offered; nothing to get wrong
    end
    local t = { 1, 9, 9, 9, 2, 9, 3 }
    ADTable.removeAll(t, function(v) return v == 9 end)
    lu.assertEquals(t, { 1, 2, 3 })
end

function TestTableHelpers:testContainsAndIndexOf()
    local t = { 'x', 'y', 'z' }
    lu.assertTrue(ADTable.contains(t, 'y'))
    lu.assertFalse(ADTable.contains(t, 'q'))
    lu.assertEquals(ADTable.indexOf(t, 'z'), 3)
    lu.assertNil(ADTable.indexOf(t, 'q'))
end

function TestTableHelpers:testCount()
    lu.assertEquals(ADTable.count({}), 0)
    lu.assertEquals(ADTable.count({ 1, 2, 3 }), 3)
    lu.assertEquals(ADTable.count({ a = 1, b = 2 }), 2)
end

------------------------------------------------------------------------------------------------------------------------
--- P3 - no work when the channel is off
------------------------------------------------------------------------------------------------------------------------
TestDebugLayer = {}

function TestDebugLayer:setUp()
    TestSetup.reset()
end

function TestDebugLayer:testChannelOffProducesNoOutput()
    AutoDrive.currentDebugChannelMask = 0
    AutoDrive.debugPrint(nil, AutoDrive.DC_PATHINFO, 'should not appear')
    lu.assertEquals(Logging.countMatching('should not appear'), 0)
end

function TestDebugLayer:testChannelOnProducesOutput()
    AutoDrive.currentDebugChannelMask = AutoDrive.DC_PATHINFO
    AutoDrive.debugPrint(nil, AutoDrive.DC_PATHINFO, 'should appear')
    lu.assertEquals(Logging.countMatching('should appear'), 1)
end

--- debugMsg is deliberately unconditional: it is the "a module debug flag was explicitly turned
--- on" path that the per-module wrappers route to. That contract is fine and stays.
---
--- B6 was never about this function. It was about Gui.lua calling it directly, for a condition
--- that is not an error, so 15 lines appeared in every player's log with all channels off. The
--- fix belongs at that call site and is covered by test_gui.lua.
function TestDebugLayer:testDebugMsgIsUnconditionalByDesign()
    AutoDrive.currentDebugChannelMask = 0
    AutoDrive.debugMsg(nil, 'explicitly requested')
    lu.assertEquals(Logging.countMatching('explicitly requested'), 1,
        'debugMsg is the explicit path and must still log')
end

--- Guard against the constant extraction silently degrading: if this drops, every channel test
--- above becomes vacuous.
function TestDebugLayer:testConstantsWereExtractedFromTheMod()
    lu.assertTrue(AutoDrive.constantsLoaded >= 20,
        'expected at least 20 constants from scripts/AutoDrive.lua, got '
        .. tostring(AutoDrive.constantsLoaded))
    lu.assertEquals(AutoDrive.DC_PATHINFO, 16)
    lu.assertEquals(AutoDrive.MODE_UNLOAD, 5)
    lu.assertEquals(AutoDrive.CHASEPOS_REAR, 3)
end

--- errorMsg is the escape hatch for genuine errors and stays unconditional.
function TestDebugLayer:testErrorMsgAlwaysLogs()
    AutoDrive.currentDebugChannelMask = 0
    AutoDrive.errorMsg(nil, 'real error')
    lu.assertEquals(Logging.countMatching('real error'), 1)
end

function TestDebugLayer:testGetDebugChannelIsSet()
    AutoDrive.currentDebugChannelMask = AutoDrive.DC_PATHINFO
    lu.assertTrue(AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO))
    lu.assertFalse(AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO))
end


------------------------------------------------------------------------------------------------------------------------
--- math.sign
---
--- The game extends the math table with sign at C level - its own scripts call math.sign a hundred
--- and seventy odd times and none of them defines it - exactly as it does with math.clamp, which
--- this file already fills in for the same reason. UnloadBGATask called MathUtil.sign, which is not
--- a member of anything: "attempt to call a nil value", every time an ARTICULATED vehicle ran a BGA
--- unload. Found by decompiling the shipped engine scripts and checking every engine helper
--- AutoDrive calls against what the engine actually defines.
------------------------------------------------------------------------------------------------------------------------
TestMathSign = {}

function TestMathSign:setUp() TestSetup.reset() end

function TestMathSign:testItIsAvailableAtAll()
    lu.assertEquals(type(math.sign), 'function',
        'the BGA unload steers with this on articulated vehicles')
end

function TestMathSign:testItAnswersTheSignOfANumber()
    lu.assertEquals(math.sign(4.2), 1)
    lu.assertEquals(math.sign(-0.001), -1)
    lu.assertEquals(math.sign(0), 0)
end

--- The one caller passes a rotation speed straight out of a spec, which can be absent.
function TestMathSign:testItSurvivesNil()
    lu.assertEquals(math.sign(nil), 0)
end

--- And the caller must not reach for it on MathUtil, which has no such member.
function TestMathSign:testTheBgaUnloadDoesNotCallItOnMathUtil()
    local f = io.open('../../scripts/Tasks/UnloadBGATask.lua', 'r')
    local src = f:read('*a')
    f:close()

    lu.assertNil(src:match('MathUtil%.sign%('),
        'MathUtil has no sign member - the game puts sign on the math table')
    lu.assertNotNil(src:match('math%.sign%('),
        'and the articulated steering still needs the sign of the axis rotation')
end

os.exit(lu.LuaUnit.run())
