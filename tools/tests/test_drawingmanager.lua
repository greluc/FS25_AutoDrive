--[[
Tests for scripts/Manager/DrawingManager.lua.

Covers finding:
  P7 every draw task looked up scaleLines and lineHeight through AutoDrive.getSetting, which is
     not a pure read - it writes the setting back when the stored index is out of range.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('Buffer')
require('FlaggedTable')
require('DrawingManager')

TestFrameSettingCache = {}

function TestFrameSettingCache:setUp()
    TestSetup.reset()
    AutoDrive.testSettings['scaleLines'] = 2
    AutoDrive.testSettings['lineHeight'] = 4
    AutoDrive.drawHeight = 0
    ADDrawingManager.settingsFrame = nil

    -- count how often the settings layer is consulted
    self.reads = 0
    self.origGetSetting = AutoDrive.getSetting
    AutoDrive.getSetting = function(name, vehicle)
        if name == 'scaleLines' or name == 'lineHeight' then
            self.reads = self.reads + 1
        end
        return self.origGetSetting(name, vehicle)
    end
end

function TestFrameSettingCache:tearDown()
    AutoDrive.getSetting = self.origGetSetting
end

function TestFrameSettingCache:testValuesAreCorrect()
    lu.assertEquals(ADDrawingManager:getScaleLines(), 2)
    lu.assertEquals(ADDrawingManager:getYOffset(), 4)
end

function TestFrameSettingCache:testManyTasksInOneFrameReadTheSettingsOnce()
    g_updateLoopIndex = 100
    for _ = 1, 500 do
        ADDrawingManager:getScaleLines()
        ADDrawingManager:getYOffset()
    end
    lu.assertEquals(self.reads, 2,
        'a frame must consult scaleLines and lineHeight once each, not once per draw task '
        .. '(got ' .. tostring(self.reads) .. ' reads for 1000 task lookups)')
end

function TestFrameSettingCache:testTheCacheRefreshesOnTheNextFrame()
    g_updateLoopIndex = 100
    ADDrawingManager:getScaleLines()
    lu.assertEquals(ADDrawingManager:getScaleLines(), 2)

    AutoDrive.testSettings['scaleLines'] = 7
    lu.assertEquals(ADDrawingManager:getScaleLines(), 2, 'still the same frame, still the old value')

    g_updateLoopIndex = 101
    lu.assertEquals(ADDrawingManager:getScaleLines(), 7,
        'a settings change must take effect on the very next frame')
end

function TestFrameSettingCache:testMissingSettingsDoNotProduceNil()
    AutoDrive.testSettings['scaleLines'] = nil
    AutoDrive.testSettings['lineHeight'] = nil
    g_updateLoopIndex = 200
    lu.assertEquals(ADDrawingManager:getScaleLines(), 1)
    lu.assertEquals(ADDrawingManager:getYOffset(), 0)
end

function TestFrameSettingCache:testDrawHeightIsStillApplied()
    AutoDrive.drawHeight = 1.5
    g_updateLoopIndex = 300
    lu.assertEquals(ADDrawingManager:getYOffset(), 5.5)
end

os.exit(lu.LuaUnit.run())
