--[[
Sweeps over the configuration space, rather than a point in it.

Review rounds of this work kept finding real defects, and the reason was never subtle: every
geometric fixture pinned ONE configuration, and it was always the flattering one. The vehicle stood
exactly abeam a way point, so the offset from it was purely lateral and any direction rule looked
right. Way points were always four metres apart, so a term that vanishes at four metres vanished
everywhere. Routes always ran along an axis. Several defects survived a green suite that way.

The sweeps live in sweep-lib.lua and call the production functions; this file asserts their violation
counts are zero. test_sweeps_have_teeth.lua runs the SAME functions against the historical broken
rules and asserts they are not - which is the only thing that makes any of this worth having.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('AutoDriveTON')
require('SortedQueue')
require('PathCalculation')
require('GraphManager')
require('DrivePathModule')
require('CollisionDetectionModule')
require('ExternalInterface')
require('AbstractTask')
require('WaitForCallTask')
require('sweep-lib')

--- The rule turnAngle replaced is transcribed once, in sweep-lib, so both this file and the proof
--- that these sweeps have teeth compare against the same thing.

------------------------------------------------------------------------------------------------------------------------
TestSweeps = {}

function TestSweeps:setUp()
    TestSetup.reset()
    g_time = 10000
    AutoDrive.testSettings['cornerSpeed'] = 1.0
    AutoDrive.testSettings['waitingPosition'] = true
end

--- Stepping aside has to end up clear of the route, from anywhere along it, on either side, at any
--- spacing, whichever way the route runs, and wherever the asker stands.
function TestSweeps:testTheStepAsideAlwaysLeavesTheRoute()
    local violations, checked, worstCase = Sweeps.escapeLeavesTheRoute()

    lu.assertTrue(checked > 1000, string.format('only %d configurations swept', checked))
    lu.assertEquals(violations, 0, string.format('%d of %d configurations stayed on the route, worst: %s',
        violations, checked, tostring(worstCase)))
end

--- And it has to open a gap for whoever asked. Leaving the route while driving AT the asker frees
--- nothing and stalls on its own front sensor.
function TestSweeps:testTheStepAsideOpensAGapForTheAsker()
    local violations, checked, worstCase = Sweeps.escapeOpensAGapForTheAsker()

    lu.assertTrue(checked > 500, string.format('only %d configurations swept', checked))
    lu.assertEquals(violations, 0, string.format('%d of %d closed on the asker, worst: %s',
        violations, checked, tostring(worstCase)))
end

function TestSweeps:testACoarserRecordingIsNeverFaster()
    local violations, checked, worstCase = Sweeps.coarserRecordingIsNeverFaster()

    lu.assertTrue(checked > 50, string.format('only %d configurations swept', checked))
    lu.assertEquals(violations, 0, string.format('%d of %d were faster when recorded coarsely: %s',
        violations, checked, tostring(worstCase)))
end

function TestSweeps:testTheLateralBudgetIsRespected()
    local violations, checked, worstCase = Sweeps.lateralBudgetIsRespected()

    lu.assertTrue(checked > 15, string.format('only %d configurations swept', checked))
    lu.assertEquals(violations, 0, string.format('%d of %d exceeded the budget: %s',
        violations, checked, tostring(worstCase)))
end

function TestSweeps:testTurnAngleMatchesTheRuleItReplaced()
    local violations, checked, worstCase = Sweeps.turnAngleMatchesTheOriginal(Sweeps.theOriginalTurnAngleRule)

    lu.assertTrue(checked > 400, string.format('only %d configurations swept', checked))
    lu.assertEquals(violations, 0, string.format('%d of %d differ, first: %s',
        violations, checked, tostring(worstCase)))
end

function TestSweeps:testExactlyOneVehicleYieldsAtEveryCrossing()
    local violations, checked, worstCase = Sweeps.exactlyOneYields()

    lu.assertTrue(checked > 30, string.format('only %d crossings swept', checked))
    lu.assertEquals(violations, 0, string.format('%d of %d crossings had both or neither yield: %s',
        violations, checked, tostring(worstCase)))
end

function TestSweeps:testEveryReverseTargetIsOneTheControllerWillDriveTo()
    local violations, chosen, worstCase = Sweeps.reverseTargetsAreDrivableTo()

    lu.assertTrue(chosen > 0, 'the sweep has to actually choose reverse sometimes, or it proves nothing')
    lu.assertEquals(violations, 0, string.format('%d of %d reverse choices would be refused: %s',
        violations, chosen, tostring(worstCase)))
end

os.exit(lu.LuaUnit.run())
