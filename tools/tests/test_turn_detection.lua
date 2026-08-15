--[[
Tests for AutoDrive.combineIsTurning.

Covers a finding from investigating the turn behaviour: a Courseplay turn that pauses for more
than 3 s was reported as "no longer turning".

combineIsTurning drops the turn flag once the harvester has stood still for 3 s. That heuristic is
aimed at Giants AI helpers, which do not pause mid-turn - standing still there means the turn is
over. A Courseplay turn legitimately pauses: reversing into a pocket, waiting for room, holding for
an unloader. When the flag dropped, FollowCombineTask left STATE_WAIT_FOR_TURN and resumed the
chase into a harvester that was about to swing.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('AutoDriveUtilFuncs')

local function harvester(noMovementMs, opts)
    opts = opts or {}
    local v = TestSetup.vehicle()
    v.ad.noMovementTimer = { elapsedTime = noMovementMs }
    v.ad.isChopper = opts.isChopper or false
    v.getAIFieldWorkerIsTurning = function() return opts.aiTurning or false end
    return v
end

TestTurnDetection = {}

function TestTurnDetection:setUp()
    TestSetup.reset()
    -- keep the field geometry out of it; those branches are covered by their own helpers
    AutoDrive.checkIsOnField = function() return false end
    AutoDrive.getLengthOfFieldInFront = function() return 100 end
end

function TestTurnDetection:testCourseplayTurnSurvivesALongPause()
    AutoDrive.getIsCPTurning = function() return true end
    lu.assertTrue(AutoDrive.combineIsTurning(harvester(9000)),
        'a Courseplay turn that pauses longer than 3 s is still a turn - dropping the flag sends '
        .. 'the unloader back into a harvester that is about to swing')
end

function TestTurnDetection:testCourseplayTurnIsReportedWhileMoving()
    AutoDrive.getIsCPTurning = function() return true end
    lu.assertTrue(AutoDrive.combineIsTurning(harvester(0)))
end

--- The 3 s rule stays for AI helpers, which do not pause mid-turn.
function TestTurnDetection:testAiTurnStillTimesOutOnStandstill()
    AutoDrive.getIsCPTurning = function() return false end
    lu.assertFalse(AutoDrive.combineIsTurning(harvester(9000, { aiTurning = true })),
        'a Giants AI helper standing still for 3 s has finished its turn')
end

function TestTurnDetection:testAiTurnIsReportedWhileMoving()
    AutoDrive.getIsCPTurning = function() return false end
    lu.assertTrue(AutoDrive.combineIsTurning(harvester(500, { aiTurning = true })))
end

function TestTurnDetection:testNoTurnAtAll()
    AutoDrive.getIsCPTurning = function() return false end
    lu.assertFalse(AutoDrive.combineIsTurning(harvester(0)))
end

os.exit(lu.LuaUnit.run())
