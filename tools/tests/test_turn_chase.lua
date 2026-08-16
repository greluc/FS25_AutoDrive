--[[
Regression test: do not drive at the chase point while the harvester is turning away from us.

Found in the game. The unloader drove into the JAGUAR from behind during its turn.

Measured in the log of that run: 419 followChasePoint calls in STATE_WAIT_FOR_TURN, all with
chaseSide 3 (CHASEPOS_REAR), and getAngleToChasePos steady at 177 degrees. The harvester had swung
around, dragging its rear chase position BEHIND the unloader - so "drive towards the chase point"
meant turning into a harvester that was swinging back at the same time.

waitForChasePos does not protect against this: it is only recomputed in the chasing state, so
during the turn it keeps whatever value it had when the turn began.

This is pre-existing behaviour, not a regression - our diff in that branch was one debug guard.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('AutoDriveTON')
require('FollowCombineTask')

local function task(angleToChasePos)
    local o = setmetatable({}, { __index = FollowCombineTask })
    o.vehicle = TestSetup.vehicle()
    o.getAngleToChasePos = function() return angleToChasePos end
    return o
end

TestTurnChaseGuard = {}

function TestTurnChaseGuard:setUp() TestSetup.reset() end

function TestTurnChaseGuard:testTheMeasuredFailureAngleIsRefused()
    lu.assertFalse(task(177):isChasePointReachableDuringTurn(),
        '177 degrees is the angle measured while the unloader drove into the harvester - the chase '
        .. 'point was directly behind it')
end

function TestTurnChaseGuard:testAChasePointAheadIsStillFollowed()
    lu.assertTrue(task(0):isChasePointReachableDuringTurn())
    lu.assertTrue(task(30):isChasePointReachableDuringTurn(),
        'a harvester turning slightly away must not stop the unloader dead - that would make it '
        .. 'lose contact on every headland')
    lu.assertTrue(task(89):isChasePointReachableDuringTurn())
end

function TestTurnChaseGuard:testTheBoundaryIsInclusive()
    lu.assertTrue(task(90):isChasePointReachableDuringTurn())
    lu.assertFalse(task(91):isChasePointReachableDuringTurn())
end

function TestTurnChaseGuard:testSidewaysIsTheCutOff()
    -- the rule in one sentence: anything no longer in front of us is refused
    for _, angle in ipairs({ 95, 120, 150, 180 }) do
        lu.assertFalse(task(angle):isChasePointReachableDuringTurn(),
            angle .. ' degrees is behind the unloader and must not be driven to during a turn')
    end
end

------------------------------------------------------------------------------------------------------------------------
--- Every frame of the state has to leave the vehicle with a drive command or a brake
---
--- All of the driving in STATE_WAIT_FOR_TURN sat inside `if AutoDrive.combineIsTurning(...)`. When
--- that went false the branch fell straight through to the release conditions and issued nothing at
--- all - no drive, no brake, and no specialDrivingModule:update, so nothing reached the vehicle for
--- that frame. And it is the ordinary case, not an edge one: the turn flag drops as soon as the
--- harvester has not moved for three seconds, which is exactly what a full AI or Courseplay
--- harvester does while it waits for us. Measured up to fourteen seconds of consecutive silent
--- frames, ended only by the fifteen second turn timeout.
------------------------------------------------------------------------------------------------------------------------
TestWaitForTurnAlwaysCommands = {}

local commands

local function waitingForTurn()
    local o = setmetatable({}, { __index = FollowCombineTask })
    o.vehicle = TestSetup.vehicle()
    o.combine = TestSetup.vehicle()
    o.combine.components = { { node = 'combineNode' } }
    g_currentMission.nodeToObject = { combineNode = o.combine }

    o.state = FollowCombineTask.STATE_WAIT_FOR_TURN
    o.lastState = FollowCombineTask.STATE_WAIT_FOR_TURN   -- no transition, so no timer reset
    o.waitForTurnTimer = AutoDriveTON:new()
    o.stuckTimer = AutoDriveTON:new()
    o.angleToCombineHeading = 0
    o.angleToCombine = 0
    o.distanceToCombine = 40
    FollowCombineTask.setStateNames(o)

    -- The state's own geometry is fixed above rather than recomputed, so what is under test is the
    -- branch and not the twenty other things updateStates reaches for.
    o.updateStates = function() end
    o.isCommandedToDrive = function() return false end
    o.hasPendingBackupRequest = function() return false end
    o.getMinCombineDistance = function() return 10 end

    commands = { drive = 0, stop = 0, update = 0 }
    o.vehicle.ad.specialDrivingModule = {
        stopVehicle = function() commands.stop = commands.stop + 1 end,
        update = function() commands.update = commands.update + 1 end,
        driveReverse = function() commands.drive = commands.drive + 1 end,
        releaseVehicle = function() end,
    }
    o.followChasePoint = function() commands.drive = commands.drive + 1 end
    o.reverse = function() commands.drive = commands.drive + 1 end

    -- the release conditions below the branch stay shut, so the state is actually held
    o.combine.ad.sensors = { frontSensorFruit = { pollInfo = function() return false end } }
    o.combine.ad.driveForwardTimer = { elapsedTime = 0 }
    o.combine.ad.isHarvester = true
    o.combine.ad.isChopper = false
    return o
end

function TestWaitForTurnAlwaysCommands:setUp()
    TestSetup.reset()
    self.turning = true
    AutoDrive.combineIsTurning = function() return self.turning end
end

--- The control: while it really is turning, the state has always braked.
function TestWaitForTurnAlwaysCommands:testItBrakesWhileTheHarvesterTurns()
    local t = waitingForTurn()
    self.turning = true

    t:update(16)

    lu.assertTrue(commands.stop + commands.drive > 0, 'test setup: the turning case commands something')
    lu.assertEquals(commands.update, 1, 'and applies it in the same frame')
end

--- And the case that issued nothing: the harvester has stopped turning but has not released us yet.
function TestWaitForTurnAlwaysCommands:testItStillBrakesOnceTheHarvesterStopsTurning()
    local t = waitingForTurn()
    self.turning = false

    for _ = 1, 20 do
        t:update(16)
    end

    lu.assertEquals(t.state, FollowCombineTask.STATE_WAIT_FOR_TURN,
        'test setup: the release conditions have to still be shut for this to mean anything')
    lu.assertEquals(commands.update, 20,
        'every frame of the state has to reach the vehicle with something, or it coasts')
    lu.assertEquals(commands.stop, 20, 'and with nothing else to do that something is the brake')
end

--- Source level: both followChasePoint calls in the turn branch must sit behind the guard, or the
--- drive command is issued before anything checks the angle.
function TestTurnChaseGuard:testBothTurnBranchesAreGuarded()
    local src = io.open('../../scripts/Tasks/FollowCombineTask.lua', 'r'):read('*a')
    local turnBranch = src:match('STATE_WAIT_FOR_TURN then(.-)elseif self%.state == FollowCombineTask%.STATE_REVERSING')
    lu.assertNotNil(turnBranch, 'could not isolate the STATE_WAIT_FOR_TURN branch')

    local calls = select(2, turnBranch:gsub('self:followChasePoint%(dt%)', ''))
    local guards = select(2, turnBranch:gsub('isChasePointReachableDuringTurn%(%)', ''))
    lu.assertTrue(calls > 0, 'expected followChasePoint calls in the turn branch')
    lu.assertEquals(guards, calls,
        string.format('%d followChasePoint call(s) in the turn branch but %d guard(s)', calls, guards))
end

os.exit(lu.LuaUnit.run())
