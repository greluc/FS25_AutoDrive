--[[
Tests for the task chain in scripts/Tasks/.

The tasks load standalone - they are plain classes over AbstractTask - so most of this file drives
the real :update() loop with stubbed vehicle modules. Only what the mock cannot answer is replaced:
the field/crop queries, the Courseplay bridge and the fill level readers. Everything the findings
are about (timers, state transitions, the arguments actually passed on) runs for real.

Covers findings:
  A7  EmptyHarvesterTask kept running after installing FollowCombineTask
  A8  FollowCombineTask compared the chase side against a constant that does not exist
  A9  shouldWaitForChasePos ran twice per frame, so angleWrongTimer aged at double speed
  A10 UnloadAtDestinationTask's pathfinder retries were dead code
  A11 UnloadBGATask passed an undefined global to getVehicleLeadingEdge
  A12 UnloadBGATask logged an error line every frame while it had no trailer
  A32 UnloadBGATask:checkForIdleCondition was unreachable and broken
  A33 HandleHarvesterTurnTask never assigned the trailers it passed on
  A34 CatchCombinePipeTask preloaded a timer that the next state change wiped
  K5  FollowCombineTask's stuck detector could not tell being stuck from waiting
]]

lu = require('luaunit')
require('test-setup')

------------------------------------------------------------------------------------------------------------------------
--- Engine and game globals the tasks reach for
------------------------------------------------------------------------------------------------------------------------
--- The transform functions are what AutoDrive.worldToLocal and friends delegate to. The test world
--- keeps every vehicle at the origin facing +z, so local and world coordinates are the same.
function worldToLocal(_, x, y, z) return x, y, z end
function localToWorld(_, x, y, z) return x, y, z end
function localDirectionToWorld(_, x, y, z) return x, y, z end

Dischargeable = { DISCHARGE_STATE_OFF = 0, DISCHARGE_STATE_OBJECT = 2 }
ADMessagesManager = { messageTypes = { WARN = 1, ERROR = 2 } }
AutoDriveMessageEvent = { sendMessageOrNotification = function() end }
--- The real CombineUnloaderMode carries mode states and no CHASEPOS_* values - that is finding A8.
--- Nothing is added here that the mod does not have, or the A8 test would pass for the wrong reason.
CombineUnloaderMode = { STATE_ACTIVE_UNLOAD_COMBINE = 3 }

require('UtilFuncs')
require('AutoDriveTON')
require('Queue')
require('AbstractTask')
require('TaskModule')
require('TrailerUtil')
require('AutoDriveUtilFuncs')

require('EmptyHarvesterTask')
require('FollowCombineTask')
require('CatchCombinePipeTask')
require('HandleHarvesterTurnTask')
require('UnloadAtDestinationTask')
require('UnloadBGATask')

------------------------------------------------------------------------------------------------------------------------
--- Stubs for what the mock world cannot answer
------------------------------------------------------------------------------------------------------------------------
--- Field shape, crop density and the Courseplay bridge are outside the mock. Fill levels are read
--- through the vehicle's fill units, which these tests do not model. All of them are inputs to the
--- code under test, never the thing being asserted on.
local Stub = {
    combineIsTurning = false,
    cpTurning = false,
    cpActive = false,
    inCrop = false,
    distanceBetween = 40,
    combineFillLevel = 5000,
    combineCapacity = 10000,
    trailerFilledToUnload = false,
    trailerFreeCapacity = 100,
}

local function installStubs()
    Stub.combineIsTurning = false
    Stub.cpTurning = false
    Stub.cpActive = false
    Stub.inCrop = false
    Stub.distanceBetween = 40
    Stub.combineFillLevel = 5000
    Stub.combineCapacity = 10000
    Stub.trailerFilledToUnload = false
    Stub.trailerFreeCapacity = 100

    AutoDrive.combineIsTurning = function() return Stub.combineIsTurning end
    AutoDrive.getIsCPTurning = function(_, _) return Stub.cpTurning end
    AutoDrive.getIsCPActive = function(_, _) return Stub.cpActive end
    AutoDrive.getIsCPCombineInPocket = function(_, _) return false end
    AutoDrive.holdCPCombine = function(_, _) end
    AutoDrive.isVehicleOrTrailerInCrop = function() return Stub.inCrop end
    AutoDrive.getDistanceBetween = function() return Stub.distanceBetween end
    AutoDrive.getObjectFillLevels = function()
        return Stub.combineFillLevel, Stub.combineCapacity, false, Stub.combineCapacity - Stub.combineFillLevel
    end
    AutoDrive.getAllFillLevels = function()
        return 0, 0, Stub.trailerFilledToUnload, Stub.trailerFreeCapacity
    end
end

------------------------------------------------------------------------------------------------------------------------
--- Vehicle fixtures
------------------------------------------------------------------------------------------------------------------------
local function unloadModeStub()
    return {
        chasePos = { x = 0, y = 0, z = 20 },
        chaseSide = AutoDrive.CHASEPOS_LEFT,
        correctSide = true,
        correctSideForChaseSide = true,
        pipeChaseCalls = 0,
        failedPathFinderCalls = 0,
        waitForCallCalls = 0,
        getPipeChasePosition = function(self)
            self.pipeChaseCalls = self.pipeChaseCalls + 1
            return self.chasePos, self.chaseSide
        end,
        getAngleToCombineHeading = function() return 0 end,
        getAngleToCombine = function() return 0 end,
        isUnloaderOnCorrectSide = function(self, side)
            if side == nil then return self.correctSide end
            return self.correctSideForChaseSide
        end,
        notifyAboutFailedPathfinder = function(self) self.failedPathFinderCalls = self.failedPathFinderCalls + 1 end,
        setToWaitForCall = function(self) self.waitForCallCalls = self.waitForCallCalls + 1 end,
    }
end

--- Mirrors the real special driving module's contract: stopVehicle marks a deliberate hold, every
--- driving call releases it again. FollowCombineTask:isCommandedToDrive reads exactly that flag.
local function drivingModuleStub()
    return {
        stopping = false,
        motorShouldNotBeStopped = false,
        stopCalls = 0,
        driveToPointCalls = 0,
        reverseCalls = 0,
        isStoppingVehicle = function(self) return self.stopping end,
        stopVehicle = function(self) self.stopCalls = self.stopCalls + 1 self.stopping = true end,
        update = function() end,
        driveToPoint = function(self) self.driveToPointCalls = self.driveToPointCalls + 1 self.stopping = false end,
        driveReverse = function(self) self.reverseCalls = self.reverseCalls + 1 self.stopping = false end,
        driveForward = function(self) self.stopping = false end,
    }
end

local function taskModuleStub()
    return {
        added = {},
        finishedCalls = 0,
        abortAllCalls = 0,
        addTask = function(self, task) table.insert(self.added, task) end,
        setCurrentTaskFinished = function(self) self.finishedCalls = self.finishedCalls + 1 end,
        abortAllTasks = function(self) self.abortAllCalls = self.abortAllCalls + 1 end,
    }
end

local function unloader()
    local v = TestSetup.vehicle()
    v.components = { { node = 'unloader' } }
    v.size = { width = 3, length = 6, lengthOffset = 0 }
    v.lastSpeedReal = 0
    v.movingDirection = 1
    v.stopAutoDriveCalls = 0
    v.stopAutoDrive = function(self) self.stopAutoDriveCalls = self.stopAutoDriveCalls + 1 end
    v.ad.modes = {}
    v.ad.modes[AutoDrive.MODE_UNLOAD] = unloadModeStub()
    v.ad.specialDrivingModule = drivingModuleStub()
    v.ad.taskModule = taskModuleStub()
    v.ad.trailerModule = {
        canBeHandledInReverse = function() return false end,
        reset = function() end,
    }
    v.ad.drivePathModule = {
        speedLimit = 0,
        isTargetReached = function() return false end,
        update = function() end,
        setWayPoints = function() end,
        getIsReversing = function() return false end,
    }
    v.ad.stateModule = {
        getName = function() return 'TestDriver' end,
        getFirstWayPoint = function() return 1 end,
        getSecondWayPoint = function() return 2 end,
        getMode = function() return AutoDrive.MODE_UNLOAD end,
        getCanRestartHelper = function() return false end,
    }
    MockEngine.nodePositions['unloader'] = { x = 0, y = 0, z = 0 }
    return v
end

local function harvester()
    local c = TestSetup.vehicle()
    c.components = { { node = 'combine' } }
    c.size = { width = 4, length = 9, lengthOffset = 0 }
    c.lastSpeedReal = 0.01
    c.movingDirection = 1
    c.ad = {
        isChopper = false,
        isHarvester = true,
        isFixedPipeChopper = false,
        isAutoAimingChopper = false,
        isReverseAttached = false,
        ADRootNode = nil,
        driveForwardTimer = { elapsedTime = 0 },
        noMovementTimer = { elapsedTime = 0 },
        stateModule = { getName = function() return 'TestCombine' end },
    }
    c.getDischargeState = function() return Dischargeable.DISCHARGE_STATE_OFF end
    c.getIsAIActive = function() return false end
    MockEngine.nodePositions['combine'] = { x = 0, y = 0, z = 30 }
    g_currentMission.nodeToObject['combine'] = c
    return c
end

--- Reads a source file of the mod. Used where the fix is the absence of something and no unit test
--- can observe it - a deleted function, a name that must not appear again.
local function readSource(name)
    local f = assert(io.open('../../scripts/Tasks/' .. name .. '.lua', 'r'), 'cannot read ' .. name)
    local src = f:read('*a')
    f:close()
    return src
end

--- Runs update() until the condition holds and returns the number of frames it took, so a test can
--- assert on the moment of a transition instead of on whatever state the task drifted into after it.
local function runUntil(task, dt, maxFrames, condition)
    for i = 1, maxFrames do
        task:update(dt)
        if condition(task) then return i end
    end
    return nil
end

local function runFrames(task, frames, dt, beforeEach)
    for i = 1, frames do
        if beforeEach then beforeEach(task, i) end
        task:update(dt)
    end
end

------------------------------------------------------------------------------------------------------------------------
--- A7 - EmptyHarvesterTask must stop its update after handing over to FollowCombineTask
------------------------------------------------------------------------------------------------------------------------
TestEmptyHarvesterHandover = {}

function TestEmptyHarvesterHandover:setUp()
    TestSetup.reset()
    installStubs()
    self.combine = harvester()
    self.vehicle = unloader()
    self.task = EmptyHarvesterTask:new(self.vehicle, self.combine)
    self.task.trailers = { self.vehicle }
    self.task.trailerCount = 1
    self.task.state = EmptyHarvesterTask.STATE_UNLOADING
    self.task.lastState = EmptyHarvesterTask.STATE_UNLOADING
end

--- setCurrentTaskFinished dequeues the task that was just added and makes it the active one. A
--- second call in the same frame therefore finishes the freshly installed FollowCombineTask.
function TestEmptyHarvesterHandover:testHandoverFinishesThisTaskExactlyOnce()
    self.combine.ad.driveForwardTimer.elapsedTime = 5000
    Stub.distanceBetween = 40 -- the fall-through path would call finished() again on this distance

    self.task:update(16)

    lu.assertEquals(#self.vehicle.ad.taskModule.added, 1, 'FollowCombineTask must be queued once')
    lu.assertEquals(self.vehicle.ad.taskModule.finishedCalls, 1,
        'update must return after the handover, otherwise it finishes the new task as well')
end

function TestEmptyHarvesterHandover:testNoHandoverBelowTheTimeThreshold()
    self.combine.ad.driveForwardTimer.elapsedTime = 3000 -- moved, but not for long enough
    Stub.distanceBetween = 40

    self.task:update(16)

    lu.assertEquals(#self.vehicle.ad.taskModule.added, 0)
    lu.assertEquals(self.vehicle.ad.taskModule.finishedCalls, 1, 'the normal "combine drove away" exit still fires')
end

------------------------------------------------------------------------------------------------------------------------
--- A8 / A9 / K5 - FollowCombineTask
------------------------------------------------------------------------------------------------------------------------
TestFollowCombine = {}

function TestFollowCombine:setUp()
    TestSetup.reset()
    installStubs()
    self.combine = harvester()
    self.vehicle = unloader()
    self.mode = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]
    self.task = FollowCombineTask:new(self.vehicle, self.combine)
    self.task:setUp()
end

------------------------------------------------------------------------------------------------------------------------
--- A8 - the chase side is compared against AutoDrive.CHASEPOS_REAR
------------------------------------------------------------------------------------------------------------------------
function TestFollowCombine:testRearChaseSideDoesNotTriggerPassBy()
    self.mode.chaseSide = AutoDrive.CHASEPOS_REAR
    self.mode.correctSideForChaseSide = false -- we are not on the chase side yet

    self.task:update(16)

    lu.assertIs(self.task.state, FollowCombineTask.STATE_CHASING,
        'chasing the rear must not be treated as a side switch')
end

function TestFollowCombine:testSideChaseSwitchTriggersPassBy()
    self.mode.chaseSide = AutoDrive.CHASEPOS_LEFT
    self.mode.correctSideForChaseSide = false

    self.task:update(16)

    lu.assertIs(self.task.state, FollowCombineTask.STATE_WAIT_FOR_PASS_BY,
        'leaving a side chase position must wait for the harvester to pass by')
end

function TestFollowCombine:testNoReferenceToTheNonExistingConstant()
    -- CombineUnloaderMode has no CHASEPOS_* fields; comparing against one is comparing against nil.
    lu.assertNil(CombineUnloaderMode.CHASEPOS_REAR)
    lu.assertNotNil(AutoDrive.CHASEPOS_REAR)
    lu.assertNil(readSource('FollowCombineTask'):find('CombineUnloaderMode.CHASEPOS', 1, true),
        'FollowCombineTask must not read chase side constants off CombineUnloaderMode')
end

------------------------------------------------------------------------------------------------------------------------
--- A9 - the chase position check runs once per frame
------------------------------------------------------------------------------------------------------------------------
function TestFollowCombine:testAngleWrongTimerAgesOncePerFrame()
    -- A chase position 90 degrees off keeps the angle above the 50 degree threshold, so the timer
    -- runs. The first frame ends with the state change that resets every timer, so two frames of
    -- chasing are left: 32 ms when the check runs once, 80 ms when it runs twice.
    self.mode.chasePos = { x = 20, y = 0, z = 0 }

    runFrames(self.task, 3, 16)

    lu.assertEquals(self.task.angleWrongTimer.elapsedTime, 32,
        'angleWrongTimer must advance by dt, not by 2 * dt')
    lu.assertTrue(self.vehicle.ad.specialDrivingModule.driveToPointCalls > 0,
        'the frames under test must actually have gone through followChasePoint')
end

function TestFollowCombine:testAngleWrongTimeoutIsNotHalved()
    self.mode.chasePos = { x = 20, y = 0, z = 0 }
    -- 3000 ms is the configured timeout; at 100 ms per frame it must not be reached in 15 frames.
    runFrames(self.task, 15, 100)

    lu.assertFalse(self.task.angleWrongTimer:done(),
        'the 3000 ms timeout must not fire after 1500 ms of chasing')
end

------------------------------------------------------------------------------------------------------------------------
--- K5 - being stuck means "commanded to drive but standing still"
------------------------------------------------------------------------------------------------------------------------
function TestFollowCombine:testStuckTimeoutIsWithinSecondsNotAMinute()
    lu.assertTrue(FollowCombineTask.MAX_STUCK_TIME >= 5000 and FollowCombineTask.MAX_STUCK_TIME <= 10000,
        'the stuck timeout must be 5-10 s, was ' .. tostring(FollowCombineTask.MAX_STUCK_TIME))
    lu.assertTrue(FollowCombineTask.MIN_STUCK_TIME <= FollowCombineTask.MAX_STUCK_TIME)
end

function TestFollowCombine:testWaitingIsNotStuck()
    -- Standing still because the task parked the vehicle must never age the stuck timer, however
    -- long it lasts. That is what allows the timeout to be short in the first place.
    self.vehicle.lastSpeedReal = 0

    runFrames(self.task, 600, 16, function(task)
        task.vehicle.ad.specialDrivingModule.stopping = true
    end)

    lu.assertEquals(self.task.stuckReactions, 0, 'a deliberate hold must not count as being stuck')
    lu.assertEquals(self.task.stuckTimer.elapsedTime, 0)
    lu.assertIs(self.task.state, FollowCombineTask.STATE_CHASING)
end

function TestFollowCombine:testStandingStillWhileDrivingIsStuck()
    self.vehicle.lastSpeedReal = 0

    local frames = runUntil(self.task, 16, 700, function(t) return t.stuckReactions >= 1 end)

    lu.assertNotNil(frames, 'the driver must react to being stuck within 10 s')
    lu.assertTrue(frames * 16 <= FollowCombineTask.MAX_STUCK_TIME + 100,
        'the reaction must come at the configured timeout')
end

function TestFollowCombine:testMovingIsNotStuck()
    self.vehicle.lastSpeedReal = 2

    runFrames(self.task, 600, 16)

    lu.assertEquals(self.task.stuckReactions, 0)
end

function TestFollowCombine:testFirstReactionRecomputesTheChasePosition()
    self.vehicle.lastSpeedReal = 0
    local callsBefore = self.mode.pipeChaseCalls

    lu.assertNotNil(runUntil(self.task, 16, 700, function(t) return t.stuckReactions >= 1 end))

    lu.assertEquals(self.task.stuckReactions, 1)
    lu.assertIs(self.task.state, FollowCombineTask.STATE_CHASING,
        'the first reaction keeps chasing with a fresh chase position')
    lu.assertTrue(self.mode.pipeChaseCalls > callsBefore)
end

function TestFollowCombine:testSecondReactionIsAShortRetreat()
    self.vehicle.lastSpeedReal = 0

    lu.assertNotNil(runUntil(self.task, 16, 1400, function(t) return t.stuckReactions >= 2 end))

    lu.assertEquals(self.task.stuckReactions, 2)
    lu.assertIs(self.task.state, FollowCombineTask.STATE_REVERSING)
    lu.assertEquals(self.task.reverseDistance, FollowCombineTask.RETREAT_DISTANCE,
        'the second reaction retreats a few metres, not the full reverse distance')
    lu.assertTrue(self.task.resumeChasingAfterReverse, 'the short retreat goes back to chasing')
end

function TestFollowCombine:testRetreatReturnsToChasing()
    self.task.state = FollowCombineTask.STATE_REVERSING
    self.task.startedChasing = true
    self.task.resumeChasingAfterReverse = true
    self.task.reverseDistance = FollowCombineTask.RETREAT_DISTANCE
    self.task.reverseStartLocation = { x = 0, y = 0, z = 0 }
    MockEngine.nodePositions['unloader'] = { x = 0, y = 0, z = -10 } -- retreated far enough

    self.task:update(16)

    lu.assertIs(self.task.state, FollowCombineTask.STATE_CHASING)
    lu.assertFalse(self.task.resumeChasingAfterReverse)
    lu.assertEquals(self.task.reverseDistance, FollowCombineTask.MAX_REVERSE_DISTANCE)
end

function TestFollowCombine:testThirdReactionIsTheFullReverse()
    self.vehicle.lastSpeedReal = 0
    self.task.stuckReactions = 2 -- two reactions already spent

    lu.assertNotNil(runUntil(self.task, 16, 700, function(t) return t.stuckReactions >= 3 end))

    lu.assertEquals(self.task.stuckReactions, 3)
    lu.assertIs(self.task.state, FollowCombineTask.STATE_REVERSING)
    lu.assertEquals(self.task.reverseDistance, FollowCombineTask.MAX_REVERSE_DISTANCE)
    lu.assertFalse(self.task.resumeChasingAfterReverse, 'the last reaction hands the job back to the mode')
end

function TestFollowCombine:testChopperReversesAwayInsteadOfRetreating()
    self.combine.ad.isChopper = true
    self.vehicle.lastSpeedReal = 0
    self.task.stuckReactions = 1 -- the fresh chase position did not help
    -- inside MAX_REVERSE_DISTANCE, otherwise the reverse is already complete on the same frame
    MockEngine.nodePositions['combine'] = { x = 0, y = 0, z = 12 }

    lu.assertNotNil(runUntil(self.task, 16, 700, function(t) return t.stuckReactions >= 2 end))

    lu.assertIs(self.task.state, FollowCombineTask.STATE_REVERSING_FROM_CHOPPER,
        'a chopper is left alone completely instead of being retreated from and chased again')
end

------------------------------------------------------------------------------------------------------------------------
--- K5 - the safety distance comes from the machines, not from a fixed 25 m
------------------------------------------------------------------------------------------------------------------------
function TestFollowCombine:testMinCombineDistanceFollowsTheTrain()
    self.task.tractorTrainLength = 6
    local short = self.task:getMinCombineDistance()
    self.task.tractorTrainLength = 26
    local long = self.task:getMinCombineDistance()

    lu.assertTrue(long > short, 'a longer train needs more room')
    lu.assertEquals(short, (6 + self.combine.size.length) / 2 + FollowCombineTask.COMBINE_DISTANCE_MARGIN)
    lu.assertEquals(long, (26 + self.combine.size.length) / 2 + FollowCombineTask.COMBINE_DISTANCE_MARGIN)
end

function TestFollowCombine:testMinCombineDistanceFollowsTheHarvester()
    self.task.tractorTrainLength = 6
    local small = self.task:getMinCombineDistance()
    self.combine.size.length = 20
    local big = self.task:getMinCombineDistance()

    lu.assertTrue(big > small, 'a bigger harvester needs more room')
end

function TestFollowCombine:testSetUpMeasuresTheTrain()
    lu.assertEquals(self.task.tractorTrainLength, self.vehicle.size.length,
        'setUp must measure the train once so getMinCombineDistance has a value')
end

function TestFollowCombine:testTooCloseShortensTheReaction()
    -- Inside the safety distance the driver must react after MIN_STUCK_TIME instead of MAX_STUCK_TIME.
    self.vehicle.lastSpeedReal = 0
    MockEngine.nodePositions['combine'] = { x = 0, y = 0, z = 5 } -- well inside the safety distance

    local maxFrames = math.floor((FollowCombineTask.MIN_STUCK_TIME + 500) / 16)
    lu.assertNotNil(runUntil(self.task, 16, maxFrames, function(t) return t.stuckReactions >= 1 end),
        'being stuck against the harvester must not wait out the full timeout')
end

function TestFollowCombine:testDistanceConditionIsNoLongerCommentedOut()
    local src = readSource('FollowCombineTask')
    lu.assertNil(src:find('-- or AutoDrive.getDistanceBetween', 1, true),
        'the disabled emergency distance check must be gone, not left commented out')
    lu.assertNotNil(src:find('getMinCombineDistance', 1, true),
        'the emergency distance must be evaluated from the machines')
end

------------------------------------------------------------------------------------------------------------------------
--- A10 - UnloadAtDestinationTask retries before it shuts the driver down
------------------------------------------------------------------------------------------------------------------------
TestUnloadAtDestinationRetry = {}

function TestUnloadAtDestinationRetry:setUp()
    TestSetup.reset()
    installStubs()
    self.vehicle = unloader()
    self.pathFinder = {
        restarts = 0,
        delays = 0,
        targetBlocked = false,
        didTimeOut = false,
        blocked = false,
        hasFinished = function() return true end,
        getPath = function() return {} end,
        isTargetBlocked = function(self) return self.targetBlocked end,
        timedOut = function(self) return self.didTimeOut end,
        isBlocked = function(self) return self.blocked end,
        reset = function() end,
        startPathPlanningToNetwork = function(self) self.restarts = self.restarts + 1 end,
        addDelayTimer = function(self) self.delays = self.delays + 1 end,
        update = function() end,
    }
    self.vehicle.ad.pathFinderModule = self.pathFinder
    self.task = UnloadAtDestinationTask:new(self.vehicle, 7)
    self.task.trailers = { self.vehicle }
    self.task.failedPathFinder = 0
    self.task.state = UnloadAtDestinationTask.STATE_PATHPLANNING
end

function TestUnloadAtDestinationRetry:testFailuresRetryInsteadOfShuttingDown()
    for _ = 1, UnloadAtDestinationTask.MAX_FAILED_PATHFINDER do
        self.task:update(16)
    end

    lu.assertEquals(self.pathFinder.restarts, UnloadAtDestinationTask.MAX_FAILED_PATHFINDER,
        'every failure below the limit must restart the pathfinder')
    lu.assertEquals(self.vehicle.stopAutoDriveCalls, 0, 'the driver must not be shut down while retries are left')
    lu.assertEquals(self.vehicle.ad.taskModule.abortAllCalls, 0)
    lu.assertEquals(Logging.countMatching('Could not calculate path'), 0)
    lu.assertIs(self.task.state, UnloadAtDestinationTask.STATE_PATHPLANNING)
end

function TestUnloadAtDestinationRetry:testShutdownOnceTheRetriesAreUsedUp()
    for _ = 1, UnloadAtDestinationTask.MAX_FAILED_PATHFINDER + 1 do
        self.task:update(16)
    end

    lu.assertEquals(self.vehicle.stopAutoDriveCalls, 1, 'the driver still gives up eventually')
    lu.assertEquals(self.vehicle.ad.taskModule.abortAllCalls, 1)
    lu.assertEquals(Logging.countMatching('Could not calculate path'), 1)
end

function TestUnloadAtDestinationRetry:testBlockedTargetRetriesTheClosestExit()
    self.pathFinder.targetBlocked = true

    self.task:update(16)

    lu.assertEquals(self.pathFinder.restarts, 1)
    lu.assertEquals(self.pathFinder.delays, 0, 'a blocked target is retried without the extra delay')
    lu.assertEquals(self.vehicle.stopAutoDriveCalls, 0)
end

function TestUnloadAtDestinationRetry:testTimedOutPathfinderRetriesWithDelay()
    self.pathFinder.didTimeOut = true

    self.task:update(16)

    lu.assertEquals(self.pathFinder.restarts, 1)
    lu.assertEquals(self.pathFinder.delays, 1, 'a timeout gets time to clear itself')
    lu.assertEquals(self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD].failedPathFinderCalls, 1)
end

function TestUnloadAtDestinationRetry:testSuccessfulPathLeavesPathPlanning()
    self.pathFinder.getPath = function() return { { x = 0, y = 0, z = 1 } } end

    self.task:update(16)

    lu.assertIs(self.task.state, UnloadAtDestinationTask.STATE_DRIVING)
    lu.assertEquals(self.vehicle.stopAutoDriveCalls, 0)
end

------------------------------------------------------------------------------------------------------------------------
--- A11 / A12 / A32 - UnloadBGATask
------------------------------------------------------------------------------------------------------------------------
TestUnloadBGA = {}

function TestUnloadBGA:setUp()
    TestSetup.reset()
    installStubs()
    self.vehicle = unloader()
    self.task = UnloadBGATask:new(self.vehicle)
end

--- Guard: the callee really does log an error for a nil object. Without this the test below would
--- pass even if someone silenced the message in TrailerUtil instead of fixing the call site.
function TestUnloadBGA:testFillLevelReaderStillReportsANilObject()
    AutoDrive.getObjectNonFuelFillLevels(nil)
    lu.assertEquals(Logging.countMatching('getObjectNonFuelFillLevels object == nil'), 1)
end

function TestUnloadBGA:testNoLogSpamWhileWaitingForATrailer()
    self.task.targetTrailer = nil
    self.task.shovel = nil

    for _ = 1, 200 do
        self.task:getCurrentStates()
    end

    lu.assertEquals(Logging.countMatching('getObjectNonFuelFillLevels object == nil'), 0,
        'having no trailer yet is normal and must not write an error line every frame')
    lu.assertEquals(self.task.trailerLeftCapacity, 0, 'no trailer means no free capacity')
end

function TestUnloadBGA:testRestartCheckDoesNotSpamEither()
    self.task.targetTrailer = nil
    self.task.targetBunker = {}
    self.task.targetUnloadTrigger = {}
    self.task.findCloseTrailer = function() return nil, nil end

    for _ = 1, 200 do
        self.task:checkIfPossibleToRestart()
    end

    lu.assertEquals(Logging.countMatching('getObjectNonFuelFillLevels object == nil'), 0)
end

function TestUnloadBGA:testDeadIdleConditionIsGone()
    lu.assertNil(UnloadBGATask.checkForIdleCondition,
        'checkForIdleCondition was unreachable and dereferenced self.self')
    lu.assertNil(readSource('UnloadBGATask'):find('self.self', 1, true))
end

function TestUnloadBGA:testLeadingEdgeIsMeasuredOnTheTaskVehicle()
    local src = readSource('UnloadBGATask')
    lu.assertNil(src:find('getVehicleLeadingEdge(vehicle)', 1, true),
        'the undefined global "vehicle" made the leading edge always 0')
    lu.assertNotNil(src:find('getVehicleLeadingEdge(self.vehicle)', 1, true))
end

--- The leading edge only ever differs from 0 for a real vehicle, which is what the call site now
--- passes. With nil it silently returns 0 - the value the mod used for three target points.
function TestUnloadBGA:testLeadingEdgeOfNothingIsZero()
    lu.assertEquals(AutoDrive.getVehicleLeadingEdge(nil), 0)
end

------------------------------------------------------------------------------------------------------------------------
--- A33 - HandleHarvesterTurnTask assigns the units it opens the covers on
------------------------------------------------------------------------------------------------------------------------
TestHandleHarvesterTurn = {}

function TestHandleHarvesterTurn:setUp()
    TestSetup.reset()
    installStubs()
    self.vehicle = unloader()
    self.task = HandleHarvesterTurnTask:create()
    self.task.vehicle = self.vehicle
    self.task.combine = harvester()
end

function TestHandleHarvesterTurn:testSetUpPassesTheRealUnitsOn()
    local recorded = nil
    local original = AutoDrive.setTrailerCoverOpen
    AutoDrive.setTrailerCoverOpen = function(_, trailers, open) recorded = { trailers = trailers, open = open } end

    lu.assertNil(self.task.trailers, 'nothing assigns trailers before setUp')
    self.task:setUp()
    AutoDrive.setTrailerCoverOpen = original

    lu.assertNotNil(self.task.trailers, 'setUp must determine the units')
    lu.assertNotNil(recorded, 'setUp still opens the covers')
    lu.assertIs(recorded.trailers, self.task.trailers)
    lu.assertIs(recorded.trailers[1], self.vehicle)
    lu.assertTrue(recorded.open)
end

--- The callee returns without doing anything when it gets nothing, which is why the bug was silent.
function TestHandleHarvesterTurn:testCoverOpenIsANoOpWithoutUnits()
    AutoDrive.testSettings['autoTrailerCover'] = true
    local trailer = { spec_cover = { state = 0, covers = { 1 }, }, getIsNextCoverStateAllowed = function() return true end }
    local switched = 0
    trailer.setCoverState = function() switched = switched + 1 end

    AutoDrive.setTrailerCoverOpen(self.vehicle, nil, true)
    lu.assertEquals(switched, 0)

    AutoDrive.setTrailerCoverOpen(self.vehicle, { trailer }, true)
    lu.assertEquals(switched, 1, 'with units the covers actually move')
end

------------------------------------------------------------------------------------------------------------------------
--- A34 - CatchCombinePipeTask asks for a path right away instead of preloading a timer
------------------------------------------------------------------------------------------------------------------------
TestCatchCombinePipe = {}

function TestCatchCombinePipe:setUp()
    TestSetup.reset()
    installStubs()
    self.combine = harvester()
    self.vehicle = unloader()
    self.task = CatchCombinePipeTask:new(self.vehicle, self.combine)
    self.pathFindingCalls = 0
    --- Stands in for the "chase position looks bad, wait and look again" outcome of the real
    --- method, which resets the delay timer exactly like this before returning false.
    self.task.startNewPathFinding = function(task)
        self.pathFindingCalls = self.pathFindingCalls + 1
        task.waitForCheckTimer:timer(false)
        return false
    end
end

function TestCatchCombinePipe:testTimerIsNotPreloaded()
    lu.assertEquals(self.task.waitForCheckTimer.elapsedTime, 0,
        'preloading the timer had no effect - the state change reset it one frame later')
    lu.assertTrue(self.task.checkPathPlanningNow, 'the intent is an immediate first check')
end

function TestCatchCombinePipe:testFirstCheckHappensOnTheFirstFrame()
    self.task:update(16)

    lu.assertEquals(self.pathFindingCalls, 1, 'the driver must not stand around for the delay first')
end

function TestCatchCombinePipe:testFollowingChecksWaitForTheDelay()
    self.task:update(16)
    self.task:update(16)
    lu.assertEquals(self.pathFindingCalls, 1, 'the delay applies to every check after the first')

    runFrames(self.task, 70, 16) -- > 1000 ms
    lu.assertEquals(self.pathFindingCalls, 2)
end

function TestCatchCombinePipe:testCombineTravelledRestartsWithoutDelay()
    self.task.state = CatchCombinePipeTask.STATE_DRIVING
    self.task.lastState = CatchCombinePipeTask.STATE_DRIVING
    self.task.checkPathPlanningNow = false -- the immediate check from the start is long used up
    self.task.combinesStartLocation = { x = 0, y = 0, z = 0 }
    MockEngine.nodePositions['combine'] = { x = 0, y = 0, z = 200 }

    self.task:update(16)
    lu.assertIs(self.task.state, CatchCombinePipeTask.STATE_DELAY_PATHPLANNING)
    lu.assertTrue(self.task.checkPathPlanningNow)

    self.task:update(16)
    lu.assertEquals(self.pathFindingCalls, 1, 'a path that lost its target is replaced immediately')
end

os.exit(lu.LuaUnit.run())
