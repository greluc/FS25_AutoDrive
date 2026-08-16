--[[
Tests for the vehicle modules in scripts/Modules/.

Covers findings:
  A2  DrivePathModule re-based the speed limit every frame, throwing away the refinement the
      throttled block had just computed - corners were driven at full road speed 3 frames in 4
  A13 TaskModule cleared 'combine'/'followingUnloader' on itself instead of on the unload mode
  A14 StateModule:previousMode wrapped a locomotive into MODE_UNLOAD
  A35 TrailerModule re-tested the identical condition, so the SEEDS/FERTILIZER fallback was dead
  A36 TrainModule called setCruiseControlState/updateVehiclePhysics on the module, not the vehicle
  A37 CollisionDetectionModule ran every vehicle's all-vehicle scans on the same frame
  A38 checkReverseCollision's 'vehicle.trailer ~= nil' guard was always true

The modules are loaded standalone and driven directly. None of them does load-time work that needs
the game, but all of them expect a fully wired vehicle at call time, so each test builds the
smallest vehicle stub the code path under test actually reads.
]]

lu = require('luaunit')
require('test-setup')
require('DrivePathModule')
require('TaskModule')
require('StateModule')
require('TrailerModule')
require('TrainModule')
require('CollisionDetectionModule')
require('AutoDriveTON')
require('SpecialDrivingModule')

------------------------------------------------------------------------------------------------------------------------
--- Globals the engine provides and mock-engine.lua does not (yet) - defined here so this file
--- stays self-contained. They belong in the mock; noted in the report.
------------------------------------------------------------------------------------------------------------------------
if math.clamp == nil then
    math.clamp = function(v, lo, hi) return math.min(math.max(v, lo), hi) end
end
Lights = Lights or { TURNLIGHT_OFF = 0, TURNLIGHT_LEFT = 1, TURNLIGHT_RIGHT = 2 }
Locomotive = Locomotive or { STATE_MANUAL_TRAVEL_ACTIVE = 1, STATE_MANUAL_TRAVEL_INACTIVE = 2 }

--- Debug output is noise for these tests, but the functions have to exist to be called.
AutoDrive.debugPrint = function() end
AutoDrive.debugMsg = function() end
AutoDrive.getDebugChannelIsSet = function() return false end
AutoDrive.Hud = { lastUIScale = 0 }

--- An object built on a module table without running its constructor: the constructors reach far
--- into the game, the methods under test do not.
local function instanceOf(class, fields)
    local o = setmetatable(fields or {}, { __index = class })
    return o
end

------------------------------------------------------------------------------------------------------------------------
--- A2 - the speed limit refinement has to survive the frames on which the gate does not run
------------------------------------------------------------------------------------------------------------------------
TestDrivePathSpeedLimit = {}

local ROAD_LIMIT = 50

--- Everything followWaypoints touches, and a record of what it handed to driveInDirection.
local function makeDrivePathModule(vehicleId)
    local drive = { calls = {} }

    local vehicle = TestSetup.vehicle({
        id = vehicleId or 1,
        lastSpeedReal = 0,
        setTurnLightState = function() end,
    })
    vehicle.ad.stateModule = {
        speedLimit = ROAD_LIMIT,
        getSpeedLimit = function(s) return s.speedLimit end,
        getFieldSpeedLimit = function() return 20 end,
        setCurrentWayPointId = function() end,
        setNextWayPointId = function() end,
    }
    vehicle.ad.collisionDetectionModule = { hasDetectedObstable = function() return false end }
    vehicle.ad.specialDrivingModule = {
        releaseVehicle = function() end,
        stopVehicle = function() end,
        update = function() end,
        shouldStopMotor = function() return false end,
    }
    vehicle.ad.trailerModule = {
        handleTrailerReversing = function() end,
        isUnloadingToBunkerSilo = function() return false end,
        getBunkerSiloSpeed = function() return 8 end,
    }
    vehicle.ad.taskModule = { getActiveTask = function() return {} end }

    local module = instanceOf(ADDrivePathModule, {
        vehicle = vehicle,
        wayPoints = TestSetup.lineNetwork(6),
        currentWayPoint = 1,
        speedLimit = ROAD_LIMIT,
        maxSpeedDiff = ADDrivePathModule.MAX_SPEED_DEVIATION,
        brakeHysteresisActive = false,
        distanceToTarget = 100,
        -- the geometry helpers are covered by their own tests, stub them out here
        getLookAheadTarget = function() return 10, 0 end,
        getDistanceToLastWaypoint = function() return 100 end,
        getSpeedLimitBySteeringAngle = function() return 999 end,
        isOnRoadNetwork = function() return true end,
        minDistanceTimer = { timer = function() end },
        waitTimer = { timer = function() end },
        blinkTimer = { timer = function() end },
    })

    AutoDrive.getDriveDirection = function() return 0, 1 end
    AutoDrive.checkIsOnField = function() return false end
    AutoDrive.isInRangeToLoadUnloadTarget = function() return false end
    AutoDrive.isVehicleInBunkerSiloArea = function() return false end
    AutoDrive.getMaxTriggerDistance = function() return 20 end
    AutoDrive.getMinLookaheadByVehicleType = function() return 3 end
    ADTriggerManager = { getMaxBunkerSiloLength = function() return 50 end }
    AutoDrive.driveInDirection = function(veh, dt, maxAngle, acceleration, _, _, _, _, lx, lz, speedLimit)
        table.insert(drive.calls, { acceleration = acceleration, speedLimit = speedLimit, lx = lx, lz = lz })
    end

    return module, drive
end

--- The frame offset the throttle gate uses, so the tests below never hardcode a phase.
local function frameInsideGate(vehicleId, period)
    for frame = 0, period - 1 do
        if ((frame + vehicleId) % period) == 0 then return frame end
    end
end

local function frameOutsideGate(vehicleId, period)
    for frame = 0, period - 1 do
        if ((frame + vehicleId) % period) ~= 0 then return frame end
    end
end

function TestDrivePathSpeedLimit:setUp()
    TestSetup.reset()
end

function TestDrivePathSpeedLimit:testRefinedLimitSurvivesTheFramesWithoutRecomputation()
    local module, drive = makeDrivePathModule(1)
    -- what the gated block would have computed for a corner on the previous frame
    module.speedLimit = 10

    g_updateLoopIndex = frameOutsideGate(module.vehicle.id, AutoDrive.PERF_FRAMES_HIGH)
    module:followWaypoints(16)

    lu.assertEquals(#drive.calls, 1)
    lu.assertEquals(drive.calls[1].speedLimit, 10,
        'the corner limit was replaced by the raw road limit on a frame that does not recompute it')
end

function TestDrivePathSpeedLimit:testLimitIsReBasedOnTheGateFrame()
    local module, drive = makeDrivePathModule(1)
    module.speedLimit = 10   -- stale corner limit, the corner is behind us now

    g_updateLoopIndex = frameInsideGate(module.vehicle.id, AutoDrive.PERF_FRAMES_HIGH)
    module:followWaypoints(16)

    lu.assertEquals(drive.calls[1].speedLimit, ROAD_LIMIT,
        'the gate frame has to re-base the limit on the state module value')
end

function TestDrivePathSpeedLimit:testFieldLimitOfTheGateFrameIsKeptForTheFollowingFrames()
    local module, drive = makeDrivePathModule(1)
    AutoDrive.checkIsOnField = function() return true end   -- field limit is 20

    g_updateLoopIndex = frameInsideGate(module.vehicle.id, AutoDrive.PERF_FRAMES_HIGH)
    module:followWaypoints(16)
    lu.assertEquals(drive.calls[1].speedLimit, 20)

    -- the next three frames must keep it
    for i = 1, AutoDrive.PERF_FRAMES_HIGH - 1 do
        g_updateLoopIndex = g_updateLoopIndex + 1
        module:followWaypoints(16)
        lu.assertEquals(drive.calls[#drive.calls].speedLimit, 20,
            string.format('field limit lost %d frame(s) after it was computed', i))
    end
end

function TestDrivePathSpeedLimit:testBrakingToleranceSurvivesTheSameWay()
    local module, drive = makeDrivePathModule(1)
    -- what the bunker silo branch of the gated block sets
    module.maxSpeedDiff = 1
    module.speedLimit = 10
    module.vehicle.lastSpeedReal = 12 / 3600     -- 12 km/h, so speedDiff is 2

    g_updateLoopIndex = frameOutsideGate(module.vehicle.id, AutoDrive.PERF_FRAMES_HIGH)
    module:followWaypoints(16)

    lu.assertTrue(drive.calls[1].acceleration < 0,
        'a speed 2 km/h over a bunker silo limit of 10 has to brake - the default tolerance of '
        .. tostring(ADDrivePathModule.MAX_SPEED_DEVIATION) .. ' never lets that happen')
end

function TestDrivePathSpeedLimit:testNormalToleranceStillDoesNotBrakeOnSmallDeviations()
    local module, drive = makeDrivePathModule(1)
    module.speedLimit = 10
    module.vehicle.lastSpeedReal = 12 / 3600     -- speedDiff 2, below MAX_SPEED_DEVIATION

    g_updateLoopIndex = frameOutsideGate(module.vehicle.id, AutoDrive.PERF_FRAMES_HIGH)
    module:followWaypoints(16)

    lu.assertEquals(drive.calls[1].acceleration, 1)
end

function TestDrivePathSpeedLimit:testResetStartsOnTheRoadLimitNotOnZero()
    local module = makeDrivePathModule(1)
    module:reset()

    -- a zero here would be read by up to PERF_FRAMES_HIGH-1 frames before the gate re-bases it,
    -- and the brake hysteresis latches on the first of them
    lu.assertEquals(module.speedLimit, ROAD_LIMIT)
    lu.assertEquals(module.maxSpeedDiff, ADDrivePathModule.MAX_SPEED_DEVIATION)
end

------------------------------------------------------------------------------------------------------------------------
--- A13 - the harvester assignment lives on the unload mode
------------------------------------------------------------------------------------------------------------------------
TestTaskModuleClearsAssignment = {}

local function makeTaskModule()
    local vehicle = TestSetup.vehicle()
    vehicle.ad.onRouteToRefuel = false
    vehicle.ad.onRouteToRepair = false
    vehicle.ad.stateModule = { getName = function() return 'TestVehicle' end }
    vehicle.ad.modes = {}
    vehicle.ad.modes[AutoDrive.MODE_UNLOAD] = {
        combine = { getName = function() return 'Harvester' end },
        followingUnloader = { getName = function() return 'Follower' end },
    }

    ADHarvestManager = { unregisterAsUnloader = function() end }
    ADTriggerManager = { getClosestRefuelDestination = function() return 7 end }
    ADGraphManager = { getMapMarkerById = function(_, id) return { id = id } end }
    RefuelTask = { new = function() return { name = 'refuel' } end }
    RepairTask = { new = function() return { name = 'repair' } end }
    AutoDrive.getClosestRepairTrigger = function() return { marker = 3 } end

    return instanceOf(ADTaskModule, { vehicle = vehicle })
end

function TestTaskModuleClearsAssignment:setUp()
    TestSetup.reset()
end

function TestTaskModuleClearsAssignment:testRefuelClearsTheModeNotTheTaskModule()
    local module = makeTaskModule()
    module.hasToRefuel = function() return true end
    local mode = module.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]

    module:RefuelIfNeeded()

    lu.assertNil(mode.combine, 'the unload mode kept a stale harvester reference')
    lu.assertNil(mode.followingUnloader, 'the unload mode kept a stale follower reference')
    lu.assertNotNil(module.activeTask)
end

function TestTaskModuleClearsAssignment:testRepairClearsTheModeNotTheTaskModule()
    local module = makeTaskModule()
    module.hasToRepair = function() return true end
    local mode = module.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]

    module:RepairIfNeeded()

    lu.assertNil(mode.combine)
    lu.assertNil(mode.followingUnloader)
    lu.assertNotNil(module.activeTask)
end

function TestTaskModuleClearsAssignment:testClearingSurvivesAVehicleWithoutModes()
    local module = makeTaskModule()
    module.vehicle.ad.modes = nil
    module.hasToRefuel = function() return true end

    lu.assertNotNil(module.clearCombineAssignment)
    module:RefuelIfNeeded()   -- must not raise
end

------------------------------------------------------------------------------------------------------------------------
--- A14 - previousMode must not hand a locomotive the harvester mode
------------------------------------------------------------------------------------------------------------------------
TestStateModuleModeCycle = {}

local function makeStateModule(isLocomotive, mode)
    local vehicle = TestSetup.vehicle({ spec_locomotive = isLocomotive and { state = 1 } or nil })
    return instanceOf(ADStateModule, {
        vehicle = vehicle,
        mode = mode,
        setAutomaticPickupTarget = function() end,
        setAutomaticUnloadTarget = function() end,
        raiseDirtyFlag = function() end,
    })
end

function TestStateModuleModeCycle:setUp()
    TestSetup.reset()
end

--- Pins down which constants the guard is allowed to name: the two the broken version used do not
--- exist, and a nil on either side of a comparison is exactly what made the guard dead.
function TestStateModuleModeCycle:testTheConstantsTheGuardNeedsAreTheOnesThatExist()
    lu.assertNotNil(AutoDrive.MODE_DRIVETO)
    lu.assertNotNil(AutoDrive.MODE_UNLOAD)
    lu.assertNotNil(ADStateModule.HIGHEST_MODE)
    lu.assertNil(ADStateModule.MODE_DRIVETO)
    lu.assertNil(AutoDrive.HIGHEST_MODE)
end

function TestStateModuleModeCycle:testLocomotiveWrapDoesNotLandOnUnloadMode()
    local state = makeStateModule(true, AutoDrive.MODE_DRIVETO)

    state:previousMode()

    lu.assertNotEquals(state.mode, AutoDrive.MODE_UNLOAD,
        'wrapping backwards from the first mode put the train into the harvester mode')
    lu.assertTrue(state.mode >= AutoDrive.MODE_DRIVETO and state.mode <= ADStateModule.HIGHEST_MODE)
end

function TestStateModuleModeCycle:testLocomotiveNeverReachesUnloadModeInEitherDirection()
    for _, step in ipairs({ 'previousMode', 'nextMode' }) do
        local state = makeStateModule(true, AutoDrive.MODE_DRIVETO)
        for i = 1, ADStateModule.HIGHEST_MODE * 3 do
            state[step](state)
            lu.assertNotEquals(state.mode, AutoDrive.MODE_UNLOAD,
                string.format('%s reached the harvester mode after %d steps', step, i))
            lu.assertTrue(state.mode >= AutoDrive.MODE_DRIVETO and state.mode <= ADStateModule.HIGHEST_MODE,
                string.format('%s left the valid range: %s', step, tostring(state.mode)))
        end
    end
end

function TestStateModuleModeCycle:testNormalVehicleStillWrapsToTheHighestMode()
    local state = makeStateModule(false, AutoDrive.MODE_DRIVETO)

    state:previousMode()

    lu.assertEquals(state.mode, ADStateModule.HIGHEST_MODE)
end

function TestStateModuleModeCycle:testNormalStepDownIsUnchanged()
    local state = makeStateModule(false, AutoDrive.MODE_DELIVERTO)
    state:previousMode()
    lu.assertEquals(state.mode, AutoDrive.MODE_PICKUPANDDELIVER)

    local train = makeStateModule(true, AutoDrive.MODE_DELIVERTO)
    train:previousMode()
    lu.assertEquals(train.mode, AutoDrive.MODE_PICKUPANDDELIVER)
end

------------------------------------------------------------------------------------------------------------------------
--- A35 - the seed/fertilizer fallback has to ask a different question than the check that failed
------------------------------------------------------------------------------------------------------------------------
TestTrailerFillTypeFallback = {}

local FILL_TYPE_INDEX = { SEEDS = 11, FERTILIZER = 12, LIQUIDFERTILIZER = 13 }

--- fillTypesMatch stub: the selected fill type (allowedFillTypes == nil) never matches, which is
--- the situation the fallback exists for. A named candidate matches only if the trigger offers it.
local function installFillTypesMatch(triggerOffers, log)
    AutoDrive.fillTypesMatch = function(_, _, _, allowedFillTypes, _)
        table.insert(log, allowedFillTypes)
        if allowedFillTypes == nil then
            return false
        end
        for _, index in pairs(allowedFillTypes) do
            if triggerOffers[index] then return true end
        end
        return false
    end
end

local function makeTrailerModule(log)
    local vehicle = TestSetup.vehicle()
    vehicle.ad.stateModule = { getFillType = function() return FillType.WHEAT end }

    g_fillTypeManager.getFillTypeIndexByName = function(_, name) return FILL_TYPE_INDEX[name] end

    return instanceOf(ADTrailerModule, {
        vehicle = vehicle,
        startLoadingAtTrigger = function(_, _, fillType) table.insert(log, { started = fillType }) end,
    })
end

function TestTrailerFillTypeFallback:setUp()
    TestSetup.reset()
end

function TestTrailerFillTypeFallback:testFallbackLoadsTheFillTypeTheTriggerOffers()
    local log = {}
    installFillTypesMatch({ [FILL_TYPE_INDEX.FERTILIZER] = true }, log)
    local module = makeTrailerModule(log)

    module:startLoadingCorrectFillTypeAtTrigger({}, { fillTypes = {} }, 1)

    local started = nil
    for _, entry in ipairs(log) do
        if type(entry) == 'table' and entry.started ~= nil then started = entry.started end
    end
    lu.assertEquals(started, FILL_TYPE_INDEX.FERTILIZER,
        'the fallback repeated the check that had already failed, so nothing was ever loaded')
end

function TestTrailerFillTypeFallback:testFallbackPassesEachCandidateAsAllowedFillType()
    local log = {}
    installFillTypesMatch({}, log)      -- trigger offers nothing, so all candidates are tried
    local module = makeTrailerModule(log)

    module:startLoadingCorrectFillTypeAtTrigger({}, { fillTypes = {} }, 1)

    local seen = {}
    for _, allowed in ipairs(log) do
        if type(allowed) == 'table' and allowed.started == nil and allowed[1] ~= nil then
            seen[allowed[1]] = true
        end
    end
    lu.assertTrue(seen[FILL_TYPE_INDEX.SEEDS], 'SEEDS was never offered as a candidate')
    lu.assertTrue(seen[FILL_TYPE_INDEX.FERTILIZER], 'FERTILIZER was never offered as a candidate')
    lu.assertTrue(seen[FILL_TYPE_INDEX.LIQUIDFERTILIZER], 'LIQUIDFERTILIZER was never offered as a candidate')
end

function TestTrailerFillTypeFallback:testNothingIsLoadedWhenNoCandidateMatches()
    local log = {}
    installFillTypesMatch({}, log)
    local module = makeTrailerModule(log)

    module:startLoadingCorrectFillTypeAtTrigger({}, { fillTypes = {} }, 1)

    for _, entry in ipairs(log) do
        lu.assertNil(type(entry) == 'table' and entry.started or nil,
            'loading was started although no candidate matched the trigger')
    end
end

function TestTrailerFillTypeFallback:testSelectedFillTypeStillTakesPrecedence()
    local log = {}
    AutoDrive.fillTypesMatch = function(_, _, _, allowedFillTypes)
        table.insert(log, allowedFillTypes)
        return allowedFillTypes == nil     -- the selected fill type matches straight away
    end
    local module = makeTrailerModule(log)

    module:startLoadingCorrectFillTypeAtTrigger({}, { fillTypes = {} }, 1)

    local started = nil
    for _, entry in ipairs(log) do
        if type(entry) == 'table' and entry.started ~= nil then started = entry.started end
    end
    lu.assertEquals(started, FillType.WHEAT)
end

------------------------------------------------------------------------------------------------------------------------
--- A36 - cruise control and physics belong to the vehicle, not to the module
------------------------------------------------------------------------------------------------------------------------
TestTrainModuleStopMotor = {}

local function makeTrainModule()
    local calls = { cruise = {}, physics = {}, stopMotor = 0 }

    local vehicle = TestSetup.vehicle({
        spec_locomotive = { state = Locomotive.STATE_MANUAL_TRAVEL_INACTIVE, speed = 0 },
        movingDirection = 1,
        getIsMotorStarted = function() return true end,
        stopMotor = function() calls.stopMotor = calls.stopMotor + 1 end,
        raiseActive = function() end,
        setLocomotiveState = function() end,
        setCruiseControlState = function(_, state) table.insert(calls.cruise, state) end,
        updateVehiclePhysics = function(_, a, b, c, d)
            table.insert(calls.physics, { a, b, c, d })
        end,
    })
    vehicle.ad.stateModule = { getCurrentDestination = function() return nil end }
    vehicle.ad.trailerModule = { getCanStopMotor = function() return true end }
    vehicle.ad.specialDrivingModule = {
        stoppedTimer = { timer = function() end, done = function() return true end },
        shouldStopMotor = function() return true end,
    }

    AutoDrive.getIsEntered = function() return false end
    ADGraphManager = { getWayPointById = function() return nil end }

    return instanceOf(ADTrainModule, { vehicle = vehicle }), calls
end

function TestTrainModuleStopMotor:setUp()
    TestSetup.reset()
end

function TestTrainModuleStopMotor:testResetSwitchesCruiseControlOffOnTheVehicle()
    local module, calls = makeTrainModule()

    module:reset()

    lu.assertEquals(calls.cruise, { Drivable.CRUISECONTROL_STATE_OFF },
        'cruise control was never switched off before the motor was stopped')
    lu.assertEquals(calls.physics, { { 0, 0, 0, 16 } },
        'the physics input was never zeroed before the motor was stopped')
    lu.assertEquals(calls.stopMotor, 1)
end

function TestTrainModuleStopMotor:testStopAndHoldSwitchesCruiseControlOffOnTheVehicle()
    local module, calls = makeTrainModule()

    module:stopAndHoldVehicle(16)

    lu.assertEquals(calls.cruise, { Drivable.CRUISECONTROL_STATE_OFF })
    lu.assertEquals(calls.physics, { { 0, 0, 0, 16 } })
    lu.assertEquals(calls.stopMotor, 1)
end

function TestTrainModuleStopMotor:testMotorIsLeftAloneWhileTheDriverIsInTheCab()
    local module, calls = makeTrainModule()
    AutoDrive.getIsEntered = function() return true end

    module:reset()

    lu.assertEquals(calls.stopMotor, 0)
    lu.assertEquals(#calls.cruise, 0)
end

------------------------------------------------------------------------------------------------------------------------
--- A37 / A38 - collision detection
------------------------------------------------------------------------------------------------------------------------
TestCollisionDetection = {}

local function makeCollisionModule(vehicleId)
    local scans = { count = 0 }

    local vehicle = TestSetup.vehicle({ id = vehicleId })
    vehicle.ad.taskModule = {
        getActiveTask = function()
            return { getExcludedVehiclesForCollisionCheck = function() return {} end }
        end,
    }
    vehicle.ad.sensors = {
        frontSensorDynamicLong = {
            getBoxShape = function()
                return { topLeft = {}, topRight = {}, downRight = {}, downLeft = {}, y = 0 }
            end,
        },
    }

    AutoDrive.testSettings['enableTrafficDetection'] = 0    -- short sensor path off
    AutoDrive.checkForVehicleCollision = function()
        scans.count = scans.count + 1
        return false
    end

    return instanceOf(ADCollisionDetectionModule, { vehicle = vehicle, detectedCollision = false }), scans
end

function TestCollisionDetection:setUp()
    TestSetup.reset()
end

function TestCollisionDetection:testEachVehicleScansOnceEveryPerfFramesWindow()
    for _, id in ipairs({ 1, 2, 7 }) do
        local module, scans = makeCollisionModule(id)
        for frame = 0, AutoDrive.PERF_FRAMES - 1 do
            g_updateLoopIndex = frame
            module:detectObstacle()
        end
        lu.assertEquals(scans.count, 1,
            string.format('vehicle %d scanned %d times per %d frames', id, scans.count, AutoDrive.PERF_FRAMES))
    end
end

function TestCollisionDetection:testVehiclesDoNotAllScanOnTheSameFrame()
    local frames = {}
    for _, id in ipairs({ 1, 2, 3, 4 }) do
        local module, scans = makeCollisionModule(id)
        for frame = 0, AutoDrive.PERF_FRAMES - 1 do
            g_updateLoopIndex = frame
            local before = scans.count
            module:detectObstacle()
            if scans.count > before then frames[id] = frame end
        end
    end
    for id, frame in pairs(frames) do
        for otherId, otherFrame in pairs(frames) do
            if id ~= otherId then
                lu.assertNotEquals(frame, otherFrame,
                    string.format('vehicles %d and %d both scan on frame %d', id, otherId, frame))
            end
        end
    end
end

------------------------------------------------------------------------------------------------------------------------
--- A38 - the reverse sensor has to go on the implement getReverseNode actually picked
------------------------------------------------------------------------------------------------------------------------
local function makeReverseRig(vehicleTrailerField, units, mostBackImplement)
    local polled = { object = nil }

    local vehicle = TestSetup.vehicle()
    vehicle.trailer = vehicleTrailerField
    vehicle.ad.sensors = {
        rearSensor = { pollInfo = function() polled.object = vehicle return false end },
    }

    --- The real ADSensor creates the sensor set on first use; the stub does the same so the code
    --- under test can enable and poll it.
    ADSensor = {
        handleSensors = function(_, object)
            object.ad = object.ad or {}
            if object.ad.sensors == nil then
                object.ad.sensors = {
                    rearSensor = {
                        enabled = false,
                        pollInfo = function() polled.object = object return false end,
                    },
                }
            end
        end,
    }

    AutoDrive.getAllUnits = function() return units, #units end
    AutoDrive.getMostBackImplementOf = function() return mostBackImplement end

    return instanceOf(ADCollisionDetectionModule, { vehicle = vehicle }), polled
end

function TestCollisionDetection:testEmptyPlaceholderIsNotTreatedAsAReverseAttachable()
    local lastFillUnit = { name = 'lastFillUnit' }
    local rearmost = { name = 'rearmost' }
    -- what ADSpecialDrivingModule:reset()/:stopVehicle() leave behind
    local module, polled = makeReverseRig({}, { {}, lastFillUnit }, rearmost)

    module:checkReverseCollision()

    lu.assertEquals(polled.object, rearmost,
        'the rear sensor watched the last fill unit although no reverse attachable was identified')
end

function TestCollisionDetection:testRealReverseAttachableStillSelectsTheLastUnit()
    local lastFillUnit = { name = 'lastFillUnit' }
    local rearmost = { name = 'rearmost' }
    -- what getReverseNode assigns: an implement with wheels
    local attachable = { name = 'attachable', spec_wheels = { wheels = {} } }
    local module, polled = makeReverseRig(attachable, { {}, lastFillUnit }, rearmost)

    module:checkReverseCollision()

    lu.assertEquals(polled.object, lastFillUnit)
end

function TestCollisionDetection:testSingleUnitRigUsesTheRearmostImplement()
    local rearmost = { name = 'rearmost' }
    local attachable = { name = 'attachable', spec_wheels = { wheels = {} } }
    local module, polled = makeReverseRig(attachable, { {} }, rearmost)

    module:checkReverseCollision()

    lu.assertEquals(polled.object, rearmost)
end

function TestCollisionDetection:testSoloVehicleFallsBackToItsOwnRearSensor()
    local module, polled = makeReverseRig({}, {}, nil)

    module:checkReverseCollision()

    lu.assertEquals(polled.object, module.vehicle)
end


------------------------------------------------------------------------------------------------------------------------
--- driveToPoint has to be able to notice it is not moving
---
--- The standstill detection ticked stoppedTimer - the timer releaseVehicle resets - on the line
--- after calling releaseVehicle. A TON handed a false signal zeroes its elapsed time, so the
--- standstill could never accumulate past a single frame: measured at sixteen milliseconds after
--- sixty seconds of standing perfectly still. isBlocked, whose only reader is the stuck recovery
--- gate in CombineUnloaderMode, was therefore false no matter how long the vehicle was wedged.
------------------------------------------------------------------------------------------------------------------------
TestDriveToPointBlocked = {}

local function drivingModule(speed)
    local m = setmetatable({}, { __index = ADSpecialDrivingModule })
    m.vehicle = TestSetup.vehicle()
    m.vehicle.components = { { node = 'dtp' } }
    MockEngine.nodePositions['dtp'] = { x = 0, y = 0, z = 0 }
    m.vehicle.lastSpeedReal = speed
    m.vehicle.ad.stateModule = { getFieldSpeedLimit = function() return 20 end }
    m.vehicle.ad.trailerModule = { handleTrailerReversing = function() end }
    ADSpecialDrivingModule.reset(m)
    AutoDrive.getDriveDirection = function() return 0, 1 end
    AutoDrive.driveInDirection = function() end
    return m
end

function TestDriveToPointBlocked:setUp()
    TestSetup.reset()
end

function TestDriveToPointBlocked:testStandingStillEventuallyCountsAsBlocked()
    local m = drivingModule(0)
    local point = { x = 0, y = 0, z = 50 }

    -- twenty seconds of being told to drive while not moving an inch
    for _ = 1, 1250 do
        m:driveToPoint(16, point, 10, false, 1, 20)
    end

    lu.assertTrue(m.isBlocked,
        'a vehicle commanded to drive that has not moved for twenty seconds is blocked, and the '
        .. 'stuck recovery has nothing else to read')
end

function TestDriveToPointBlocked:testAMovingVehicleIsNeverBlocked()
    local m = drivingModule(0.005)
    local point = { x = 0, y = 0, z = 50 }

    for _ = 1, 1250 do
        m:driveToPoint(16, point, 10, false, 1, 20)
    end

    lu.assertFalse(m.isBlocked)
end

--- And the motor-stop timer keeps its own life: releasing the vehicle still clears it, or a driver
--- that stopped once would carry that standstill into the next time it halts and cut the ten
--- seconds before its engine is switched off.
function TestDriveToPointBlocked:testReleasingStillClearsTheMotorStopTimer()
    local m = drivingModule(0)
    m.stoppedTimer:timer(true, 10000, 9000)
    lu.assertTrue(m.stoppedTimer.elapsedTime > 0, 'test setup')

    m:releaseVehicle()

    lu.assertEquals(m.stoppedTimer.elapsedTime, 0)
end


--- The guided reverse target is a WORLD point a hundred metres behind where the episode began, and
--- the only thing that ever cleared it needs two consecutive update() calls that the chase path
--- never makes - driveToPoint's driving branch does not call update at all, and the
--- retreat-and-resume route goes straight from reversing back to chasing without one. So the next
--- reverse steered at a point recorded minutes and hundreds of metres earlier, and once the rig had
--- driven past it the controller reported itself arrived and commanded nothing.
function TestDriveToPointBlocked:testAFreshReverseEpisodeGetsAFreshTarget()
    local m = drivingModule(0)
    m.vehicle.ad.collisionDetectionModule = { checkReverseCollision = function() return false end }
    -- driveReverse reaches the controller through vehicle.ad.specialDrivingModule, which in the
    -- game is this very module
    m.vehicle.ad.specialDrivingModule = m
    local reversedTo = {}
    m.reverseToTargetLocation = function(_, _, target) reversedTo[#reversedTo + 1] = target end
    AutoDrive.localToWorld = function(_, _, _, z)
        local pos = MockEngine.nodePositions['dtp']
        return pos.x, 0, pos.z + z
    end

    g_updateLoopIndex = 100
    m:driveReverse(16, 8, 1, true)
    local firstTarget = reversedTo[1]
    lu.assertNotNil(firstTarget)

    -- the chase resumes: hundreds of frames elsewhere, and the vehicle moves a long way
    g_updateLoopIndex = 100 + 3000
    MockEngine.nodePositions['dtp'] = { x = 0, y = 0, z = 300 }

    m:driveReverse(16, 8, 1, true)

    lu.assertNotEquals(reversedTo[2].z, firstTarget.z,
        'a reverse that does not continue the previous frame has to anchor where the vehicle is '
        .. 'now, not where it stood the last time it reversed')
end

--- And a reverse that DOES continue keeps its target, or the vehicle chases a point that runs away
--- from it every frame.
function TestDriveToPointBlocked:testAContinuingReverseKeepsItsTarget()
    local m = drivingModule(0)
    m.vehicle.ad.collisionDetectionModule = { checkReverseCollision = function() return false end }
    -- driveReverse reaches the controller through vehicle.ad.specialDrivingModule, which in the
    -- game is this very module
    m.vehicle.ad.specialDrivingModule = m
    local reversedTo = {}
    m.reverseToTargetLocation = function(_, _, target) reversedTo[#reversedTo + 1] = target end
    AutoDrive.localToWorld = function(_, _, _, z)
        local pos = MockEngine.nodePositions['dtp']
        return pos.x, 0, pos.z + z
    end

    g_updateLoopIndex = 100
    m:driveReverse(16, 8, 1, true)
    g_updateLoopIndex = 101
    MockEngine.nodePositions['dtp'] = { x = 0, y = 0, z = -2 }
    m:driveReverse(16, 8, 1, true)

    lu.assertEquals(reversedTo[2], reversedTo[1])
end


--- A reset ends whatever was going on. Leaving the reverse target standing was the other half of
--- how it survived into the next episode.
function TestDriveToPointBlocked:testResetDropsTheReverseTarget()
    local m = drivingModule(0)
    m.reverseTarget = { x = 1, y = 0, z = 2 }
    m.lastGuidedReverseFrame = 42

    ADSpecialDrivingModule.reset(m)

    lu.assertNil(m.reverseTarget)
    lu.assertNil(m.lastGuidedReverseFrame)
end


------------------------------------------------------------------------------------------------------------------------
--- Getting nowhere while standing still
---
--- checkIfStuck measures progress towards the next way point, and its timer only runs while the
--- vehicle is NOT being stopped - the else branch resets it. So the instant anything holds the
--- vehicle, the stuck clock goes to zero and stays there, and a vehicle held by an obstacle can
--- never be found stuck however long it stands.
---
--- Measured in game: four vehicles nose to tail at a field exit, every one reporting an obstacle and
--- braking every frame, four and a half minutes of standstill, and not one stuck message in the log.
--- Nothing else was going to break it either - the off-route yield does not run on the road network,
--- and a make-way request only goes to a PARKED vehicle, which none of them was.
------------------------------------------------------------------------------------------------------------------------
TestHeldTooLong = {}

function TestHeldTooLong:setUp()
    TestSetup.reset()
end

--- The module with only what checkIfStuck touches, plus a record of the stuck calls.
local function heldModule(held, activeTask)
    local stuck = { count = 0 }
    local vehicle = TestSetup.vehicle({ id = 1, isServer = true })
    vehicle.components = { { node = 'held' } }
    MockEngine.nodePositions['held'] = { x = 0, y = 0, z = 0 }
    vehicle.ad.specialDrivingModule = { isStoppingVehicle = function() return held end }
    vehicle.ad.taskModule = { getActiveTask = function() return activeTask end }

    local module = instanceOf(ADDrivePathModule, {
        vehicle = vehicle,
        minDistanceTimer = AutoDriveTON:new(),
        heldTimer = AutoDriveTON:new(),
        minDistanceToNextWp = math.huge,
        wayPoints = TestSetup.lineNetwork(6),
        currentWayPoint = 1,
        handleBeingStuck = function() stuck.count = stuck.count + 1 end,
    })
    return module, stuck
end

local function run(module, milliseconds)
    local step = 100
    for _ = 1, milliseconds / step do
        module:checkIfStuck(step)
    end
end

--- The reported case: held, and never let go.
function TestHeldTooLong:testAVehicleHeldForeverIsEventuallyStuck()
    local module, stuck = heldModule(true, {})

    run(module, ADDrivePathModule.MAX_HELD_TIME + 2000)

    lu.assertTrue(stuck.count > 0,
        'four and a half minutes of standstill has to be noticed by something')
end

--- But an ordinary traffic wait must not raise anything.
function TestHeldTooLong:testAShortWaitIsNotStuck()
    local module, stuck = heldModule(true, {})

    run(module, ADDrivePathModule.MAX_HELD_TIME - 5000)

    lu.assertEquals(stuck.count, 0, 'waiting for traffic is normal and must stay silent')
end

--- And the clock starts again once the vehicle is released, so a series of ordinary waits never
--- adds up to a fault.
function TestHeldTooLong:testBeingReleasedResetsTheClock()
    local module, stuck = heldModule(true, {})
    run(module, ADDrivePathModule.MAX_HELD_TIME - 5000)

    module.vehicle.ad.specialDrivingModule.isStoppingVehicle = function() return false end
    run(module, 1000)
    module.vehicle.ad.specialDrivingModule.isStoppingVehicle = function() return true end
    run(module, ADDrivePathModule.MAX_HELD_TIME - 5000)

    lu.assertEquals(stuck.count, 0, 'two short waits are not one long one')
end

--- A driver waiting to be called is not stuck, and may stand for as long as the harvest takes.
--- Those tasks are exactly the ones that advertise canMakeWay.
function TestHeldTooLong:testAParkedWaiterIsNeverStuck()
    local module, stuck = heldModule(true, { canMakeWay = true })

    run(module, ADDrivePathModule.MAX_HELD_TIME * 3)

    lu.assertEquals(stuck.count, 0, 'waiting for a call is the job, not a fault')
end

os.exit(lu.LuaUnit.run())
