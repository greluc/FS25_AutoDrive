--[[
Tests for scripts/Modes/CombineUnloaderMode.lua.

Covers findings:
  A1  self.activeTask was not kept in sync with the task that is actually running, so the
      AD_continue key could call :continue() on nil or on the wrong task
  A6  CombineUnloaderMode:continue() queued a task on a stopped driver
  A30 getPipeSlopeCorrection was evaluated twice per frame in the chopper branch
  A31 dead helpers that read fields nobody assigns are gone
  K4  the lateral clearance to the pipe used the TRACTOR's width instead of the widest unit of the
      train, plus the new internal safety margin and its one-time pipeOffset migration
  K6  the chase side was picked purely from the harvester's sensors, with no consideration of
      whether the unloader can get there at all

The mode itself loads standalone - it only needs AbstractMode at load time - so this is a real
behaviour test, not a source-level one. What the mode reaches for at runtime (the harvester, the
task module, the pipe geometry helpers) is stubbed per test; the transform helpers are the real
ones from UtilFuncs on top of an identity-rotation scene, so the offsets that come out are the
offsets the mod computes.
]]

lu = require('luaunit')
require('test-setup')

require('UtilFuncs')
require('AutoDriveUtilFuncs')
require('Settings')
require('AbstractMode')
require('AutoDriveTON')
-- getPipeChasePosition reserves the side it picks, so the manager holding those
-- reservations has to be present
require('HarvestManager')
require('CombineUnloaderMode')

------------------------------------------------------------------------------------------------------------------------
--- Scene: every vehicle sits at its node position facing +Z, so local == world minus origin.
------------------------------------------------------------------------------------------------------------------------
function localToWorld(node, x, y, z)
    local p = MockEngine.nodePositions[node] or { x = 0, y = 0, z = 0 }
    return p.x + x, p.y + y, p.z + z
end

function worldToLocal(node, x, y, z)
    local p = MockEngine.nodePositions[node] or { x = 0, y = 0, z = 0 }
    return x - p.x, y - p.y, z - p.z
end

function localToLocal(fromNode, toNode, x, y, z)
    return worldToLocal(toNode, localToWorld(fromNode, x, y, z))
end

function localDirectionToWorld(_, x, y, z) return x, y, z end
function worldDirectionToLocal(_, x, y, z) return x, y, z end
function getTerrainNormalAtWorldPos() return 0, 1, 0 end

--- Counts how often the geometry check actually fires, so a test cannot pass by never running it.
local engineOverlapBox = overlapBox
local overlapBoxCalls = 0
function overlapBox(...)
    overlapBoxCalls = overlapBoxCalls + 1
    return engineOverlapBox(...)
end

------------------------------------------------------------------------------------------------------------------------
--- Fixtures
------------------------------------------------------------------------------------------------------------------------
local addedTasks = {}

local function task(name)
    return {
        new = function(_, ...)
            local t = { name = name, args = { ... } }
            return t
        end
    }
end

local function resumableTask(name)
    return {
        new = function(_, ...)
            local t = { name = name, args = { ... }, continued = 0 }
            t.continue = function(self) self.continued = self.continued + 1 end
            return t
        end
    }
end

local function makeVehicle(isActive)
    local v = TestSetup.vehicle()
    v.components = { { node = 1 } }
    v.size = { width = 3, length = 6, lengthOffset = 0 }
    v.ad.stateModule = {
        active = isActive ~= false,
        isActive = function(self) return self.active end,
        getFirstMarker = function() return { id = 1 } end,
        getSecondMarker = function() return { id = 2 } end,
        setSecondMarker = function() end,
        getName = function() return 'TestDriver' end,
    }
    v.ad.taskModule = {
        aborts = 0,
        addTask = function(_, t) table.insert(addedTasks, t) end,
        abortCurrentTask = function(self) self.aborts = self.aborts + 1 end,
        abortAllTasks = function(self) self.aborts = self.aborts + 1 end,
    }
    v.ad.trailerModule = { reset = function() end }
    setTranslation(1, 0, 0, -20)
    return v
end

local function makeCombine(overrides)
    local c = {
        size = { width = 4, length = 8, lengthOffset = 0 },
        components = { { node = 10 } },
        lastSpeedReal = 0,
        ad = {
            ADRootNode = 10,
            isChopper = false,
            isFixedPipeChopper = false,
            isAutoAimingChopper = false,
            isSugarcaneHarvester = false,
            isHarvester = true,
            noMovementTimer = { elapsedTime = 0 },
            sensors = {},
        },
        getRootVehicle = function(self) return self end,
    }
    for _, name in ipairs({ 'leftSensor', 'leftSensorFruit', 'leftSensorField',
                            'leftFrontSensor', 'leftFrontSensorFruit',
                            'rightSensor', 'rightSensorFruit', 'rightSensorField',
                            'rightFrontSensor', 'rightFrontSensorFruit', 'frontSensorFruit' }) do
        -- field sensors report "we are on the field", everything else reports "nothing there"
        local clear = name:find('Field') ~= nil
        c.ad.sensors[name] = { pollInfo = function() return clear end }
    end
    setTranslation(10, 0, 0, 0)
    setTranslation(11, 3, 2, 0) -- discharge node, left of the harvester
    if overrides then
        for k, val in pairs(overrides) do c.ad[k] = val end
    end
    return c
end

local function makeMode(vehicle)
    local mode = CombineUnloaderMode:create()
    mode.vehicle = vehicle
    mode.trailers = {}
    mode.trailerCount = 1
    mode.tractorTrainLength = 12
    mode.tractorTrainWidth = 0
    CombineUnloaderMode.setStateNames(mode)
    return mode
end

--- The world outside the mode. Individual tests overwrite what they care about.
local function installStubs()
    addedTasks = {}
    overlapBoxCalls = 0

    ADGraphManager = { getWayPointById = function(_, _) return { x = 0, y = 0, z = 0 } end }
    -- The real manager, not a stub: getPipeChasePosition reserves the side it picks through it,
    -- and a stub would let every test silently pass that reservation. load() gives each test a
    -- fresh claim table, otherwise one test's reservation refuses the next test's side.
    ADHarvestManager:load()
    g_time = 10000
    ADHarvestManager.unregisterAsUnloader = function() end
    ADHarvestManager.getAssignedUnloader = function() return nil end
    ADMultipleTargetsManager = { getNextTarget = function() return nil end }

    UnloadAtDestinationTask = resumableTask('UnloadAtDestinationTask')
    ExitFieldTask = resumableTask('ExitFieldTask')
    WaitForCallTask = task('WaitForCallTask')
    EmptyHarvesterTask = task('EmptyHarvesterTask')
    CatchCombinePipeTask = task('CatchCombinePipeTask')
    DriveToVehicleTask = task('DriveToVehicleTask')
    DriveToDestinationTask = task('DriveToDestinationTask')
    ReverseFromBadLocationTask = task('ReverseFromBadLocationTask')
    ClearCropTask = task('ClearCropTask')
    FollowCombineTask = task('FollowCombineTask')
    HandleHarvesterTurnTask = task('HandleHarvesterTurnTask')

    AutoDrive.checkIsOnField = function() return false end
    AutoDrive.getAllFillLevels = function() return 0, 100, false, 100 end
    AutoDrive.getObjectFillLevels = function() return 0, 100, false, 100 end
    AutoDrive.getAllDischargeableUnits = function() return {}, 0 end
    AutoDrive.validateCachedPipeData = function() end
    AutoDrive.isPipeOut = function() return false end
    AutoDrive.getPipeLength = function() return 3 end
    AutoDrive.getPipeSide = function() return AutoDrive.CHASEPOS_LEFT end
    AutoDrive.getDischargeNode = function() return 11 end
    AutoDrive.getPipeRoot = function() return 10 end
    AutoDrive.getFrontToolWidth = function(combine) return combine.size.width end
    AutoDrive.getFrontToolLength = function() return 0 end
    AutoDrive.getNextFreeDischargeableUnit = function() return 1, 20 end
    AutoDrive.combineIsTurning = function() return false end
    AutoDrive.getIsCPActive = function() return false end
    AutoDrive.getAllImplements = function(vehicle) return { vehicle } end
    AutoDrive.getAllUnits = function(vehicle) return { vehicle }, 1 end
    AutoDrive.collisionMaskVehicleDimesions = CollisionFlag.VEHICLE
    AutoDrive.dynamicChaseDistance = false
end

local function pipeOffsetIndexFor(value)
    for index, v in ipairs(AutoDrive.settings.pipeOffset.values) do
        if math.abs(v - value) < 1e-9 then
            return index
        end
    end
    error('no pipeOffset grid entry for ' .. tostring(value))
end

------------------------------------------------------------------------------------------------------------------------
--- A1 / A6 - the mode must always know which task is running
------------------------------------------------------------------------------------------------------------------------
TestActiveTaskBookkeeping = {}

function TestActiveTaskBookkeeping:setUp()
    TestSetup.reset()
    installStubs()
    self.vehicle = makeVehicle(true)
    self.mode = makeMode(self.vehicle)
end

--- A6: the continue key is bindable and reaches a driver that is not running.
function TestActiveTaskBookkeeping:testContinueOnStoppedDriverDoesNothing()
    self.vehicle.ad.stateModule.active = false
    self.mode.state = self.mode.STATE_DRIVE_TO_UNLOAD

    self.mode:continue()

    lu.assertEquals(#addedTasks, 0, 'a stopped driver must not get a task queued for its next start')
    lu.assertEquals(self.vehicle.ad.taskModule.aborts, 0)
    lu.assertNil(self.mode.activeTask)
end

--- A1: setToWaitForCall leaves STATE_DRIVE_TO_UNLOAD behind with no task recorded at all. That used
--- to be `nil:continue()`.
function TestActiveTaskBookkeeping:testContinueWithoutActiveTaskDoesNotCrash()
    self.mode.state = self.mode.STATE_DRIVE_TO_UNLOAD
    self.mode.activeTask = nil

    self.mode:continue()

    lu.assertNotNil(self.mode.activeTask, 'continue must leave a task behind it can account for')
    lu.assertEquals(#addedTasks, 1)
    lu.assertEquals(addedTasks[1], self.mode.activeTask)
end

--- A1: HandleHarvesterTurnTask finishes with DONT_PROPAGATE and then calls setToWaitForCall, so
--- activeTask could point at a task class that has no continue method at all.
function TestActiveTaskBookkeeping:testContinueWithNonResumableTaskDoesNotCrash()
    self.mode.state = self.mode.STATE_DRIVE_TO_UNLOAD
    self.mode.activeTask = { name = 'HandleHarvesterTurnTask' } -- AbstractTask has no continue

    self.mode:continue()

    lu.assertEquals(#addedTasks, 1, 'the unload task has to be rebuilt instead')
    lu.assertEquals(addedTasks[1].name, 'UnloadAtDestinationTask')
    lu.assertEquals(self.mode.activeTask, addedTasks[1])
end

--- The case that always worked must keep working: a real unload task is resumed, not replaced.
function TestActiveTaskBookkeeping:testContinueResumesTheRunningUnloadTask()
    local running = UnloadAtDestinationTask:new(self.vehicle, 2)
    self.mode.state = self.mode.STATE_DRIVE_TO_UNLOAD
    self.mode.activeTask = running

    self.mode:continue()

    lu.assertEquals(running.continued, 1)
    lu.assertEquals(#addedTasks, 0, 'resuming must not queue a second task')
    lu.assertEquals(self.vehicle.ad.taskModule.aborts, 0)
    lu.assertEquals(self.mode.activeTask, running)
end

function TestActiveTaskBookkeeping:testSetToWaitForCallRecordsTheWaitTask()
    self.mode:setToWaitForCall()

    lu.assertEquals(self.mode.state, self.mode.STATE_WAIT_TO_BE_CALLED)
    lu.assertEquals(#addedTasks, 1)
    lu.assertEquals(addedTasks[1].name, 'WaitForCallTask')
    lu.assertEquals(self.mode.activeTask, addedTasks[1])
end

function TestActiveTaskBookkeeping:testSetToWaitForCallRecordsTheUnloadTask()
    AutoDrive.getAllFillLevels = function() return 100, 100, true, 0 end

    self.mode:setToWaitForCall()

    lu.assertEquals(self.mode.state, self.mode.STATE_DRIVE_TO_UNLOAD)
    lu.assertEquals(addedTasks[1].name, 'UnloadAtDestinationTask')
    lu.assertEquals(self.mode.activeTask, addedTasks[1])
end

function TestActiveTaskBookkeeping:testSetToWaitForCallRecordsTheExitFieldTask()
    AutoDrive.getAllFillLevels = function() return 100, 100, true, 0 end
    AutoDrive.checkIsOnField = function() return true end
    ADGraphManager.getWayPointById = function() return { x = 500, y = 0, z = 500 } end

    self.mode:setToWaitForCall()

    lu.assertEquals(self.mode.state, self.mode.STATE_EXIT_FIELD)
    lu.assertEquals(addedTasks[1].name, 'ExitFieldTask')
    lu.assertEquals(self.mode.activeTask, addedTasks[1])
end

--- getNextTask returns nil when it delegated to setToWaitForCall. Assigning that nil on top of the
--- task setToWaitForCall just enqueued is exactly how activeTask went missing.
function TestActiveTaskBookkeeping:testHandleFinishedTaskKeepsTheTaskSetToWaitForCallQueued()
    self.mode.state = self.mode.STATE_DRIVE_TO_START

    self.mode:handleFinishedTask()

    lu.assertEquals(self.mode.state, self.mode.STATE_WAIT_TO_BE_CALLED)
    lu.assertEquals(#addedTasks, 1, 'the wait task must be queued exactly once')
    lu.assertEquals(self.mode.activeTask, addedTasks[1])
end

function TestActiveTaskBookkeeping:testAssignToHarvesterRecordsTheTask()
    self.mode.state = self.mode.STATE_WAIT_TO_BE_CALLED

    self.mode:assignToHarvester(makeCombine())

    lu.assertEquals(#addedTasks, 1)
    lu.assertEquals(addedTasks[1].name, 'CatchCombinePipeTask')
    lu.assertEquals(self.mode.activeTask, addedTasks[1])
end

function TestActiveTaskBookkeeping:testAssignToHarvesterWithExtendedPipeRecordsTheTask()
    AutoDrive.isPipeOut = function() return true end
    AutoDrive.getObjectFillLevels = function() return 100, 100, true, 0 end
    self.mode.state = self.mode.STATE_WAIT_TO_BE_CALLED

    self.mode:assignToHarvester(makeCombine())

    lu.assertEquals(addedTasks[1].name, 'EmptyHarvesterTask')
    lu.assertEquals(self.mode.activeTask, addedTasks[1])
end

function TestActiveTaskBookkeeping:testDriveToUnloaderRecordsTheTask()
    local other = makeVehicle(true)
    other.ad.modes = { [AutoDrive.MODE_UNLOAD] = { registerFollowingUnloader = function() end } }
    self.mode.state = self.mode.STATE_WAIT_TO_BE_CALLED

    self.mode:driveToUnloader(other)

    lu.assertEquals(addedTasks[1].name, 'DriveToVehicleTask')
    lu.assertEquals(self.mode.activeTask, addedTasks[1])
end

------------------------------------------------------------------------------------------------------------------------
--- A31 - dead helpers reading fields nobody ever assigns
------------------------------------------------------------------------------------------------------------------------
TestDeadCode = {}

function TestDeadCode:setUp()
    TestSetup.reset()
end

--- getPipeSlopeCorrection1 read self.combineNode, getDynamicSideChaseOffsetZ_FS19 read
--- self.targetTrailer / self.targetTrailerFillRatio. None of the three is ever written on the mode,
--- so both functions could only ever have thrown.
function TestDeadCode:testHelpersReadingUnassignedFieldsAreGone()
    lu.assertNil(CombineUnloaderMode.getPipeSlopeCorrection1)
    lu.assertNil(CombineUnloaderMode.getDynamicSideChaseOffsetZ_FS19)
end

--- The survivor is the one that is actually wired up.
function TestDeadCode:testTheUsedSlopeCorrectionSurvives()
    lu.assertIsFunction(CombineUnloaderMode.getPipeSlopeCorrection)
    lu.assertIsFunction(CombineUnloaderMode.getPipeSlopeCorrection2)
end

------------------------------------------------------------------------------------------------------------------------
--- K4 - the clearance to the pipe has to fit the whole train, not the tractor
------------------------------------------------------------------------------------------------------------------------
TestTrainWidth = {}

function TestTrainWidth:setUp()
    TestSetup.reset()
    installStubs()
    self.vehicle = makeVehicle(true)
    -- own copy of the vehicle specific settings, so setSettingState cannot leak into other tests
    AutoDrive.copySettingsToVehicle(self.vehicle)
    self.mode = makeMode(self.vehicle)
    self.mode.combine = makeCombine()
end

function TestTrainWidth:testWidestUnitWins()
    local trailer = { size = { width = 4.2, length = 8 } }
    AutoDrive.getAllUnits = function(v) return { v, trailer }, 2 end

    lu.assertAlmostEquals(CombineUnloaderMode.getTrainWidth(self.vehicle), 4.2, 1e-9)
end

--- CollisionDetectionUtils measures the two half widths separately because implements are not
--- symmetric around the centre line.
function TestTrainWidth:testMeasuredHalfWidthsBeatTheVehicleDefinition()
    local trailer = {
        size = { width = 2.5, length = 8 },
        ad = { adDimensions = { maxWidthLeft = 2.1, maxWidthRight = 2.3 } },
    }
    AutoDrive.getAllUnits = function(v) return { v, trailer }, 2 end

    lu.assertAlmostEquals(CombineUnloaderMode.getTrainWidth(self.vehicle), 4.4, 1e-9)
end

function TestTrainWidth:testFallsBackToTheVehicleDefinition()
    AutoDrive.getAllUnits = function() return nil, 0 end

    lu.assertAlmostEquals(CombineUnloaderMode.getTrainWidth(self.vehicle), self.vehicle.size.width, 1e-9)
end

--- The defect itself: a 5 m chaser bin behind a 3 m tractor used to be given 3 m of clearance.
function TestTrainWidth:testSideChaseOffsetClearsTheTrailerNotTheTractor()
    local trailer = { size = { width = 5, length = 10 } }
    AutoDrive.getAllUnits = function(v) return { v, trailer }, 2 end
    AutoDrive.getPipeLength = function() return 0 end -- keep the pipe out of the max()
    AutoDrive.setSettingState('pipeOffset', pipeOffsetIndexFor(0), self.vehicle)

    local margin = CombineUnloaderMode.calculatePipeSafetyMargin(5)
    -- combine half width + widest unit half width + header extra (0 here) + margin + offset (0)
    lu.assertAlmostEquals(self.mode:getSideChaseOffsetX(), 4 / 2 + 5 / 2 + margin, 1e-9)

    -- and it is strictly further out than the tractor-width answer the mod used to give
    lu.assertTrue(self.mode:getSideChaseOffsetX() > 4 / 2 + self.vehicle.size.width / 2)
end

function TestTrainWidth:testHeaderOverhangIsStillAdded()
    AutoDrive.getPipeLength = function() return 0 end
    AutoDrive.getFrontToolWidth = function() return 9 end -- 2.5 m of header each side of a 4 m body
    AutoDrive.setSettingState('pipeOffset', pipeOffsetIndexFor(0), self.vehicle)

    local margin = CombineUnloaderMode.calculatePipeSafetyMargin(self.vehicle.size.width)
    lu.assertAlmostEquals(self.mode:getSideChaseOffsetX(), 4 / 2 + 3 / 2 + 2.5 + margin, 1e-9)
end

------------------------------------------------------------------------------------------------------------------------
--- K4 - safety margin and the one-time pipeOffset migration
------------------------------------------------------------------------------------------------------------------------
TestPipeSafetyMargin = {}

function TestPipeSafetyMargin:setUp()
    TestSetup.reset()
    installStubs()
    self.vehicle = makeVehicle(true)
    AutoDrive.copySettingsToVehicle(self.vehicle)
    self.mode = makeMode(self.vehicle)
    self.mode.combine = makeCombine()
end

function TestPipeSafetyMargin:testMarginGrowsWithTheTrainButStaysBounded()
    lu.assertAlmostEquals(CombineUnloaderMode.calculatePipeSafetyMargin(2),
        CombineUnloaderMode.PIPE_SAFETY_MARGIN_MIN, 1e-9)
    lu.assertAlmostEquals(CombineUnloaderMode.calculatePipeSafetyMargin(6),
        6 * CombineUnloaderMode.PIPE_SAFETY_MARGIN_FACTOR, 1e-9)
    lu.assertAlmostEquals(CombineUnloaderMode.calculatePipeSafetyMargin(100),
        CombineUnloaderMode.PIPE_SAFETY_MARGIN_MAX, 1e-9)
    lu.assertAlmostEquals(CombineUnloaderMode.calculatePipeSafetyMargin(nil),
        CombineUnloaderMode.PIPE_SAFETY_MARGIN_MIN, 1e-9)
end

--- The player setting is a preference now; the margin is the mod's own and is added on top.
function TestPipeSafetyMargin:testPipeOffsetIsPreferencePlusMargin()
    AutoDrive.setSettingState('pipeOffset', pipeOffsetIndexFor(0.5), self.vehicle)

    lu.assertAlmostEquals(self.mode:getPipeOffset(),
        0.5 + CombineUnloaderMode.calculatePipeSafetyMargin(self.vehicle.size.width), 1e-9)
end

function TestPipeSafetyMargin:testMigrationTakesTheMarginOutOfTheStoredPreference()
    local trailer = { size = { width = 4, length = 10 } }
    AutoDrive.getAllUnits = function(v) return { v, trailer }, 2 end
    local margin = CombineUnloaderMode.calculatePipeSafetyMargin(4) -- 0.6
    AutoDrive.setSettingState('pipeOffset', pipeOffsetIndexFor(1.0), self.vehicle)

    lu.assertTrue(CombineUnloaderMode.migratePipeOffsetForSafetyMargin(self.vehicle))

    lu.assertAlmostEquals(AutoDrive.getSetting('pipeOffset', self.vehicle), 1.0 - margin, 1e-9)
    -- what the mod actually drives with is unchanged, which is the point of migrating at all
    lu.assertAlmostEquals(self.mode:getPipeOffset(), 1.0, 1e-9)
end

function TestPipeSafetyMargin:testMigrationRunsOnlyOnce()
    AutoDrive.setSettingState('pipeOffset', pipeOffsetIndexFor(1.0), self.vehicle)

    lu.assertTrue(CombineUnloaderMode.migratePipeOffsetForSafetyMargin(self.vehicle))
    local afterFirst = AutoDrive.getSetting('pipeOffset', self.vehicle)

    lu.assertFalse(CombineUnloaderMode.migratePipeOffsetForSafetyMargin(self.vehicle))
    lu.assertEquals(AutoDrive.getSetting('pipeOffset', self.vehicle), afterFirst)
end

--- A negative preference used to eat straight into the clearance. After the migration it cannot.
function TestPipeSafetyMargin:testMigrationFloorsAtZero()
    AutoDrive.setSettingState('pipeOffset', pipeOffsetIndexFor(-1.0), self.vehicle)

    CombineUnloaderMode.migratePipeOffsetForSafetyMargin(self.vehicle)

    lu.assertAlmostEquals(AutoDrive.getSetting('pipeOffset', self.vehicle), 0, 1e-9)
end

--- The setting is an index into a coarse grid at the top end; landing between two entries must
--- round down, never up, or the migration would hand out clearance nobody asked for.
function TestPipeSafetyMargin:testMigrationRoundsDownOnTheCoarseGrid()
    local trailer = { size = { width = 4, length = 10 } }
    AutoDrive.getAllUnits = function(v) return { v, trailer }, 2 end
    AutoDrive.setSettingState('pipeOffset', pipeOffsetIndexFor(3.0), self.vehicle)

    CombineUnloaderMode.migratePipeOffsetForSafetyMargin(self.vehicle)

    -- 3.0 - 0.6 = 2.4, and the grid only offers 2.25 and 2.5 there
    lu.assertAlmostEquals(AutoDrive.getSetting('pipeOffset', self.vehicle), 2.25, 1e-9)
end

------------------------------------------------------------------------------------------------------------------------
--- A30 / K6 - choosing the chase position
------------------------------------------------------------------------------------------------------------------------
TestPipeChasePosition = {}

function TestPipeChasePosition:setUp()
    TestSetup.reset()
    installStubs()
    self.vehicle = makeVehicle(true)
    AutoDrive.copySettingsToVehicle(self.vehicle)
    self.mode = makeMode(self.vehicle)
    self.mode.combine = makeCombine()
    self.mode.combineRootVehicle = self.mode.combine
    -- unloader trailing the harvester on its left, i.e. already on the pipe side
    self:placeUnloader(4, -18)

    self.slopeCorrectionCalls = 0
    local mode = self
    self.mode.getPipeSlopeCorrection = function()
        mode.slopeCorrectionCalls = mode.slopeCorrectionCalls + 1
        return 0
    end
end

--- Node 20 is the fill node the dynamic Z offset is measured to; keeping it on the unloader means
--- the chase position stays alongside the harvester instead of drifting with the unloader.
function TestPipeChasePosition:placeUnloader(x, z)
    setTranslation(1, x, 0, z)
    setTranslation(20, x, 0, z)
end

function TestPipeChasePosition:blockWith(object)
    g_currentMission.nodeToObject[99] = object
    MockEngine.overlapBoxHits = { 99 }
end

--- A30: one terrain query per frame, not one per side.
function TestPipeChasePosition:testSlopeCorrectionIsEvaluatedOncePerFrameForChoppers()
    self.mode.combine.ad.isChopper = true

    self.mode:getPipeChasePosition(false)

    lu.assertEquals(self.slopeCorrectionCalls, 1,
        'the chopper branch used to ask for the slope correction once per side')
end

function TestPipeChasePosition:testSlopeCorrectionIsEvaluatedOnceForHarvesters()
    self.mode:getPipeChasePosition(false)

    lu.assertEquals(self.slopeCorrectionCalls, 1)
end

--- K6, the no-regression half: with nothing in the way the answer must be what it always was, and
--- the check must genuinely have run (otherwise this test proves nothing).
function TestPipeChasePosition:testClearFieldStillPicksTheSideChase()
    self.mode.combine.ad.isChopper = true

    local _, sideIndex = self.mode:getPipeChasePosition(false)

    lu.assertEquals(sideIndex, AutoDrive.CHASEPOS_LEFT)
    lu.assertTrue(overlapBoxCalls > 0, 'the reachability check has to actually run')
end

--- K6, the fix: the harvester's sensors report both sides clear - they cannot see the vehicle
--- standing between the unloader and the pipe - so the mode has to notice by itself.
function TestPipeChasePosition:testVehicleInTheWayRulesOutTheSideChase()
    self.mode.combine.ad.isChopper = true
    self:blockWith({ size = { width = 3, length = 6 }, components = { { node = 98 } },
                     getRootVehicle = function(self) return self end })

    local _, sideIndex = self.mode:getPipeChasePosition(false)

    lu.assertEquals(sideIndex, AutoDrive.CHASEPOS_REAR)
end

--- The straight line runs alongside the harvester by construction, so hitting it must not count.
function TestPipeChasePosition:testTheHarvesterItselfDoesNotBlockTheSide()
    self.mode.combine.ad.isChopper = true
    self:blockWith(self.mode.combine)

    local _, sideIndex = self.mode:getPipeChasePosition(false)

    lu.assertEquals(sideIndex, AutoDrive.CHASEPOS_LEFT)
end

function TestPipeChasePosition:testOurOwnTrainDoesNotBlockTheSide()
    self.mode.combine.ad.isChopper = true
    self:blockWith(self.vehicle)

    local _, sideIndex = self.mode:getPipeChasePosition(false)

    lu.assertEquals(sideIndex, AutoDrive.CHASEPOS_LEFT)
end

--- Same for the bunker harvester branch, where only the pipe side is a candidate.
function TestPipeChasePosition:testHarvesterBranchClearFieldPicksThePipeSide()
    local _, sideIndex = self.mode:getPipeChasePosition(false)

    lu.assertEquals(sideIndex, AutoDrive.CHASEPOS_LEFT)
    lu.assertTrue(overlapBoxCalls > 0)
end

function TestPipeChasePosition:testHarvesterBranchFallsBackToTheRearWhenThePipeSideIsUnreachable()
    self:blockWith({ size = { width = 3, length = 6 }, components = { { node = 98 } },
                     getRootVehicle = function(self) return self end })

    local _, sideIndex = self.mode:getPipeChasePosition(false)

    lu.assertEquals(sideIndex, AutoDrive.CHASEPOS_REAR)
end

------------------------------------------------------------------------------------------------------------------------
--- Approach reservations
---
--- Several unloaders on one field used to pick the same side of the same harvester and then drive
--- at the same patch of ground. The side a chase position lands on is now reserved for the unloader
--- that got there first, and the others have to look elsewhere.
------------------------------------------------------------------------------------------------------------------------

--- Another unloader, already holding one side of our harvester.
function TestPipeChasePosition:claimSideForSomeoneElse(side)
    local other = TestSetup.vehicle()
    other.id = 4711
    lu.assertTrue(ADHarvestManager:claimApproach(other, self.mode.combine, side),
        'test setup: the foreign claim has to be granted before it can block anything')
    return other
end

function TestPipeChasePosition:testAChosenSideIsReservedForUs()
    self.mode:getPipeChasePosition(false)

    local other = TestSetup.vehicle()
    other.id = 4711
    lu.assertTrue(ADHarvestManager:isApproachClaimedByOther(other, self.mode.combine, AutoDrive.CHASEPOS_LEFT),
        'picking a side has to reserve it, or the next unloader picks it too')
end

--- The regression: the harvester branch only offers the pipe side, so a foreign claim on it has to
--- send us to the rear rather than to the same spot.
function TestPipeChasePosition:testAForeignClaimOnThePipeSideSendsUsToTheRear()
    self:claimSideForSomeoneElse(AutoDrive.CHASEPOS_LEFT)

    local _, sideIndex = self.mode:getPipeChasePosition(false)

    lu.assertEquals(sideIndex, AutoDrive.CHASEPOS_REAR)
end

--- Same for the chopper branch, which checks the reservation per flank. Which side it ends up on
--- depends on where the unloader already is - here it is on the left, and an unloader does not
--- swing around to the far flank, so the rear is the answer. The invariant worth pinning is only
--- that it does not take the claimed side.
function TestPipeChasePosition:testAClaimedFlankIsNotTaken()
    self.mode.combine.ad.isChopper = true
    self:claimSideForSomeoneElse(AutoDrive.CHASEPOS_LEFT)

    local _, sideIndex = self.mode:getPipeChasePosition(false)

    lu.assertNotEquals(sideIndex, AutoDrive.CHASEPOS_LEFT)
end

--- And without the claim the very same setup does take that side, so the test above cannot pass
--- just because the fixture never wanted the left flank in the first place.
function TestPipeChasePosition:testTheSameFlankIsTakenWhenNobodyHoldsIt()
    self.mode.combine.ad.isChopper = true

    local _, sideIndex = self.mode:getPipeChasePosition(false)

    lu.assertEquals(sideIndex, AutoDrive.CHASEPOS_LEFT)
end

--- Planning is the mode asking "where would I go", not "I am going there" - reserving on those
--- calls would have a vehicle hold sides it never drives to.
function TestPipeChasePosition:testPlanningDoesNotReserve()
    self.mode:getPipeChasePosition(true)

    local other = TestSetup.vehicle()
    other.id = 4711
    lu.assertFalse(ADHarvestManager:isApproachClaimedByOther(other, self.mode.combine, AutoDrive.CHASEPOS_LEFT))
end

--- Our own claim from the previous frame must not push us off our own side.
function TestPipeChasePosition:testOurOwnClaimDoesNotBlockUs()
    local _, first = self.mode:getPipeChasePosition(false)
    local _, second = self.mode:getPipeChasePosition(false)

    lu.assertEquals(second, first, 'a vehicle has to be able to hold the side it already reserved')
end

--- An explicit chaseSide is the player overruling the automatic choice; the extra geometry check
--- has no business running, let alone overruling them back.
function TestPipeChasePosition:testExplicitChaseSideSkipsTheReachabilityCheck()
    self.mode.combine.ad.isChopper = true
    AutoDrive.copySettingsToVehicle(self.mode.combine)
    AutoDrive.setSettingState('chaseSide', 4, self.mode.combine) -- CHASEPOS_RIGHT
    overlapBoxCalls = 0

    local _, sideIndex = self.mode:getPipeChasePosition(false)

    lu.assertEquals(sideIndex, AutoDrive.CHASEPOS_RIGHT)
    lu.assertEquals(overlapBoxCalls, 0)
end

--- A fixed pipe chopper is aimed straight at its discharge node and never uses the two side
--- positions, so there is nothing to check and nothing to pay for.
function TestPipeChasePosition:testFixedPipeChopperSkipsTheReachabilityCheck()
    self.mode.combine.ad.isChopper = true
    self.mode.combine.ad.isFixedPipeChopper = true
    self.mode.targetFillNode = 20
    overlapBoxCalls = 0

    local _, sideIndex = self.mode:getPipeChasePosition(false)

    lu.assertEquals(sideIndex, AutoDrive.CHASEPOS_LEFT) -- self.pipeSide
    lu.assertEquals(overlapBoxCalls, 0)
end

--- Out of range the straight line says nothing, so it must not answer.
function TestPipeChasePosition:testFarAwayChasePositionIsNotJudged()
    self.mode.combine.ad.isChopper = true
    self:placeUnloader(4, -400)
    self:blockWith({ size = { width = 3, length = 6 }, components = { { node = 98 } },
                     getRootVehicle = function(self) return self end })
    overlapBoxCalls = 0

    self.mode:getPipeChasePosition(false)

    lu.assertEquals(overlapBoxCalls, 0, 'nothing to gain from a 400 m straight line')
end

------------------------------------------------------------------------------------------------------------------------
--- A harvester that no longer exists
---
--- Sell or delete a harvester mid-chase and the tasks notice: four of them guard against it and end
--- themselves. They end by propagating, though, so the mode then builds the NEXT task out of the
--- same dead vehicle - and two of those branches reach through to the engine, which no longer has a
--- node for it. Because the active task is cleared only after getNextTask returns, the failure
--- repeated every frame rather than once: one frozen unloader and a log filling up.
------------------------------------------------------------------------------------------------------------------------
TestDeadCombine = {}

function TestDeadCombine:setUp()
    TestSetup.reset()
    installStubs()
    self.vehicle = makeVehicle(true)
    AutoDrive.copySettingsToVehicle(self.vehicle)
    -- makeMode rather than :new, which needs engine globals this suite does not provide
    self.mode = makeMode(self.vehicle)
    self.mode.combine = makeVehicle(false)
    self.mode.combineRootVehicle = self.mode.combine
    g_currentMission.nodeToObject[self.mode.combine.components[1].node] = self.mode.combine
    self.unregistered = false
    ADHarvestManager.unregisterAsUnloader = function() self.unregistered = true end
end

local function deleteHarvester(mode)
    local node = mode.combine.components[1].node
    MockEngine.deletedNodes[node] = true
    g_currentMission.nodeToObject[node] = nil
end

function TestDeadCombine:testALiveHarvesterIsAlive()
    lu.assertTrue(self.mode:isCombineAlive(), 'test setup: it has to start out alive')
end

function TestDeadCombine:testADeletedHarvesterIsNot()
    deleteHarvester(self.mode)

    lu.assertFalse(self.mode:isCombineAlive(),
        'the vehicle table survives deletion because we hold a reference; its scene node does not')
end

function TestDeadCombine:testAHarvesterWithoutComponentsIsNot()
    self.mode.combine.components = nil
    lu.assertFalse(self.mode:isCombineAlive())
end

--- The regression: every branch of getNextTask funnels through here, so one check covers them all.
function TestDeadCombine:testGetNextTaskStandsDownInsteadOfBuildingFromTheCorpse()
    deleteHarvester(self.mode)
    self.mode.state = self.mode.STATE_ACTIVE_UNLOAD_COMBINE

    local nextTask = self.mode:getNextTask()

    lu.assertNil(nextTask, 'standing down enqueues its own task, so nil is what the callers expect')
    lu.assertNil(self.mode.combine, 'holding the dead vehicle would keep it alive and try again')
    lu.assertNil(self.mode.combineRootVehicle)
    lu.assertTrue(self.unregistered, 'the manager must not keep counting us as one of its unloaders')
end

function TestDeadCombine:testItGoesBackToWaitingForACall()
    deleteHarvester(self.mode)
    self.mode.state = self.mode.STATE_ACTIVE_UNLOAD_COMBINE

    self.mode:getNextTask()

    lu.assertEquals(self.mode.state, self.mode.STATE_WAIT_TO_BE_CALLED)
    lu.assertNotNil(self.mode.activeTask, 'standing down has to leave a task running, not nothing')
end

--- And a live harvester still goes through the normal branches.
function TestDeadCombine:testALiveHarvesterIsNotStoodDownFrom()
    self.mode.state = self.mode.STATE_ACTIVE_UNLOAD_COMBINE

    self.mode:getNextTask()

    lu.assertNotNil(self.mode.combine, 'a harvester that still exists must not be dropped')
    lu.assertFalse(self.unregistered)
end

--- With no harvester at all there is nothing to validate and nothing to stand down from.
function TestDeadCombine:testNoHarvesterIsNotAnError()
    self.mode.combine = nil
    self.mode.state = self.mode.STATE_INIT

    self.mode:getNextTask()

    lu.assertFalse(self.unregistered)
end


------------------------------------------------------------------------------------------------------------------------
--- Holding the reachability answer for a throttle window
---
--- getPipeChasePosition is called every frame from FollowCombineTask:updateStates, and the physics
--- query in isChaseSideReachable was the one thing in it without a rate limit - the eight sensor
--- polls above it already have one through ADSensor.EXECUTION_DELAY.
------------------------------------------------------------------------------------------------------------------------
TestReachabilityCache = {}

function TestReachabilityCache:setUp()
    TestSetup.reset()
    installStubs()
    self.vehicle = makeVehicle(true)
    AutoDrive.copySettingsToVehicle(self.vehicle)
    self.mode = makeMode(self.vehicle)
    self.mode.combine = makeVehicle(false)
    self.mode.combineRootVehicle = self.mode.combine
    g_updateLoopIndex = 0
    overlapBoxCalls = 0
end

local function chasePosAt(x, z)
    return { x = x, y = 0, z = z }
end

function TestReachabilityCache:testTheQueryRunsOncePerWindow()
    self.mode:isChaseSideReachable(chasePosAt(6, 6), AutoDrive.CHASEPOS_LEFT)
    local afterFirst = overlapBoxCalls
    lu.assertTrue(afterFirst > 0, 'test setup: the first call has to actually query')

    for _ = 1, 5 do
        self.mode:isChaseSideReachable(chasePosAt(6, 6), AutoDrive.CHASEPOS_LEFT)
    end

    lu.assertEquals(overlapBoxCalls, afterFirst,
        'repeat calls inside one window must answer from the held result')
end

function TestReachabilityCache:testTheQueryRunsAgainInTheNextWindow()
    self.mode:isChaseSideReachable(chasePosAt(6, 6), AutoDrive.CHASEPOS_LEFT)
    local afterFirst = overlapBoxCalls

    g_updateLoopIndex = g_updateLoopIndex + AutoDrive.PERF_FRAMES
    self.mode:isChaseSideReachable(chasePosAt(6, 6), AutoDrive.CHASEPOS_LEFT)

    lu.assertTrue(overlapBoxCalls > afterFirst, 'the answer must not be held forever')
end

--- The reason the key is the side and not the position: the left and right chase spots are mirror
--- images of each other by construction, so anything derived from their coordinates - a sum, a
--- length - is identical for both, and one side would answer for the other.
function TestReachabilityCache:testTheTwoFlanksDoNotShareAnAnswer()
    self.mode:isChaseSideReachable(chasePosAt(-6, 6), AutoDrive.CHASEPOS_LEFT)
    local afterLeft = overlapBoxCalls

    self.mode:isChaseSideReachable(chasePosAt(6, -6), AutoDrive.CHASEPOS_RIGHT)

    lu.assertTrue(overlapBoxCalls > afterLeft,
        'the mirrored right hand position must be asked about in its own right')
end

--- Without a side there is nothing to key on, so it simply always queries.
function TestReachabilityCache:testWithoutASideItAlwaysQueries()
    self.mode:isChaseSideReachable(chasePosAt(6, 6))
    local afterFirst = overlapBoxCalls

    self.mode:isChaseSideReachable(chasePosAt(6, 6))

    lu.assertTrue(overlapBoxCalls > afterFirst)
end

--- The early outs still short circuit: on top of the spot, or far enough that only the pathfinder
--- can answer, there is nothing to query.
function TestReachabilityCache:testTheEarlyOutsStillSkipTheQuery()
    -- the vehicle stands at z = -20, so these are 0.2 m and 500 m from it
    lu.assertTrue(self.mode:isChaseSideReachable(chasePosAt(0.2, -20), AutoDrive.CHASEPOS_LEFT))
    lu.assertTrue(self.mode:isChaseSideReachable(chasePosAt(500, -20), AutoDrive.CHASEPOS_RIGHT))

    lu.assertEquals(overlapBoxCalls, 0)
end



------------------------------------------------------------------------------------------------------------------------
--- monitorTasks: releasing the harvester, and the stuck escalation
---
--- Nothing in the suite called monitorTasks at all - an `error()` as its first statement left the
--- whole gate green. Two rules live in there and both were unobserved.
---
--- The 25 m rule is the ONLY thing that ends an unloader's engagement once its trailer is full: it
--- clears self.combine and unregisters, which is what frees the harvester for a replacement and what
--- promotes a queued second unloader. Widening it from 25 to 2500 left every test passing. The
--- damage it guards against is not "the harvest stops" - the link is released one round trip late,
--- when setToWaitForCall clears the combine anyway - but a second unloader is never promoted, and if
--- the harvester goes idle while this one is still 400 m away with a full trailer, the manager fires
--- it: AutoDrive switched off on a public road, mid-load.
---
--- And the stuck escalation below it: ask parked vehicles to move first, once per episode, and only
--- reverse out if nobody could help.
------------------------------------------------------------------------------------------------------------------------
TestMonitorTasks = {}

local function monitored(state, distance)
    local vehicle = TestSetup.vehicle({ id = 71, components = { { node = 'mtU' } } })
    local combine = TestSetup.vehicle({ id = 72, components = { { node = 'mtC' } } })
    MockEngine.nodePositions['mtU'] = { x = 0, y = 0, z = 0 }
    MockEngine.nodePositions['mtC'] = { x = 0, y = 0, z = distance }
    vehicle.lastSpeedReal = 0

    local mode = setmetatable({}, { __index = CombineUnloaderMode })
    mode.vehicle = vehicle
    mode.combine = combine
    mode.state = state
    mode.failedPathFinder = 0
    mode.askedNearbyVehiclesForWay = false
    mode.followingUnloader = TestSetup.vehicle({ id = 73 })

    local log = { unregistered = 0, asked = 0, aborted = 0, added = 0, released = 0 }
    vehicle.ad.taskModule = {
        getActiveTask = function() return nil end,
        abortAllTasks = function() log.aborted = log.aborted + 1 end,
        addTask = function() log.added = log.added + 1 end,
    }
    vehicle.ad.specialDrivingModule = {
        shouldStopMotor = function() return false end,
        stoppedTimer = { done = function() return false end },
        isBlocked = false,
        releaseVehicle = function() log.released = log.released + 1 end,
    }
    ADHarvestManager.unregisterAsUnloader = function() log.unregistered = log.unregistered + 1 end
    AutoDrive.askNearbyVehiclesToMakeWay = function() return 0 end
    -- the real one lives in CollisionDetectionUtils, which this suite does not load
    AutoDrive.getDistanceBetween = function(a, b)
        local pa = MockEngine.nodePositions[a.components[1].node]
        local pb = MockEngine.nodePositions[b.components[1].node]
        return MathUtil.vector2Length(pb.x - pa.x, pb.z - pa.z)
    end
    return mode, log
end

function TestMonitorTasks:setUp()
    TestSetup.reset()
    self.savedUnregister = ADHarvestManager.unregisterAsUnloader
    self.savedAsk = AutoDrive.askNearbyVehiclesToMakeWay
    self.savedDistance = AutoDrive.getDistanceBetween
end

function TestMonitorTasks:tearDown()
    ADHarvestManager.unregisterAsUnloader = self.savedUnregister
    AutoDrive.askNearbyVehiclesToMakeWay = self.savedAsk
    AutoDrive.getDistanceBetween = self.savedDistance
end

function TestMonitorTasks:testItKeepsTheHarvesterWhileStillBesideIt()
    local mode, log = monitored(CombineUnloaderMode.STATE_DRIVE_TO_UNLOAD, 24)

    mode:monitorTasks(16)

    lu.assertNotNil(mode.combine, 'twenty four metres away is still working with it')
    lu.assertEquals(log.unregistered, 0)
end

function TestMonitorTasks:testDrivingOffReleasesTheHarvester()
    local mode, log = monitored(CombineUnloaderMode.STATE_DRIVE_TO_UNLOAD, 26)

    mode:monitorTasks(16)

    lu.assertNil(mode.combine, 'past the threshold the harvester is free for somebody else')
    lu.assertNil(mode.followingUnloader, 'and the queued second unloader is no longer ours to hold')
    lu.assertEquals(log.unregistered, 1,
        'unregistering is what takes us out of activeUnloaders and promotes the one waiting')
end

--- The other half of the rule: it must not fire while we are still working the harvester.
function TestMonitorTasks:testItDoesNotReleaseWhileUnloadingFromTheCombine()
    local mode, log = monitored(CombineUnloaderMode.STATE_ACTIVE_UNLOAD_COMBINE, 400)
    mode.leaveBreadCrumbs = function() end

    mode:monitorTasks(16)

    lu.assertNotNil(mode.combine)
    lu.assertEquals(log.unregistered, 0)
end

--- Stuck, and one of our own parked vehicles is in the way: ask it to move rather than reversing.
function TestMonitorTasks:testItAsksParkedVehiclesBeforeReversingOut()
    local mode, log = monitored(CombineUnloaderMode.STATE_DRIVE_TO_UNLOAD, 10)
    mode.failedPathFinder = 5
    AutoDrive.askNearbyVehiclesToMakeWay = function() return 1 end

    mode:monitorTasks(16)

    lu.assertTrue(mode.askedNearbyVehiclesForWay, 'the ask happens once per stuck episode')
    lu.assertEquals(log.aborted, 0, 'and it must not throw the running task away to do it')
    lu.assertEquals(mode.failedPathFinder, 0, 'the stuck detection restarts to give them time')
end

--- And when nobody can help, it does reverse out.
function TestMonitorTasks:testItReversesOutWhenNobodyCanMakeWay()
    local mode, log = monitored(CombineUnloaderMode.STATE_DRIVE_TO_UNLOAD, 10)
    mode.failedPathFinder = 5
    AutoDrive.askNearbyVehiclesToMakeWay = function() return 0 end

    mode:monitorTasks(16)

    lu.assertEquals(log.aborted, 1)
    -- rawequal, not assertEquals: the states are empty tables and luaunit compares tables
    -- structurally, so every one of them is "equal" to every other and the assertion would hold
    -- whatever the state actually is.
    lu.assertTrue(rawequal(mode.state, CombineUnloaderMode.STATE_REVERSE_FROM_BAD_LOCATION))
end

--- A driver mid step-aside is not stuck - the manoeuvre meets resistance by design and has its own
--- timeout. Tearing it out reverses the vehicle out of the spot it was only trying to leave.
function TestMonitorTasks:testAVehicleMakingWayIsNotTreatedAsStuck()
    local mode, log = monitored(CombineUnloaderMode.STATE_DRIVE_TO_UNLOAD, 10)
    mode.failedPathFinder = 5
    mode.vehicle.ad.taskModule.getActiveTask = function()
        return { isMakingWay = function() return true end }
    end

    mode:monitorTasks(16)

    lu.assertEquals(log.aborted, 0)
    lu.assertFalse(rawequal(mode.state, CombineUnloaderMode.STATE_REVERSE_FROM_BAD_LOCATION))
end


------------------------------------------------------------------------------------------------------------------------
--- Only a driver that is waiting to be called can be assigned
---
--- assignToHarvester does nothing unless the mode is in STATE_WAIT_TO_BE_CALLED, and that guard is
--- what the harvest manager's skip list depends on: a driver that has already moved on stays in
--- idleUnloaders, declines here, and the manager has to try the next one. The one scheduler test
--- covering "declining" replaces assignToHarvester with a stub, so the guard itself was never run -
--- deleting it would have left the gate green while booking committed drivers to a second harvester.
------------------------------------------------------------------------------------------------------------------------
TestAssignToHarvester = {}

local function assignable(state)
    local vehicle = TestSetup.vehicle({ id = 81, components = { { node = 'atU' } } })
    MockEngine.nodePositions['atU'] = { x = 0, y = 0, z = 0 }
    local mode = setmetatable({}, { __index = CombineUnloaderMode })
    mode.vehicle = vehicle
    mode.state = state
    mode.combine = nil

    local log = { aborted = 0 }
    vehicle.ad.taskModule = {
        abortCurrentTask = function() log.aborted = log.aborted + 1 end,
        addTask = function() end,
        getActiveTask = function() return nil end,
    }
    return mode, log
end

function TestAssignToHarvester:setUp()
    TestSetup.reset()
    self.harvester = TestSetup.vehicle({ id = 82, components = { { node = 'atC' } } })
    MockEngine.nodePositions['atC'] = { x = 0, y = 0, z = 20 }
    self.harvester.getRootVehicle = function(self) return self end
    self.harvester.ad.isHarvester = true
    self.harvester.ad.noMovementTimer = { elapsedTime = 10000 }
    AutoDrive.validateCachedPipeData = function() end
    AutoDrive.isPipeOut = function() return false end
end

--- A driver already following another unloader across the field is not available, and saying so is
--- what makes the manager walk on to the next candidate.
function TestAssignToHarvester:testADriverThatHasMovedOnDeclines()
    local mode, log = assignable(CombineUnloaderMode.STATE_FOLLOW_CURRENT_UNLOADER)

    mode:assignToHarvester(self.harvester)

    lu.assertNil(mode.combine,
        'booking a committed driver loses it into the active list with two harvesters expecting it')
    lu.assertEquals(log.aborted, 0, 'and its running task must not be torn up either')
end

function TestAssignToHarvester:testADriverAlreadyUnloadingDeclines()
    local mode = assignable(CombineUnloaderMode.STATE_ACTIVE_UNLOAD_COMBINE)

    mode:assignToHarvester(self.harvester)

    lu.assertNil(mode.combine)
end

--- And one that is waiting takes the job, which is the half the manager reads to book it.
function TestAssignToHarvester:testAWaitingDriverTakesTheHarvester()
    local mode, log = assignable(CombineUnloaderMode.STATE_WAIT_TO_BE_CALLED)

    mode:assignToHarvester(self.harvester)

    lu.assertTrue(rawequal(mode.combine, self.harvester),
        'the manager books an unloader by checking exactly this afterwards')
    lu.assertEquals(log.aborted, 1, 'the waiting task has to give way to the new job')
end

os.exit(lu.LuaUnit.run())
