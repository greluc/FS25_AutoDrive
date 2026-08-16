--[[
Tests for scripts/Manager/HarvestManager.lua and scripts/Manager/Scheduler.lua.

Covers findings:
  A46 ADHarvestManager:update removed entries from the tables it was walking with pairs()
  B3  hasHarvesterPotentialUnloaders filtered against self.vehicle, which the manager never had
  B4  adHasHarvesterAvailableUnloader ran the whole query twice, once only to be thrown away
  B5  ADScheduler:load initialised activePathFindervehicle, everything else reads ...Vehicle
  P6  updateAverageFPS summed all 3600 ring buffer slots on every call

Both modules load standalone - they are plain function tables with no load time game access - so
these are real behaviour tests. The two spots where behaviour alone cannot see the fix (the ring
buffer being summed rather than tracked, and the field name itself) are pinned by reading the
source, in the style of Courseplay's AutoDriveInterfaceTest.lua.
]]

lu = require('luaunit')
require('test-setup')

require('UtilFuncs')
require('AutoDriveTON')
require('HarvestManager')
require('Scheduler')

------------------------------------------------------------------------------------------------------------------------
--- Stubs for the game side functions these two managers call
------------------------------------------------------------------------------------------------------------------------
--- Not in mock-engine.lua: entityExists is an engine global and AutoDrive.getAllVehicles /
--- isPipeOut / getIsEntered live in modules that cannot be loaded without the full game.
local TestState = { entities = {}, allVehicles = {} }

function entityExists(node)
    return TestState.entities[node] == true
end

function AutoDrive.getAllVehicles()
    return TestState.allVehicles
end

function AutoDrive.isPipeOut(vehicle)
    return vehicle ~= nil and vehicle.ad ~= nil and vehicle.ad.testPipeOut == true
end

function AutoDrive:getIsEntered(vehicle)
    return vehicle ~= nil and vehicle.testIsEntered == true
end

function AutoDrive.getControlledVehicle()
    return nil
end

local function stateModuleStub(marker, mode, active)
    return {
        getFirstMarker = function() return marker end,
        getMode = function() return mode or AutoDrive.MODE_DRIVETO end,
        isActive = function() return active == true end,
        getName = function() return 'Stub' end,
    }
end

--- A harvester complete enough to survive a full ADHarvestManager:update pass.
local function harvesterStub(id, marker)
    local h = TestSetup.vehicle({
        id = id,
        components = { { node = 'node' .. tostring(id) } },
        movingDirection = 1,
        lastSpeedReal = 0,
        getName = function() return 'Harvester' .. tostring(id) end,
        getIsAIActive = function() return true end,
    })
    h.ad.isChopper = true
    h.ad.noMovementTimer = AutoDriveTON:new()
    h.ad.driveForwardTimer = AutoDriveTON:new()
    h.ad.stateModule = stateModuleStub(marker or 1)
    return h
end

local function pathfinderVehicleStub(id)
    local delays = {}
    local v = TestSetup.vehicle({ id = id, getName = function() return 'PF' .. tostring(id) end })
    v.ad.stateModule = stateModuleStub(1)
    v.ad.pathFinderModule = {
        delays = delays,
        addDelayTimer = function(_, ms) table.insert(delays, ms) end,
    }
    return v
end

local function readSource(relativePath)
    local f = io.open(relativePath, 'r')
    if f == nil then
        error('cannot read ' .. relativePath .. ' - run the tests from tools/tests')
    end
    local src = f:read('*a')
    f:close()
    return src
end

------------------------------------------------------------------------------------------------------------------------
--- A46 - update() must not skip harvesters while it reshuffles the lists
------------------------------------------------------------------------------------------------------------------------
TestHarvestManagerUpdate = {}

function TestHarvestManagerUpdate:setUp()
    TestSetup.reset()
    TestState.entities = {}
    TestState.allVehicles = {}
    ADHarvestManager:load()
end

--- Four idle harvesters that all became AI active in the same frame. Removing them from
--- idleHarvesters inside the pairs() loop shifted the array under the iterator and left every
--- second one behind, so half the fleet stayed idle until some later frame removed nothing.
function TestHarvestManagerUpdate:testAllActivatedHarvestersMoveOutOfIdleInOnePass()
    local expected = {}
    for i = 1, 4 do
        local h = harvesterStub(i)
        TestState.entities[h.components[1].node] = true
        g_currentMission.nodeToObject[h.components[1].node] = h
        table.insert(ADHarvestManager.idleHarvesters, h)
        expected[i] = h
    end

    ADHarvestManager:update(16)

    lu.assertEquals(#ADHarvestManager.idleHarvesters, 0,
        'every activated harvester must leave idleHarvesters in the same pass')
    lu.assertEquals(#ADHarvestManager.harvesters, 4)
    lu.assertEquals(ADHarvestManager.harvesters, expected, 'order must be preserved')
end

--- The mirror case: four active harvesters that all go idle at once must all leave self.harvesters.
function TestHarvestManagerUpdate:testAllDeactivatedHarvestersLeaveTheActiveList()
    local expected = {}
    for i = 1, 4 do
        local h = harvesterStub(i)
        h.getIsAIActive = function() return false end
        TestState.entities[h.components[1].node] = true
        g_currentMission.nodeToObject[h.components[1].node] = h
        table.insert(ADHarvestManager.harvesters, h)
        expected[i] = h
    end

    ADHarvestManager:update(16)

    lu.assertEquals(#ADHarvestManager.harvesters, 0,
        'every deactivated harvester must leave harvesters in the same pass')
    lu.assertEquals(#ADHarvestManager.idleHarvesters, 4)
    lu.assertEquals(ADHarvestManager.idleHarvesters, expected, 'order must be preserved')
end

--- Third loop: harvesters whose node is gone (sold, left the map) are dropped. Same skip.
function TestHarvestManagerUpdate:testAllVanishedHarvestersAreDropped()
    for i = 1, 4 do
        local h = harvesterStub(i)
        -- deliberately not registered in nodeToObject: the vehicle no longer exists
        table.insert(ADHarvestManager.harvesters, h)
    end

    ADHarvestManager:update(16)

    lu.assertEquals(#ADHarvestManager.harvesters, 0,
        'a harvester whose node is gone must be dropped, none may be skipped')
end

--- A harvester that is still fine must not be dropped along the way.
function TestHarvestManagerUpdate:testValidHarvestersSurviveTheUpdate()
    local kept = harvesterStub(1)
    TestState.entities[kept.components[1].node] = true
    g_currentMission.nodeToObject[kept.components[1].node] = kept
    local gone = harvesterStub(2)
    table.insert(ADHarvestManager.harvesters, kept)
    table.insert(ADHarvestManager.harvesters, gone)

    ADHarvestManager:update(16)

    lu.assertEquals(ADHarvestManager.harvesters, { kept })
end

------------------------------------------------------------------------------------------------------------------------
--- B3 - hasHarvesterPotentialUnloaders excludes the harvester, not a field that never existed
------------------------------------------------------------------------------------------------------------------------
TestPotentialUnloaders = {}

function TestPotentialUnloaders:setUp()
    TestSetup.reset()
    TestState.entities = {}
    TestState.allVehicles = {}
    ADHarvestManager:load()
end

--- The premise of B3: the manager is a singleton and never had a .vehicle field, so the old
--- "other ~= self.vehicle" compared against nil and excluded nothing.
function TestPotentialUnloaders:testManagerHasNoVehicleField()
    lu.assertNil(ADHarvestManager.vehicle,
        'ADHarvestManager is a singleton manager - a .vehicle field here would be a new concept')
end

function TestPotentialUnloaders:testHarvesterDoesNotCountAsItsOwnUnloader()
    local harvester = harvesterStub(1)
    harvester.ad.stateModule = stateModuleStub(7, AutoDrive.MODE_UNLOAD, true)
    TestState.allVehicles = { harvester }

    lu.assertFalse(ADHarvestManager:hasHarvesterPotentialUnloaders(harvester),
        'the harvester itself must not be counted as a potential unloader')
end

function TestPotentialUnloaders:testAnotherUnloadVehicleOnTheSameMarkerCounts()
    local harvester = harvesterStub(1)
    harvester.ad.stateModule = stateModuleStub(7, AutoDrive.MODE_UNLOAD, true)
    local unloader = harvesterStub(2)
    unloader.ad.stateModule = stateModuleStub(7, AutoDrive.MODE_UNLOAD, true)
    TestState.allVehicles = { harvester, unloader }

    lu.assertTrue(ADHarvestManager:hasHarvesterPotentialUnloaders(harvester),
        'a genuine unloader must still be found - the fix must not narrow the search')
end

function TestPotentialUnloaders:testUnloadVehicleOnAnotherMarkerDoesNotCount()
    local harvester = harvesterStub(1)
    harvester.ad.stateModule = stateModuleStub(7, AutoDrive.MODE_UNLOAD, true)
    local elsewhere = harvesterStub(2)
    elsewhere.ad.stateModule = stateModuleStub(9, AutoDrive.MODE_UNLOAD, true)
    TestState.allVehicles = { harvester, elsewhere }

    lu.assertFalse(ADHarvestManager:hasHarvesterPotentialUnloaders(harvester))
end

--- The registered lists are checked before the global sweep and are unaffected by the fix.
function TestPotentialUnloaders:testIdleUnloaderOnTheSameMarkerCounts()
    local harvester = harvesterStub(1)
    harvester.ad.stateModule = stateModuleStub(7)
    local idle = harvesterStub(2)
    idle.ad.stateModule = stateModuleStub(7)
    table.insert(ADHarvestManager.idleUnloaders, idle)

    lu.assertTrue(ADHarvestManager:hasHarvesterPotentialUnloaders(harvester))
end

------------------------------------------------------------------------------------------------------------------------
--- B4 - the Courseplay entry point must run the query once, not once per debug argument list
------------------------------------------------------------------------------------------------------------------------
TestAvailableUnloaderEntryPoint = {}

function TestAvailableUnloaderEntryPoint:setUp()
    TestSetup.reset()
    ADHarvestManager:load()
    self.originalQuery = ADHarvestManager.hasHarvesterAvailableUnloader
    self.calls = 0
end

function TestAvailableUnloaderEntryPoint:tearDown()
    ADHarvestManager.hasHarvesterAvailableUnloader = self.originalQuery
end

function TestAvailableUnloaderEntryPoint:spyOn(result)
    local this = self
    ADHarvestManager.hasHarvesterAvailableUnloader = function(_, _)
        this.calls = this.calls + 1
        return result
    end
end

function TestAvailableUnloaderEntryPoint:testQueryRunsOnceWithDebugOff()
    self:spyOn(true)
    AutoDrive.currentDebugChannelMask = 0
    local harvester = harvesterStub(1)

    local result = ADHarvestManager.adHasHarvesterAvailableUnloader(harvester)

    lu.assertTrue(result)
    lu.assertEquals(self.calls, 1,
        'the query must not be evaluated a second time just to build a suppressed debug message')
end

function TestAvailableUnloaderEntryPoint:testQueryRunsOnceWithDebugOn()
    self:spyOn(false)
    AutoDrive.currentDebugChannelMask = AutoDrive.DC_EXTERNALINTERFACEINFO
    local harvester = harvesterStub(1)

    local result = ADHarvestManager.adHasHarvesterAvailableUnloader(harvester)

    lu.assertFalse(result)
    lu.assertEquals(self.calls, 1)
    lu.assertEquals(Logging.countMatching('hasHarvesterAvailableUnloader false'), 1,
        'the logged value must be the value that is returned')
end

--- The real query, so the spy above cannot drift away from it.
function TestAvailableUnloaderEntryPoint:testRealQueryFindsAnIdleUnloaderOnTheSameMarker()
    local harvester = harvesterStub(1)
    harvester.ad.stateModule = stateModuleStub(3)
    local idle = harvesterStub(2)
    idle.ad.stateModule = stateModuleStub(3)
    table.insert(ADHarvestManager.idleUnloaders, idle)

    lu.assertTrue(ADHarvestManager.adHasHarvesterAvailableUnloader(harvester))

    idle.ad.stateModule = stateModuleStub(4)
    lu.assertFalse(ADHarvestManager.adHasHarvesterAvailableUnloader(harvester))
end

------------------------------------------------------------------------------------------------------------------------
--- B5 - one spelling of the active pathfinder vehicle
------------------------------------------------------------------------------------------------------------------------
TestSchedulerActiveVehicle = {}

function TestSchedulerActiveVehicle:setUp()
    TestSetup.reset()
    ADScheduler:load()
end

--- The bug with teeth: load() cleared a field nobody reads, so a vehicle left over from the
--- previous mission stayed in activePathFinderVehicle across a reload.
function TestSchedulerActiveVehicle:testLoadClearsTheFieldThatIsActuallyRead()
    ADScheduler.activePathFinderVehicle = { stale = true }

    ADScheduler:load()

    lu.assertNil(ADScheduler.activePathFinderVehicle,
        'load() must reset the field updateActiveVehicle reads, not a misspelled twin')
end

function TestSchedulerActiveVehicle:testMisspelledFieldIsGoneFromTheSource()
    local src = readSource('../../scripts/Manager/Scheduler.lua')
    lu.assertNil(src:find('activePathFindervehicle', 1, true),
        'the lowercase v spelling must not exist anywhere in Scheduler.lua')
    lu.assertNotNil(src:find('self.activePathFinderVehicle = nil', 1, true),
        'load() must still initialise the active pathfinder vehicle')
end

function TestSchedulerActiveVehicle:testUpdateActiveVehicleActivatesTheQueueHead()
    local first = pathfinderVehicleStub(1)
    local second = pathfinderVehicleStub(2)
    ADScheduler:addPathfinderVehicle(first)
    ADScheduler:addPathfinderVehicle(second)

    ADScheduler:updateActiveVehicle()

    lu.assertEquals(ADScheduler.activePathFinderVehicle, first)
    -- the head is unpaused, the queued one keeps its delay
    lu.assertEquals(first.ad.pathFinderModule.delays[#first.ad.pathFinderModule.delays], 0)
    lu.assertEquals(second.ad.pathFinderModule.delays[#second.ad.pathFinderModule.delays], 5000)
end

function TestSchedulerActiveVehicle:testActiveVehicleRotatesToTheBackOfTheQueue()
    local first = pathfinderVehicleStub(1)
    local second = pathfinderVehicleStub(2)
    ADScheduler:addPathfinderVehicle(first)
    ADScheduler:addPathfinderVehicle(second)

    ADScheduler:updateActiveVehicle()
    ADScheduler:updateActiveVehicle()

    lu.assertEquals(ADScheduler.activePathFinderVehicle, second)
    lu.assertEquals(ADScheduler.pathFinderVehicles, { second, first })
end

function TestSchedulerActiveVehicle:testRemovingTheActiveVehicleShortcutsTheTimer()
    local vehicle = pathfinderVehicleStub(1)
    ADScheduler:addPathfinderVehicle(vehicle)
    ADScheduler:updateActiveVehicle()

    ADScheduler:removePathfinderVehicle(vehicle)

    lu.assertEquals(#ADScheduler.pathFinderVehicles, 0)
    lu.assertEquals(ADScheduler.updateTimer.elapsedTime, ADScheduler.UPDATE_TIME,
        'removing the active vehicle must shortcut the timer so the next one starts at once')
end

------------------------------------------------------------------------------------------------------------------------
--- P6 - the average FPS ring buffer is tracked, not re-summed
------------------------------------------------------------------------------------------------------------------------
TestSchedulerAverageFPS = {}

function TestSchedulerAverageFPS:setUp()
    TestSetup.reset()
    ADScheduler:load()
end

local function feed(frames, dt)
    for _ = 1, frames do
        ADScheduler:update(dt)
    end
end

--- The case the old loop could not handle at all: while the buffer is still filling, the unused
--- slots are nil. The average has to be over the values that are actually there.
function TestSchedulerAverageFPS:testAverageIsCorrectDuringTheInitialFill()
    for _, dt in ipairs({ 10, 20, 30, 40 }) do
        ADScheduler:update(dt)
    end

    ADScheduler:updateAverageFPS()

    lu.assertEquals(ADScheduler.average_filled, 4)
    lu.assertEquals(ADScheduler.average_sum, 100)
    lu.assertAlmostEquals(ADScheduler.average_fps, 1000 / 25, 1e-9)
end

function TestSchedulerAverageFPS:testAverageWithNoFramesYetDoesNotBlowUp()
    ADScheduler:updateAverageFPS()

    lu.assertEquals(ADScheduler.average_fps, 0,
        'with nothing recorded the average stays at its start value')
end

function TestSchedulerAverageFPS:testAverageOverAFullBuffer()
    feed(ADScheduler.FRAMES_TO_CHECK_FOR_AVERAGE_FPS, 20)

    ADScheduler:updateAverageFPS()

    lu.assertEquals(ADScheduler.average_filled, ADScheduler.FRAMES_TO_CHECK_FOR_AVERAGE_FPS)
    lu.assertEquals(ADScheduler.average_sum, 20 * ADScheduler.FRAMES_TO_CHECK_FOR_AVERAGE_FPS)
    lu.assertAlmostEquals(ADScheduler.average_fps, 50, 1e-9)
end

--- Half the ring overwritten: the running sum only stays right if the overwritten value is
--- subtracted again. 1800 slots at 10 ms plus 1800 at 20 ms averages 15 ms.
function TestSchedulerAverageFPS:testOverwrittenSlotsAreSubtractedAgain()
    local size = ADScheduler.FRAMES_TO_CHECK_FOR_AVERAGE_FPS
    feed(size, 20)
    feed(size / 2, 10)

    ADScheduler:updateAverageFPS()

    lu.assertEquals(ADScheduler.average_filled, size, 'the buffer does not grow past its size')
    lu.assertEquals(ADScheduler.average_sum, (size / 2) * 10 + (size / 2) * 20)
    lu.assertAlmostEquals(ADScheduler.average_fps, 1000 / 15, 1e-9)
end

--- And once every slot has been replaced the old values are completely gone.
function TestSchedulerAverageFPS:testFullyReplacedBufferForgetsTheOldValues()
    local size = ADScheduler.FRAMES_TO_CHECK_FOR_AVERAGE_FPS
    feed(size, 20)
    feed(size, 10)

    ADScheduler:updateAverageFPS()

    lu.assertEquals(ADScheduler.average_sum, 10 * size)
    lu.assertAlmostEquals(ADScheduler.average_fps, 100, 1e-9)
end

--- The point of P6 is the cost, and that is only visible in the source: no walk over the buffer.
function TestSchedulerAverageFPS:testUpdateAverageFPSDoesNotWalkTheBuffer()
    local src = readSource('../../scripts/Manager/Scheduler.lua')
    local body = src:match('function ADScheduler:updateAverageFPS%(%)(.-)\nend')
    lu.assertNotNil(body, 'updateAverageFPS must still exist')
    lu.assertNil(body:find('for ', 1, true),
        'updateAverageFPS must not loop - the sum is maintained in update()')
    lu.assertNil(body:find('FRAMES_TO_CHECK_FOR_AVERAGE_FPS', 1, true),
        'dividing by the buffer size is wrong while it is still filling; use the filled count')
end

------------------------------------------------------------------------------------------------------------------------
--- Assigning an unloader that declines
---
--- assignToHarvester can refuse: an unloader sitting in idleUnloaders whose mode has already moved
--- on - following another unloader across the field, say - will not take the job. Booking it as
--- active regardless loses it into the active list with no harvester of its own. Simply not booking
--- it stalls, because the very same one is the closest candidate again on the next tick and the
--- harvester waits with willing vehicles standing beside it. It has to move on to the next.
------------------------------------------------------------------------------------------------------------------------
TestAssignmentDeclines = {}

--- An unloader that accepts or refuses on command, and records that it was asked.
local function candidate(id, marker, accepts, distanceNode)
    local v = TestSetup.vehicle({
        id = id,
        components = { { node = distanceNode } },
        getName = function() return 'Unloader' .. tostring(id) end,
    })
    v.ad.stateModule = stateModuleStub(marker)
    v.asked = 0
    v.ad.modes = {}
    v.ad.modes[AutoDrive.MODE_UNLOAD] = {
        combine = nil,
        assignToHarvester = function(self, harvester)
            v.asked = v.asked + 1
            if accepts then
                self.combine = harvester
            end
        end,
    }
    return v
end

function TestAssignmentDeclines:setUp()
    TestSetup.reset()
    ADHarvestManager:load()
    -- lives in CollisionDetectionUtils, which this suite does not load
    AutoDrive.getDistanceBetween = function(a, b)
        local ax, _, az = getWorldTranslation(a.components[1].node)
        local bx, _, bz = getWorldTranslation(b.components[1].node)
        return MathUtil.vector2Length(ax - bx, az - bz)
    end
    self.harvester = harvesterStub(1, 7)
    MockEngine.nodePositions['node1'] = { x = 0, y = 0, z = 0 }
    -- the refuser is nearer, so it is chosen first
    MockEngine.nodePositions['near'] = { x = 5, y = 0, z = 0 }
    MockEngine.nodePositions['far'] = { x = 40, y = 0, z = 0 }
    self.refuser = candidate(2, 7, false, 'near')
    self.willing = candidate(3, 7, true, 'far')
    ADHarvestManager.idleUnloaders = { self.refuser, self.willing }
    TestState.entities['node1'] = true
end

--- The regression: the harvester used to get nobody at all.
function TestAssignmentDeclines:testItMovesOnToTheNextCandidate()
    ADHarvestManager:assignUnloaderToHarvester(self.harvester)

    lu.assertEquals(self.refuser.asked, 1, 'the nearer one is asked first')
    lu.assertEquals(self.willing.asked, 1, 'and the next one is asked when it declines')
    lu.assertEquals(self.willing.ad.modes[AutoDrive.MODE_UNLOAD].combine, self.harvester)
end

function TestAssignmentDeclines:testTheOneThatAcceptedIsTheOneBooked()
    ADHarvestManager:assignUnloaderToHarvester(self.harvester)

    lu.assertTrue(ADTable.contains(ADHarvestManager.activeUnloaders, self.willing))
    lu.assertFalse(ADTable.contains(ADHarvestManager.activeUnloaders, self.refuser),
        'a vehicle that refused must not be booked as working')
    lu.assertFalse(ADTable.contains(ADHarvestManager.idleUnloaders, self.willing))
    lu.assertTrue(ADTable.contains(ADHarvestManager.idleUnloaders, self.refuser),
        'and it stays idle, so it can be called once its own state allows')
end

--- Everybody refusing has to end, not spin.
function TestAssignmentDeclines:testAllDecliningTerminates()
    self.willing.ad.modes[AutoDrive.MODE_UNLOAD].assignToHarvester = function(self2)
        self.willing.asked = self.willing.asked + 1
    end

    ADHarvestManager:assignUnloaderToHarvester(self.harvester)

    lu.assertEquals(self.refuser.asked, 1)
    lu.assertEquals(self.willing.asked, 1)
    lu.assertEquals(#ADHarvestManager.activeUnloaders, 0)
end

function TestAssignmentDeclines:testNoCandidatesIsHarmless()
    ADHarvestManager.idleUnloaders = {}

    ADHarvestManager:assignUnloaderToHarvester(self.harvester)

    lu.assertEquals(#ADHarvestManager.activeUnloaders, 0)
end

--- The ordinary case still takes the closest one and asks nobody else.
function TestAssignmentDeclines:testAWillingClosestOneIsTakenStraightAway()
    self.refuser.ad.modes[AutoDrive.MODE_UNLOAD].assignToHarvester = function(self2, harvester)
        self.refuser.asked = self.refuser.asked + 1
        self2.combine = harvester
    end

    ADHarvestManager:assignUnloaderToHarvester(self.harvester)

    lu.assertEquals(self.refuser.asked, 1)
    lu.assertEquals(self.willing.asked, 0, 'no need to trouble anybody else')
end



------------------------------------------------------------------------------------------------------------------------
--- The second unloader call has to walk past a candidate that cannot take it
---
--- driveToUnloader only acts on a mode still waiting to be called, and nothing removes a spare from
--- idleUnloaders when it is committed - so a driver already following somebody else stayed the
--- closest candidate and was silently re-picked on every tick, while one parked a few metres further
--- on was never asked. Two combines on one field, four drivers: the second combine got no relief for
--- the rest of the field. The sibling call site has carried a skip list for exactly this since it
--- was found there; getClosestIdleUnloader has taken one all along.
------------------------------------------------------------------------------------------------------------------------
TestSecondUnloaderCall = {}

--- `available` models the mode still being in its waiting state. A committed driver refuses, which
--- in the real mode is driveToUnloader returning without doing anything.
local function spareUnloader(id, x, available)
    local v = TestSetup.vehicle({
        id = id,
        components = { { node = 'u' .. tostring(id) } },
        getName = function() return 'U' .. tostring(id) end,
    })
    MockEngine.nodePositions['u' .. tostring(id)] = { x = x, y = 0, z = 0 }
    v.ad.stateModule = stateModuleStub(1)
    v.ad.modes = {
        [AutoDrive.MODE_UNLOAD] = {
            available = available,
            follower = nil,
            getFollowingUnloader = function(self) return self.follower end,
            driveToUnloader = function(self, caller)
                if self.available then
                    caller.ad.modes[AutoDrive.MODE_UNLOAD].follower = v
                    self.available = false
                end
            end,
        },
    }
    return v
end

function TestSecondUnloaderCall:setUp()
    TestSetup.reset()
    ADHarvestManager:load()
    self.harvester = harvesterStub(1, 1)
    MockEngine.nodePositions['node1'] = { x = 0, y = 0, z = 0 }
    -- the driver currently unloading, which is the one asking for company
    self.caller = spareUnloader(2, 0, false)
end

function TestSecondUnloaderCall:testAnAvailableSpareIsCalled()
    local free = spareUnloader(3, 100, true)
    table.insert(ADHarvestManager.idleUnloaders, free)

    lu.assertTrue(ADHarvestManager:callSecondUnloaderFor(self.harvester, self.caller))
    lu.assertEquals(self.caller.ad.modes[AutoDrive.MODE_UNLOAD]:getFollowingUnloader(), free)
end

--- The measured case: the nearest one is already committed, and four metres behind it stands one
--- that is not.
function TestSecondUnloaderCall:testACommittedSpareDoesNotBlockTheOneBehindIt()
    local committed = spareUnloader(3, 100, false)
    local free = spareUnloader(4, 104, true)
    table.insert(ADHarvestManager.idleUnloaders, committed)
    table.insert(ADHarvestManager.idleUnloaders, free)

    lu.assertTrue(ADHarvestManager:callSecondUnloaderFor(self.harvester, self.caller),
        'the closest candidate cannot take the call, so the next one has to be asked')
    lu.assertEquals(self.caller.ad.modes[AutoDrive.MODE_UNLOAD]:getFollowingUnloader(), free)
end

--- And when nobody can take it, the walk ends rather than looping.
function TestSecondUnloaderCall:testItGivesUpWhenNobodyCanTakeIt()
    table.insert(ADHarvestManager.idleUnloaders, spareUnloader(3, 100, false))
    table.insert(ADHarvestManager.idleUnloaders, spareUnloader(4, 104, false))

    lu.assertFalse(ADHarvestManager:callSecondUnloaderFor(self.harvester, self.caller))
    lu.assertNil(self.caller.ad.modes[AutoDrive.MODE_UNLOAD]:getFollowingUnloader())
end

os.exit(lu.LuaUnit.run())
