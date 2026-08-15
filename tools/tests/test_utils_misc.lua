--[[
Tests for the "utils-misc" group.

Covers findings:
  A19 checkForVehiclePathInBox always returned false when called without a direction vector
  A42 checkForVehiclesInBox ran the allocating checkIsConnected before the cheap distance cull
  A43 Queue:Peek / SortedQueue:peek decremented the count without removing anything
  A45 readGraphFromXml discarded the error codes of its own check helpers
  A20 getRefuelTriggers rescanned every load trigger on the map once per frame
  A22 checkIfPathTraversedOverPosition recursed without a visited set
  A29 DONT_PROPAGATE was passed to addTask instead of to the task constructor

All modules in this group load standalone, so these are real unit tests against the shipped code.
The one exception is the DriveToMode argument-placement guard at the bottom, which is a
source-level contract test: a call that passes an ignored extra argument cannot be observed from
the outside, only the constructor side can, so the source is checked as well.
]]

lu = require('luaunit')
require('test-setup')

require('UtilFuncs')
require('PathFinderUtils')
require('Queue')
require('SortedQueue')
require('CollisionDetectionUtils')
require('TriggerManager')
require('AutoDrivePlaceableData')
require('AbstractMode')
require('TaskModule')
require('DriveToMode')

------------------------------------------------------------------------------------------------------------------------
--- Engine functions the mock does not provide
------------------------------------------------------------------------------------------------------------------------
--- mock-engine.lua deliberately has no XML layer, no entityExists and no string.split. They are
--- defined here rather than there so the mock stays untouched; see the report for the request to
--- add them centrally.
function entityExists() return true end
function getWorldRotation() return 0, 0, 0 end
function setRotation() end

--- Giants' string.split, as the mod uses it: split on a single-character separator.
if string.split == nil then
    function string.split(s, sep)
        local out = {}
        if s == nil then
            error('string.split called with nil - the guard under test should have prevented this')
        end
        for token in string.gmatch(s, '([^' .. sep .. ']+)') do
            table.insert(out, token)
        end
        return out
    end
end

------------------------------------------------------------------------------------------------------------------------
--- A43 - Peek must look, not take
------------------------------------------------------------------------------------------------------------------------
TestQueue = {}

function TestQueue:setUp() TestSetup.reset() end

function TestQueue:testPeekLeavesTheQueueUntouched()
    local q = Queue:new()
    q:Enqueue('a')
    q:Enqueue('b')

    lu.assertEquals(q:Peek(), 'a')
    lu.assertEquals(q:Peek(), 'a')
    lu.assertEquals(q:Peek(), 'a')
    lu.assertEquals(q:Count(), 2, 'Peek must not consume the count')
    lu.assertEquals(#q:GetItems(), 2)
end

--- The drift was the actual damage: Count() went to zero while items were still queued, so a
--- `while Count() > 0 do Dequeue() end` loop left work behind.
function TestQueue:testCountStaysInSyncWithTheItems()
    local q = Queue:new()
    for _, v in ipairs({ 1, 2, 3 }) do q:Enqueue(v) end
    for _ = 1, 5 do q:Peek() end

    lu.assertEquals(q:Count(), #q:GetItems())
    lu.assertEquals(q:Dequeue(), 1)
    lu.assertEquals(q:Dequeue(), 2)
    lu.assertEquals(q:Dequeue(), 3)
    lu.assertEquals(q:Count(), 0)
    lu.assertNil(q:Dequeue())
end

function TestQueue:testPeekOnEmptyQueue()
    local q = Queue:new()
    lu.assertNil(q:Peek())
    lu.assertEquals(q:Count(), 0)
end

TestSortedQueue = {}

function TestSortedQueue:setUp() TestSetup.reset() end

--- SortedQueue:peek called self:Count(), which this class never had - so it raised
--- "attempt to call a nil value" on the first call rather than returning the head.
function TestSortedQueue:testPeekReturnsTheSmallestWithoutRemovingIt()
    local q = SortedQueue:new('distance')
    q:enqueue({ distance = 5 })
    q:enqueue({ distance = 1 })
    q:enqueue({ distance = 3 })

    lu.assertEquals(q:peek().distance, 1)
    lu.assertEquals(q:peek().distance, 1)
    lu.assertEquals(q:count(), 3)
    lu.assertEquals(q:dequeue().distance, 1)
    lu.assertEquals(q:count(), 2)
    lu.assertEquals(q:peek().distance, 3)
end

function TestSortedQueue:testPeekOnEmptyQueueReturnsNil()
    local q = SortedQueue:new('distance')
    lu.assertNil(q:peek())
    lu.assertTrue(q:empty())
end

--- The pathfinder drains this queue with empty()/dequeue(); a drifting count ends the search early.
function TestSortedQueue:testDrainsCompletely()
    local q = SortedQueue:new('distance')
    for _, d in ipairs({ 7, 2, 9, 4 }) do q:enqueue({ distance = d }) end

    local drained = {}
    while not q:empty() do
        q:peek()
        table.insert(drained, q:dequeue().distance)
    end
    lu.assertEquals(drained, { 2, 4, 7, 9 })
end

------------------------------------------------------------------------------------------------------------------------
--- A42 / A19 - collision detection
------------------------------------------------------------------------------------------------------------------------
TestCollisionDetection = {}

local function boxAround(cx, cz, half)
    return {
        { x = cx - half, y = 0, z = cz - half },
        { x = cx + half, y = 0, z = cz - half },
        { x = cx + half, y = 0, z = cz + half },
        { x = cx - half, y = 0, z = cz + half },
    }
end

local function otherVehicleAt(node, x, z)
    MockEngine.nodePositions[node] = { x = x, y = 0, z = z }
    return TestSetup.vehicle({
        components = { { node = node } },
        rootNode = node,
        ad = nil,
    })
end

function TestCollisionDetection:setUp()
    TestSetup.reset()
    self.connectedCalls = 0
    --- AutoDriveUtilFuncs is not loaded here; these two live there and are the ones under test.
    AutoDrive.checkIsConnected = function(_, _, _)
        TestCollisionDetection.instance.connectedCalls = TestCollisionDetection.instance.connectedCalls + 1
        return false
    end
    AutoDrive.localDirectionToWorld = function() return 0, 0, 1 end
    TestCollisionDetection.instance = self
end

function TestCollisionDetection:testDistanceCullRunsBeforeTheConnectionCheck()
    local far = otherVehicleAt('farNode', 1000, 1000)
    AutoDrive.getAllVehicles = function() return { far } end

    local excluded = { otherVehicleAt('excludedNode', 0, 0) }
    lu.assertFalse(AutoDrive.checkForVehiclesInBox(boxAround(0, 0, 2), excluded))
    lu.assertEquals(self.connectedCalls, 0,
        'checkIsConnected walks the whole implement chain and must not run for a vehicle 1 km away')
end

--- The cull must not have swallowed the exclusion logic itself.
function TestCollisionDetection:testConnectionCheckStillRunsForNearbyVehicles()
    local near = otherVehicleAt('nearNode', 5, 5)
    AutoDrive.getAllVehicles = function() return { near } end
    AutoDrive.checkIsConnected = function(_, _, _)
        TestCollisionDetection.instance.connectedCalls = TestCollisionDetection.instance.connectedCalls + 1
        return true -- pretend the near vehicle is our own trailer
    end

    local excluded = { otherVehicleAt('excludedNode', 0, 0) }
    lu.assertFalse(AutoDrive.checkForVehiclesInBox(boxAround(5, 5, 4), excluded),
        'a connected vehicle is still excluded')
    lu.assertEquals(self.connectedCalls, 1)
end

function TestCollisionDetection:testNearbyUnrelatedVehicleIsACollision()
    local near = otherVehicleAt('nearNode', 5, 5)
    AutoDrive.getAllVehicles = function() return { near } end

    lu.assertTrue(AutoDrive.checkForVehiclesInBox(boxAround(5, 5, 4), {}))
end

function TestCollisionDetection:testTrainsAndRunningConveyorBeltsAreIgnored()
    local train = otherVehicleAt('trainNode', 5, 5)
    train.trainSystem = {}
    AutoDrive.getAllVehicles = function() return { train } end

    lu.assertFalse(AutoDrive.checkForVehiclesInBox(boxAround(5, 5, 4), {}))
end

------------------------------------------------------------------------------------------------------------------------
--- A19 - the angle test must be skipped, not failed, when no direction vector is supplied
------------------------------------------------------------------------------------------------------------------------
TestVehiclePathInBox = {}

function TestVehiclePathInBox:setUp()
    TestSetup.reset()

    --- A pathfinder route running straight along +x, long enough that the index window
    --- (2 < index < #wps - 5) inside checkForVehiclePathInBox is non-empty.
    local wps = {}
    for i = 1, 14 do
        wps[i] = { x = i * 5, y = 0, z = 0, isPathFinderPoint = true }
    end

    local other = TestSetup.vehicle({
        components = { { node = 'otherPathNode' } },
        rootNode = 'otherPathNode',
    })
    other.ad.drivePathModule = { getWayPoints = function() return wps, 1 end }
    other.ad.stateModule = { isActive = function() return true end }

    self.other = other
    self.searching = TestSetup.vehicle({ components = { { node = 'searchNode' } }, rootNode = 'searchNode' })
    AutoDrive.getAllVehicles = function() return { other } end

    -- big enough to overlap every cell box built from that route
    self.box = boxAround(35, 0, 100)
end

function TestVehiclePathInBox:testReportsAHitWithoutADirectionVector()
    lu.assertTrue(AutoDrive.checkForVehiclePathInBox(self.box, 10, self.searching),
        'with no direction vector to compare against, the angle test must be skipped, not treated '
        .. 'as a mismatch - otherwise this call can never report a hit')
end

function TestVehiclePathInBox:testReportsAHitForAParallelDirectionVector()
    lu.assertTrue(AutoDrive.checkForVehiclePathInBox(self.box, 10, self.searching, { x = -5, z = 0 }))
end

--- The angle test still has to do its job, otherwise the fix would just be "return true".
function TestVehiclePathInBox:testNoHitForAPerpendicularDirectionVector()
    lu.assertFalse(AutoDrive.checkForVehiclePathInBox(self.box, 10, self.searching, { x = 0, z = -5 }))
end

function TestVehiclePathInBox:testNoHitWhenTheBoxIsElsewhere()
    lu.assertFalse(AutoDrive.checkForVehiclePathInBox(boxAround(5000, 5000, 5), 10, self.searching))
end

------------------------------------------------------------------------------------------------------------------------
--- A45 - readGraphFromXml must return the error codes its helpers produce
------------------------------------------------------------------------------------------------------------------------
TestPlaceableData = {}

function TestPlaceableData:setUp()
    TestSetup.reset()
    self.xmlValues = {}
    hasXMLProperty = function(_, key) return self.xmlValues[key] ~= nil end
    getXMLString = function(_, key) return self.xmlValues[key] end
    getXMLFloat = function() return nil end -- no map markers in the file, so that loop exits at once
    ADGraphManager = {
        getWayPointsCount = function() return 0 end,
        getMapMarkers = function() return {} end,
    }
end

--- Before the fix the -1 was thrown away and execution fell through to string.split(nil, ",").
function TestPlaceableData:testMissingPropertyReturnsMinusOne()
    lu.assertEquals(AutoDrivePlaceableData.readGraphFromXml(1, {}), -1)
end

function TestPlaceableData:testMissingValueReturnsMinusTwo()
    -- every property exists, but the waypoint lists carry no value
    hasXMLProperty = function() return true end
    getXMLString = function() return nil end
    lu.assertEquals(AutoDrivePlaceableData.readGraphFromXml(1, {}), -2)
end

--- A negative code has to survive all the way out, because onFinalizePlacement branches on `ret < 0`.
function TestPlaceableData:testErrorCodeIsNegative()
    lu.assertTrue(AutoDrivePlaceableData.readGraphFromXml(1, {}) < 0)
end

--- Control: a complete, consistent set of keys must NOT be rejected by the new guards.
function TestPlaceableData:testValidDataIsAccepted()
    AutoDrive.getTerrainHeightAtWorldPos = function() return 0 end
    self.xmlValues = {
        ['placeable'] = 'x',
        ['placeable.AutoDrive'] = 'x',
        ['placeable.AutoDrive.wayPoints'] = 'x',
        ['placeable.AutoDrive.waypoints.x'] = '10,20',
        ['placeable.AutoDrive.waypoints.y'] = '0,0',
        ['placeable.AutoDrive.waypoints.z'] = '30,40',
        ['placeable.AutoDrive.waypoints.out'] = '2;-1',
        ['placeable.AutoDrive.waypoints.incoming'] = '-1;1',
        ['placeable.AutoDrive.waypoints.flags'] = '0,0',
    }

    lu.assertEquals(AutoDrivePlaceableData.readGraphFromXml(1, { rootNode = 'placeableRoot' }), 0)
    lu.assertEquals(#AutoDrivePlaceableData.wayPoints, 2)
end

------------------------------------------------------------------------------------------------------------------------
--- A20 - the refuel trigger scan is cached, the fill levels are not
------------------------------------------------------------------------------------------------------------------------
TestRefuelTriggers = {}

function TestRefuelTriggers:setUp()
    TestSetup.reset()

    self.flagChecks = 0
    CollisionFlag.getHasMaskFlagSet = function(node)
        self.flagChecks = self.flagChecks + 1
        return node ~= 'plainNode'
    end
    CollisionFlag.getHasGroupFlagSet = function() return true end

    self.fuelFillLevelReads = 0
    self.plainFillLevelReads = 0

    self.fuelTrigger = {
        triggerNode = 'fuelNode',
        fillTypes = { [3] = true },
        source = {
            getAllFillLevels = function()
                self.fuelFillLevelReads = self.fuelFillLevelReads + 1
                return { [3] = 5000 }
            end,
        },
    }
    self.plainTrigger = {
        triggerNode = 'plainNode',
        fillTypes = { [4] = true },
        source = {
            getAllFillLevels = function()
                self.plainFillLevelReads = self.plainFillLevelReads + 1
                return { [4] = 5000 }
            end,
        },
    }

    ADTriggerManager.searchedForTriggers = true
    ADTriggerManager.siloTriggers = { self.fuelTrigger, self.plainTrigger }
    ADTriggerManager.invalidateRefuelTriggerCandidates()

    AutoDrive.getRequiredRefuels = function() return { 3 } end
    self.vehicle = TestSetup.vehicle({ getOwnerFarmId = function() return 1 end })
end

function TestRefuelTriggers:testFindsTheMatchingTrigger()
    local triggers = ADTriggerManager.getRefuelTriggers(self.vehicle)
    lu.assertEquals(#triggers, 1)
    lu.assertIs(triggers[1], self.fuelTrigger)
end

--- The map scan is what used to run every frame. It must happen once for the whole trigger set.
function TestRefuelTriggers:testMapScanHappensOnceForRepeatedCalls()
    for _ = 1, 10 do ADTriggerManager.getRefuelTriggers(self.vehicle) end
    lu.assertEquals(self.flagChecks, 2,
        'one collision flag test per load trigger, not per load trigger per frame')
end

--- Only candidates may be asked for fill levels - that table allocation per trigger per frame was
--- the reported cost.
function TestRefuelTriggers:testNonCandidateTriggersAreNeverAskedForFillLevels()
    for _ = 1, 10 do ADTriggerManager.getRefuelTriggers(self.vehicle) end
    lu.assertEquals(self.plainFillLevelReads, 0)
end

--- ...but the fill levels themselves must stay fresh, they change with every load and unload.
function TestRefuelTriggers:testFillLevelsAreReadOnEveryCall()
    for _ = 1, 10 do ADTriggerManager.getRefuelTriggers(self.vehicle) end
    lu.assertEquals(self.fuelFillLevelReads, 10)
end

function TestRefuelTriggers:testEmptyTriggerIsNotReturned()
    self.fuelTrigger.source.getAllFillLevels = function() return { [3] = 0 } end
    lu.assertEquals(#ADTriggerManager.getRefuelTriggers(self.vehicle), 0)
end

function TestRefuelTriggers:testExplicitInvalidationRescans()
    ADTriggerManager.getRefuelTriggers(self.vehicle)
    lu.assertEquals(self.flagChecks, 2)

    ADTriggerManager.invalidateRefuelTriggerCandidates()
    ADTriggerManager.getRefuelTriggers(self.vehicle)
    lu.assertEquals(self.flagChecks, 4)
end

--- A trigger removed from the map must disappear from the cached scan too, otherwise the cache
--- would keep a dead trigger alive forever.
function TestRefuelTriggers:testDeletingATriggerInvalidatesTheCache()
    lu.assertEquals(#ADTriggerManager.getRefuelTriggers(self.vehicle), 1)

    ADTriggerManager.loadTriggerDelete(self.fuelTrigger, function() end)
    lu.assertEquals(#ADTriggerManager.getRefuelTriggers(self.vehicle), 0)
end

function TestRefuelTriggers:testAddingATriggerInvalidatesTheCache()
    ADTriggerManager.siloTriggers = { self.plainTrigger }
    ADTriggerManager.invalidateRefuelTriggerCandidates()
    lu.assertEquals(#ADTriggerManager.getRefuelTriggers(self.vehicle), 0)

    ADTriggerManager.loadTriggerLoad(self.fuelTrigger, function() return true end)
    lu.assertEquals(#ADTriggerManager.getRefuelTriggers(self.vehicle), 1)
end

------------------------------------------------------------------------------------------------------------------------
--- A22 - checkIfPathTraversedOverPosition needs a visited set
------------------------------------------------------------------------------------------------------------------------
TestPathTraversal = {}

function TestPathTraversal:setUp()
    TestSetup.reset()

    --- A dense chain: every node lists both neighbours as incoming, which is what a two-way road
    --- looks like in the AutoDrive network. Without a visited set the search walks back into the
    --- node it came from and branches at every step.
    local n = 20
    local wps = {}
    for i = 1, n do
        wps[i] = TestSetup.waypoint(i, i * 10, 0, {}, {})
    end
    for i = 1, n do
        if i > 1 then table.insert(wps[i].incoming, i - 1) end
        if i < n then table.insert(wps[i].incoming, i + 1) end
    end
    self.wps = wps

    self.lookups = 0
    ADGraphManager = {
        getWayPointById = function(_, id)
            self.lookups = self.lookups + 1
            return self.wps[id]
        end,
    }
end

function TestPathTraversal:testUnreachableTargetTerminatesQuickly()
    local hit = AutoDrive:checkIfPathTraversedOverPosition(self.wps[10], { x = -5000, z = -5000 }, 5, 20)
    lu.assertFalse(hit)
    lu.assertTrue(self.lookups < 500,
        'a 20 node two-way chain searched 20 steps deep took ' .. tostring(self.lookups)
        .. ' lookups - without the visited set this is exponential')
end

--- The pruning must not cost results: a target on the path is still found.
function TestPathTraversal:testFindsATargetOnThePath()
    lu.assertTrue(AutoDrive:checkIfPathTraversedOverPosition(self.wps[10], { x = 50, z = 0 }, 5, 20))
end

function TestPathTraversal:testFindsATargetAtTheStartNode()
    lu.assertTrue(AutoDrive:checkIfPathTraversedOverPosition(self.wps[10], { x = 100, z = 0 }, 5, 20))
end

--- The step budget is still honoured: node 5 is five hops from node 10, so three steps cannot
--- reach it.
function TestPathTraversal:testStepLimitIsStillEnforced()
    lu.assertFalse(AutoDrive:checkIfPathTraversedOverPosition(self.wps[10], { x = 50, z = 0 }, 5, 3))
    lu.assertTrue(AutoDrive:checkIfPathTraversedOverPosition(self.wps[10], { x = 50, z = 0 }, 5, 6))
end

function TestPathTraversal:testExhaustedBudgetReturnsFalse()
    lu.assertFalse(AutoDrive:checkIfPathTraversedOverPosition(self.wps[10], { x = 100, z = 0 }, 5, 0))
end

------------------------------------------------------------------------------------------------------------------------
--- A29 - DONT_PROPAGATE goes to the task, not to addTask
------------------------------------------------------------------------------------------------------------------------
TestDriveToMode = {}

function TestDriveToMode:setUp()
    TestSetup.reset()

    self.constructed = nil
    self.addTaskCalls = {}

    StopAndDisableADTask = {
        new = function(_, vehicle, propagate, restart)
            self.constructed = { vehicle = vehicle, propagate = propagate, restart = restart }
            return { isStopTask = true }
        end,
    }
    ADGraphManager = { getMapMarkerByWayPointId = function() return nil end }
    ADMessagesManager = { messageTypes = { INFO = 1, WARN = 2, ERROR = 3 } }
    AutoDriveMessageEvent = { sendMessageOrNotification = function() end }

    local vehicle = TestSetup.vehicle()
    vehicle.ad.trailerModule = { reset = function() end }
    vehicle.ad.stateModule = {
        getFirstMarker = function() return { id = 7, name = 'Farm' } end,
        getName = function() return 'Driver' end,
    }
    vehicle.ad.isStoppingWithError = false
    vehicle.ad.taskModule = {
        addTask = function(_, task, extraArgument)
            table.insert(self.addTaskCalls, { task = task, extraArgument = extraArgument })
        end,
    }
    self.vehicle = vehicle

    self.mode = DriveToMode:new(vehicle)
    self.mode.driveToDestinationTask = { isDriveTask = true }
    self.mode.destinationID = 7
end

function TestDriveToMode:testStopTaskReceivesDontPropagate()
    self.mode:handleFinishedTask()

    lu.assertNotNil(self.constructed, 'a StopAndDisableADTask must be constructed')
    lu.assertEquals(self.constructed.propagate, ADTaskModule.DONT_PROPAGATE,
        'DONT_PROPAGATE belongs in the constructor - the task passes it on to '
        .. 'setCurrentTaskFinished, which is the only place it has an effect')
    lu.assertIs(self.constructed.vehicle, self.vehicle)
end

function TestDriveToMode:testAddTaskGetsOnlyTheTask()
    self.mode:handleFinishedTask()

    lu.assertEquals(#self.addTaskCalls, 1)
    lu.assertNil(self.addTaskCalls[1].extraArgument,
        'addTask takes one argument; anything else is silently dropped')
    lu.assertTrue(self.addTaskCalls[1].task.isStopTask)
end

--- Source-level guard for "check the same mistake is not repeated in this file": an argument that
--- addTask ignores leaves no trace at runtime, so the call sites are inspected directly.
function TestDriveToMode:testNoAddTaskCallSitePassesAnExtraArgument()
    local f = io.open('../../scripts/Modes/DriveToMode.lua', 'r')
    lu.assertNotNil(f, 'run the tests from tools/tests')
    local src = f:read('*a')
    f:close()

    local function topLevelArgumentCount(argumentList)
        if argumentList:match('^%s*$') then
            return 0
        end
        local depth, count = 0, 1
        for i = 1, #argumentList do
            local c = argumentList:sub(i, i)
            if c == '(' then
                depth = depth + 1
            elseif c == ')' then
                depth = depth - 1
            elseif c == ',' and depth == 0 then
                count = count + 1
            end
        end
        return count
    end

    local seen = 0
    for call in src:gmatch('addTask%b()') do
        seen = seen + 1
        local argumentList = call:sub(#'addTask' + 2, -2)
        lu.assertEquals(topLevelArgumentCount(argumentList), 1,
            'addTask takes exactly one argument, found: addTask(' .. argumentList .. ')')
    end
    lu.assertTrue(seen >= 2, 'expected to find the addTask call sites in DriveToMode.lua')
end

os.exit(lu.LuaUnit.run())
