--[[
Tests for scripts/ExternalInterface.lua - the API other mods (Courseplay, AIVE) call into.

Covers findings:
  B1 GetClosestPointToLocation searched the network only when it was EMPTY, so the GetPath /
     GetPathVia fallback always returned -1
  B2 StopCP wrote vehicleToCheck.ad.restartCP outside the "ad ~= nil" guard
  B7 onCpEmpty / onCpFull built a stub `ad = {}` table on vehicles without AutoDrive data,
     which made every later "vehicle.ad ~= nil" check pass with stateModule and settings missing

ExternalInterface.lua is one of the few AutoDrive files that loads standalone: it only defines
functions on the AutoDrive namespace. Its collaborators are supplied here as fakes rather than by
requiring the real modules - ADGraphManager in particular is a large module whose internals are
irrelevant to these three fixes, and a fake keeps this suite from failing for reasons that have
nothing to do with ExternalInterface. The fakes reproduce the contract the code under test relies
on: getWayPointsCount returns #wayPoints, FastShortestPath returns {} for unusable input.
]]

lu = require('luaunit')
require('test-setup')
require('ExternalInterface')

--- Debug output is a no-op here; the mod's own debugPrint lives in UtilFuncs and needs the
--- vehicle/HUD plumbing. Only defined when the namespace does not already carry it.
if AutoDrive.debugPrint == nil then
    function AutoDrive.debugPrint() end
end
if AutoDrive.normalizeAngleToPlusMinusPI == nil then
    function AutoDrive.normalizeAngleToPlusMinusPI(inputAngle)
        while inputAngle > math.pi do inputAngle = inputAngle - 2 * math.pi end
        while inputAngle < -math.pi do inputAngle = inputAngle + 2 * math.pi end
        return inputAngle
    end
end
--- The helper ids only have to be distinct: the stub state module below answers with the very
--- same constant the code under test compares against, so the literal value is never asserted on.
if ADStateModule == nil then
    ADStateModule = { HELPER_CP = 1, HELPER_AIVE = 2, HELPER_AI = 3 }
end

------------------------------------------------------------------------------------------------------------------------
--- Fake graph manager
------------------------------------------------------------------------------------------------------------------------
local FakeGraph = {}
FakeGraph.__index = FakeGraph

function FakeGraph.new(wayPoints, mapMarkers)
    local self = setmetatable({}, FakeGraph)
    self.wayPoints = wayPoints or {}
    self.mapMarkers = mapMarkers or {}
    self.matchingWayPoint = -1      -- what findMatchingWayPoint answers; -1 forces the fallback
    self.pathRequests = {}
    return self
end

function FakeGraph:getWayPoints() return self.wayPoints end
function FakeGraph:getWayPointById(id) return self.wayPoints[id] end
function FakeGraph:getWayPointsCount() return #self.wayPoints end
function FakeGraph:getMapMarkers() return self.mapMarkers end
function FakeGraph:getMapMarkerById(id) return self.mapMarkers[id] end

function FakeGraph:getWayPointsInRange(point, rangeMin, rangeMax)
    local inRange = {}
    for _, wp in pairs(self.wayPoints) do
        local dis = MathUtil.vector2Length(wp.x - point.x, wp.z - point.z)
        if dis < rangeMax and dis > rangeMin then
            table.insert(inRange, wp.id)
        end
    end
    return inRange
end

function FakeGraph:findMatchingWayPoint()
    return self.matchingWayPoint
end

--- Walks the straight test chain from `start` to the marker's waypoint, and mirrors the real
--- implementation in returning an empty table - never nil - when it cannot start.
--- The target is looked up by NAME, as GraphManager does; markerId is only recorded, because how
--- that argument is resolved belongs to GraphManager and not to this suite. The markers below
--- therefore carry distinct names, so the lookup rule cannot influence the assertions.
function FakeGraph:FastShortestPath(start, markerName, markerId)
    table.insert(self.pathRequests, { start = start, markerName = markerName, markerId = markerId })
    local targetId = nil
    for _, marker in pairs(self.mapMarkers) do
        if marker.name == markerName then
            targetId = marker.id
            break
        end
    end
    if start == nil or start == 0 or targetId == nil or self.wayPoints[start] == nil then
        return {}
    end
    local path = {}
    local step = targetId >= start and 1 or -1
    for i = start, targetId, step do
        table.insert(path, self.wayPoints[i])
    end
    return path
end

local function pathIds(path)
    local ids = {}
    for _, wp in ipairs(path or {}) do table.insert(ids, wp.id) end
    return ids
end

--- Chain of waypoints at x = 10, 20, 30, 40, 50 on z = 0.
local function chainNetwork()
    local wps = {}
    for i = 1, 5 do
        wps[i] = TestSetup.waypoint(i, i * 10, 0,
            i < 5 and { i + 1 } or {},
            i > 1 and { i - 1 } or {})
    end
    return wps
end

------------------------------------------------------------------------------------------------------------------------
--- B1 - the closest point fallback
------------------------------------------------------------------------------------------------------------------------
TestClosestPointToLocation = {}

function TestClosestPointToLocation:setUp()
    TestSetup.reset()
    ADGraphManager = FakeGraph.new(chainNetwork(), {
        [7] = { id = 3, name = 'Silo', group = 'All' },
        [9] = { id = 5, name = 'Hof', group = 'All' },
    })
end

function TestClosestPointToLocation:testFindsTheNearestWayPoint()
    -- (21, 0) sits 1 m past waypoint 2
    lu.assertEquals(AutoDrive:GetClosestPointToLocation(21, 0, 0), 2,
        'the search must run when the network HAS waypoints; guarding it with "count < 1" made '
        .. 'this fallback return -1 for every non-empty network')
end

function TestClosestPointToLocation:testEmptyNetworkReturnsMinusOne()
    ADGraphManager = FakeGraph.new({}, {})
    lu.assertEquals(AutoDrive:GetClosestPointToLocation(21, 0, 0), -1)
end

function TestClosestPointToLocation:testPointsCloserThanMinDistanceAreSkipped()
    -- from (21, 0): wp2 is 1 m away, wp3 is 9 m, wp1 is 11 m
    lu.assertEquals(AutoDrive:GetClosestPointToLocation(21, 0, 5), 3)
end

function TestClosestPointToLocation:testMinDistanceLargerThanTheWholeNetworkReturnsMinusOne()
    lu.assertEquals(AutoDrive:GetClosestPointToLocation(21, 0, 1000), -1)
end

------------------------------------------------------------------------------------------------------------------------
--- B1 - the two callers of that fallback
------------------------------------------------------------------------------------------------------------------------
TestGetPath = {}

function TestGetPath:setUp()
    TestSetup.reset()
    ADGraphManager = FakeGraph.new(chainNetwork(), {
        [7] = { id = 3, name = 'Silo', group = 'All' },
        [9] = { id = 5, name = 'Hof', group = 'All' },
    })
    ADGraphManager.matchingWayPoint = -1     -- no directional match, so the fallback decides
end

function TestGetPath:testFallsBackToTheClosestPointWhenNoWayPointMatchesTheHeading()
    local path = AutoDrive:GetPath(21, 0, 0, 7)
    lu.assertEquals(pathIds(path), { 2, 3 },
        'with no directional match GetPath must route from the closest waypoint')
    lu.assertEquals(#ADGraphManager.pathRequests, 1)
    lu.assertEquals(ADGraphManager.pathRequests[1].start, 2)
end

function TestGetPath:testDirectionalMatchStillWins()
    ADGraphManager.matchingWayPoint = 1
    local path = AutoDrive:GetPath(21, 0, 0, 7)
    lu.assertEquals(pathIds(path), { 1, 2, 3 })
    lu.assertEquals(ADGraphManager.pathRequests[1].start, 1)
end

function TestGetPath:testEmptyNetworkYieldsNoPathAndNoPathRequest()
    ADGraphManager = FakeGraph.new({}, { [7] = { id = 3, name = 'Silo', group = 'All' } })
    lu.assertNil(AutoDrive:GetPath(21, 0, 0, 7))
    lu.assertEquals(#ADGraphManager.pathRequests, 0)
end

function TestGetPath:testUnknownDestinationStillReturnsNothing()
    lu.assertNil(AutoDrive:GetPath(21, 0, 0, 42))
end

TestGetPathVia = {}

function TestGetPathVia:setUp()
    TestSetup.reset()
    ADGraphManager = FakeGraph.new(chainNetwork(), {
        [7] = { id = 3, name = 'Silo', group = 'All' },
        [9] = { id = 5, name = 'Hof', group = 'All' },
    })
    ADGraphManager.matchingWayPoint = -1
end

function TestGetPathVia:testFallbackPathIsJoinedAtTheViaPoint()
    local path = AutoDrive:GetPathVia(21, 0, 0, 7, 9)
    lu.assertEquals(pathIds(path), { 2, 3, 4, 5 },
        'the via point must appear once: leg one ends on it, leg two starts on it')
    lu.assertEquals(ADGraphManager.pathRequests[1].start, 2)
    lu.assertEquals(ADGraphManager.pathRequests[2].start, 3)
end

function TestGetPathVia:testEmptyNetworkYieldsNoPath()
    ADGraphManager = FakeGraph.new({}, {
        [7] = { id = 3, name = 'Silo', group = 'All' },
        [9] = { id = 5, name = 'Hof', group = 'All' },
    })
    lu.assertNil(AutoDrive:GetPathVia(21, 0, 0, 7, 9))
    lu.assertEquals(#ADGraphManager.pathRequests, 0)
end

------------------------------------------------------------------------------------------------------------------------
--- B2 - StopCP on a Courseplay vehicle that has no AutoDrive data
------------------------------------------------------------------------------------------------------------------------
TestStopCP = {}

--- A vehicle with the Courseplay interface. `ad` is added by the individual tests, because its
--- absence is exactly what is under test here.
local function cpVehicle()
    local v = {
        cpActive = true,
        cpStartStopCalls = 0,
        getRootVehicle = function(self) return self end,
        getIsCpActive = function(self) return self.cpActive end,
    }
    v.cpStartStopDriver = function(self)
        self.cpStartStopCalls = self.cpStartStopCalls + 1
        self.cpActive = false
    end
    return v
end

local function stateModuleStub(usedHelper, startHelper)
    local sm = { usedHelper = usedHelper, startHelper = startHelper, active = true }
    function sm:getUsedHelper() return self.usedHelper end
    function sm:getStartHelper() return self.startHelper end
    function sm:setStartHelper(state) self.startHelper = state end
    function sm:isActive() return self.active end
    return sm
end

function TestStopCP:setUp()
    TestSetup.reset()
end

function TestStopCP:testVehicleWithoutAutoDriveDataDoesNotRaise()
    local v = cpVehicle()
    local ok, err = pcall(function() AutoDrive:StopCP(v) end)
    lu.assertTrue(ok, 'StopCP must survive a Courseplay vehicle without AutoDrive data: '
        .. tostring(err))
    lu.assertEquals(v.cpStartStopCalls, 1, 'the CP driver must still be stopped')
    lu.assertNil(v.ad, 'StopCP must not conjure an ad table either')
end

function TestStopCP:testClearsRestartFlagWhenTheVehicleHasAutoDriveData()
    local v = cpVehicle()
    v.ad = { restartCP = true }
    AutoDrive:StopCP(v)
    lu.assertEquals(v.ad.restartCP, false, 'the CP course must not be resumed after a stop')
end

function TestStopCP:testDeactivatesTheCpButtonAndClearsTheRestartFlag()
    local v = cpVehicle()
    v.ad = { restartCP = true, stateModule = stateModuleStub(ADStateModule.HELPER_CP, true) }
    AutoDrive:StopCP(v)
    lu.assertEquals(v.ad.stateModule.startHelper, false)
    lu.assertEquals(v.ad.restartCP, false)
end

function TestStopCP:testLeavesTheCpButtonOfAnotherHelperAlone()
    local v = cpVehicle()
    v.ad = { restartCP = true, stateModule = stateModuleStub(ADStateModule.HELPER_AI, true) }
    AutoDrive:StopCP(v)
    lu.assertEquals(v.ad.stateModule.startHelper, true)
    lu.assertEquals(v.ad.restartCP, false)
end

--- Without the CP interface nothing is touched at all - the restart flag included.
function TestStopCP:testWithoutTheCpInterfaceNothingIsTouched()
    local v = cpVehicle()
    v.cpStartStopDriver = nil
    v.ad = { restartCP = true }
    AutoDrive:StopCP(v)
    lu.assertEquals(v.ad.restartCP, true)
end

------------------------------------------------------------------------------------------------------------------------
--- B7 - onCpEmpty / onCpFull must not fabricate an ad table
------------------------------------------------------------------------------------------------------------------------
TestCpFillEvents = {}

function TestCpFillEvents:setUp()
    TestSetup.reset()
end

--- A vehicle carrying AutoDrive data. isActive() answers true so handleCPFieldWorker takes its
--- "AD already active" branch: it clears restartCP on the way, which is what proves it ran.
local function adVehicle()
    local sm = stateModuleStub(ADStateModule.HELPER_CP, true)
    local v = {
        isServer = true,
        startAutoDrive = function() end,
        ad = { restartCP = true, stateModule = sm },
    }
    v.getRootVehicle = function(self) return self end
    return v
end

local function plainVehicle()
    local v = { isServer = true }
    v.getRootVehicle = function(self) return self end
    return v
end

function TestCpFillEvents:testOnCpEmptyLeavesAVehicleWithoutAutoDriveDataUntouched()
    local v = plainVehicle()
    local ok, err = pcall(AutoDrive.onCpEmpty, v)
    lu.assertTrue(ok, tostring(err))
    lu.assertNil(v.ad,
        'a stub ad table makes every later "vehicle.ad ~= nil" check pass while stateModule and '
        .. 'settings are missing - and Courseplay reads self.ad.restartCP off it')
end

function TestCpFillEvents:testOnCpFullLeavesAVehicleWithoutAutoDriveDataUntouched()
    local v = plainVehicle()
    local ok, err = pcall(AutoDrive.onCpFull, v)
    lu.assertTrue(ok, tostring(err))
    lu.assertNil(v.ad)
end

--- The event may arrive on an implement; the root vehicle is the one that would get the stub.
function TestCpFillEvents:testRootVehicleWithoutAutoDriveDataIsNotStubbedEither()
    local root = plainVehicle()
    local implement = { isServer = true, getRootVehicle = function() return root end }
    local ok, err = pcall(AutoDrive.onCpFull, implement)
    lu.assertTrue(ok, tostring(err))
    lu.assertNil(root.ad)
    lu.assertNil(implement.ad)
end

function TestCpFillEvents:testOnCpEmptyRecordsTheFlagAndHandsOver()
    local v = adVehicle()
    AutoDrive.onCpEmpty(v)
    lu.assertEquals(v.ad.isCpEmpty, true)
    lu.assertEquals(v.ad.restartCP, false, 'handleCPFieldWorker must still run for AD vehicles')
end

function TestCpFillEvents:testOnCpFullRecordsTheFlagAndHandsOver()
    local v = adVehicle()
    AutoDrive.onCpFull(v)
    lu.assertEquals(v.ad.isCpFull, true)
    lu.assertEquals(v.ad.restartCP, false)
end

--- A vehicle with no getRootVehicle at all must not raise on the way out.
function TestCpFillEvents:testMissingGetRootVehicleIsSurvivable()
    local ok, err = pcall(AutoDrive.onCpEmpty, { isServer = true })
    lu.assertTrue(ok, tostring(err))
end

os.exit(lu.LuaUnit.run())
