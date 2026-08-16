--[[
Tests for scripts/Manager/GraphManager.lua.

Covers findings:
  A21 FastShortestPath ignored its markerId argument and resolved by marker NAME
  P2  getWayPointsInRange was a full linear scan over every waypoint
  P8  every single deletion renumbered the whole graph
  P1  the spatial index those two now share

The measured networks this is calibrated against: 20.310 to 55.595 waypoints, of which under
1 % lie within 200 m of any given point. The index has to reproduce a linear scan exactly - a
faster query that returns a different set is worse than a slow one.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('SortedQueue')
require('GraphManager')
require('PathCalculation')

local function buildGrid(nx, nz, spacing)
    -- A regular grid of waypoints, each linked to its right neighbour.
    local wps = {}
    local id = 0
    for ix = 0, nx - 1 do
        for iz = 0, nz - 1 do
            id = id + 1
            wps[id] = TestSetup.waypoint(id, ix * spacing, iz * spacing, {}, {})
        end
    end
    return wps
end

--- Reference implementation: the linear scan the index replaces.
local function linearInRange(wps, point, rangeMin, rangeMax)
    local out = {}
    for _, wp in pairs(wps) do
        local d = math.sqrt((wp.x - point.x) ^ 2 + (wp.z - point.z) ^ 2)
        if d < rangeMax and d > rangeMin then
            table.insert(out, wp.id)
        end
    end
    table.sort(out)
    return out
end

------------------------------------------------------------------------------------------------------------------------
--- P1 / P2 - the spatial index
------------------------------------------------------------------------------------------------------------------------
TestSpatialIndex = {}

function TestSpatialIndex:setUp()
    TestSetup.reset()
    ADGraphManager:load()
end

function TestSpatialIndex:testRangeQueryMatchesLinearScanExactly()
    local wps = buildGrid(30, 30, 7)   -- 900 waypoints, 7 m apart
    ADGraphManager:setWayPoints(wps)

    local probes = {
        { x = 0, z = 0 }, { x = 50, z = 50 }, { x = 101.5, z = 3.25 },
        { x = -40, z = 120 }, { x = 203, z = 203 }, { x = 99, z = 1 },
    }
    for _, p in ipairs(probes) do
        for _, range in ipairs({ { 0, 10 }, { 1, 20 }, { 0, 50 }, { 5, 200 }, { 0, 1000 } }) do
            local got = ADGraphManager:getWayPointsInRange(p, range[1], range[2])
            table.sort(got)
            local want = linearInRange(wps, p, range[1], range[2])
            lu.assertEquals(got, want,
                string.format('range query differs from linear scan at (%.1f,%.1f) min=%d max=%d',
                    p.x, p.z, range[1], range[2]))
        end
    end
end

function TestSpatialIndex:testEmptyNetworkReturnsEmpty()
    ADGraphManager:setWayPoints({})
    lu.assertEquals(ADGraphManager:getWayPointsInRange({ x = 0, z = 0 }, 0, 100), {})
end

function TestSpatialIndex:testIndexFollowsWaypointCreation()
    ADGraphManager:setWayPoints({})
    lu.assertEquals(#ADGraphManager:getWayPointsInRange({ x = 10, z = 10 }, 0, 5), 0)

    ADGraphManager:setWayPoints({ TestSetup.waypoint(1, 10, 10, {}, {}) })
    lu.assertEquals(ADGraphManager:getWayPointsInRange({ x = 10, z = 10 }, -1, 5), { 1 })
end

--- The index must not survive a move that takes a waypoint out of its cell.
function TestSpatialIndex:testIndexFollowsWaypointMove()
    local wps = { TestSetup.waypoint(1, 0, 0, {}, {}) }
    ADGraphManager:setWayPoints(wps)
    lu.assertEquals(ADGraphManager:getWayPointsInRange({ x = 0, z = 0 }, -1, 5), { 1 })

    ADGraphManager:moveWayPoint(1, 500, 0, 500, 0, false)
    lu.assertEquals(ADGraphManager:getWayPointsInRange({ x = 0, z = 0 }, -1, 5), {})
    lu.assertEquals(ADGraphManager:getWayPointsInRange({ x = 500, z = 500 }, -1, 5), { 1 })
end

--- The whole point of the index: a query must not touch the whole network.
function TestSpatialIndex:testQueryDoesNotScanEveryWaypoint()
    local wps = buildGrid(40, 40, 5)   -- 1600 waypoints spanning ~200 m
    ADGraphManager:setWayPoints(wps)

    lu.assertNotNil(ADGraphManager.getRangeQueryVisitCount,
        'the index must expose how many waypoints a query inspected, so this cannot silently '
        .. 'regress to a full scan')

    ADGraphManager:getWayPointsInRange({ x = 100, z = 100 }, 0, 10)
    local visited = ADGraphManager:getRangeQueryVisitCount()
    lu.assertTrue(visited < 1600 * 0.25,
        string.format('a 10 m query inspected %d of 1600 waypoints - the index is not being used',
            visited))
end

------------------------------------------------------------------------------------------------------------------------
--- A21 - path target resolved by id, not by name
------------------------------------------------------------------------------------------------------------------------
TestFastShortestPath = {}

function TestFastShortestPath:setUp()
    TestSetup.reset()
    ADGraphManager:load()
    -- ADPathCalculator:getDetourWeights reads this; without it the A* comparison hits nil.
    AutoDrive.testSettings['mapMarkerDetour'] = 0

    -- A straight chain 1..6 so a path exists between any two points.
    local wps = {}
    for i = 1, 6 do
        wps[i] = TestSetup.waypoint(i, i * 10, 0,
            i < 6 and { i + 1 } or {},
            i > 1 and { i - 1 } or {})
    end
    ADGraphManager:setWayPoints(wps)

    -- Two markers sharing a name, pointing at DIFFERENT waypoints. This is not a contrived case:
    -- savegame4 has 167 duplicate names among 565 markers.
    ADGraphManager:setMapMarkers({
        [1] = { id = 2, name = 'Silo', group = 'All' },
        [2] = { id = 6, name = 'Silo', group = 'All' },
    })
end

function TestFastShortestPath:testResolvesTheRequestedMarkerNotTheFirstWithThatName()
    local path = ADGraphManager:FastShortestPath(1, 'Silo', 2)
    lu.assertNotNil(path)
    lu.assertTrue(#path > 0, 'expected a path to marker 2')
    lu.assertEquals(path[#path].id, 6,
        'FastShortestPath must honour the markerId it is given; resolving by name returns the '
        .. 'first marker called "Silo" (waypoint 2) instead of the requested one (waypoint 6)')
end

function TestFastShortestPath:testStillWorksForUniqueNames()
    ADGraphManager:setMapMarkers({ [1] = { id = 4, name = 'Hof', group = 'All' } })
    local path = ADGraphManager:FastShortestPath(1, 'Hof', 1)
    lu.assertTrue(#path > 0)
    lu.assertEquals(path[#path].id, 4)
end

function TestFastShortestPath:testUnknownMarkerReturnsEmptyPath()
    lu.assertEquals(ADGraphManager:FastShortestPath(1, 'Nope', 99), {})
end

function TestFastShortestPath:testSameStartAndTargetReturnsSinglePoint()
    local path = ADGraphManager:FastShortestPath(2, 'Silo', 1)
    lu.assertEquals(#path, 1)
    lu.assertEquals(path[1].id, 2)
end

------------------------------------------------------------------------------------------------------------------------
--- P8 - deletion renumbers once, not once per deleted point
------------------------------------------------------------------------------------------------------------------------
TestBatchDelete = {}

function TestBatchDelete:setUp()
    TestSetup.reset()
    ADGraphManager:load()
end

function TestBatchDelete:testDeletingSeveralPointsKeepsTheGraphConsistent()
    local wps = {}
    for i = 1, 10 do
        wps[i] = TestSetup.waypoint(i, i, 0,
            i < 10 and { i + 1 } or {},
            i > 1 and { i - 1 } or {})
    end
    ADGraphManager:setWayPoints(wps)

    ADGraphManager:removeWayPoints({ 8, 5, 3 }, false)

    local remaining = ADGraphManager:getWayPoints()
    lu.assertEquals(#remaining, 7)

    -- ids must be consecutive again, and every connection must point at a real waypoint
    for i, wp in ipairs(remaining) do
        lu.assertEquals(wp.id, i, 'ids must be renumbered consecutively')
        for _, outId in pairs(wp.out) do
            lu.assertNotNil(remaining[outId],
                string.format('waypoint %d has a dangling out connection to %d', i, outId))
        end
        for _, inId in pairs(wp.incoming) do
            lu.assertNotNil(remaining[inId],
                string.format('waypoint %d has a dangling incoming connection to %d', i, inId))
        end
    end
end

function TestBatchDelete:testBatchDeleteRenumbersOnlyOnce()
    local wps = {}
    for i = 1, 40 do
        wps[i] = TestSetup.waypoint(i, i, 0, {}, {})
    end
    ADGraphManager:setWayPoints(wps)

    lu.assertNotNil(ADGraphManager.getRenumberCount,
        'batch deletion must expose how often it renumbered, otherwise a regression to '
        .. 'per-point renumbering is invisible')

    ADGraphManager:resetRenumberCount()
    ADGraphManager:removeWayPoints({ 30, 20, 10, 5 }, false)
    lu.assertEquals(ADGraphManager:getRenumberCount(), 1,
        'four deleted points must cost one renumbering pass, not four')
    lu.assertEquals(#ADGraphManager:getWayPoints(), 36)
end

function TestBatchDelete:testSinglePointDeletionStillWorks()
    local wps = {}
    for i = 1, 5 do
        wps[i] = TestSetup.waypoint(i, i, 0, {}, {})
    end
    ADGraphManager:setWayPoints(wps)
    ADGraphManager:removeWayPoint(3, false)
    lu.assertEquals(#ADGraphManager:getWayPoints(), 4)
end

------------------------------------------------------------------------------------------------------------------------
--- Checking the road network for faults
---
--- The duplicate-point check groups way points into ten metre tiles and compares the ones that share
--- a tile. It walked that group with "for _, j in tileHashMap[hash] do" - no ipairs - which asks Lua
--- to CALL the table as an iterator and raises on the spot. Two way points in one tile is all it
--- takes, so an ordinary route recorded at five metre spacing killed the check on its very first
--- tile, and the network fault finder could never report anything to anybody.
------------------------------------------------------------------------------------------------------------------------
TestNetworkCheck = {}

function TestNetworkCheck:setUp()
    TestSetup.reset()
    ADGraphManager:load()
    AutoDrive.testSettings['mapMarkerDetour'] = 0
    AutoDrive.Hud = { lastUIScale = 1 }
    AutoDrive.getAllVehicles = function() return {} end
    AutoDrive.notifyDestinationListeners = function() end
    -- Without this the entire body of createDebugMarkers is skipped and both tests below pass
    -- without executing a line of what they are about.
    AutoDrive.currentDebugChannelMask = AutoDrive.DC_ROADNETWORKINFO
    lu.assertTrue(AutoDrive.getDebugChannelIsSet(AutoDrive.DC_ROADNETWORKINFO))
end

--- Points five metres apart on a straight line - the most ordinary recorded route there is, and
--- several of them share a ten metre tile.
function TestNetworkCheck:testItSurvivesAnOrdinaryRoute()
    local wps = {}
    for i = 1, 8 do
        wps[i] = TestSetup.waypoint(i, i * 5, 0, i < 8 and { i + 1 } or {}, i > 1 and { i - 1 } or {})
    end
    ADGraphManager:setWayPoints(wps)

    local ok, err = pcall(function() ADGraphManager:createDebugMarkers(false) end)

    lu.assertTrue(ok, 'checking a perfectly healthy route raised: ' .. tostring(err))
end

--- And it still finds what it is for: two way points on exactly the same spot.
function TestNetworkCheck:testItStillFindsGenuineDuplicates()
    local wps = {}
    for i = 1, 4 do
        wps[i] = TestSetup.waypoint(i, i * 5, 0, i < 4 and { i + 1 } or {}, i > 1 and { i - 1 } or {})
    end
    wps[5] = TestSetup.waypoint(5, 2 * 5, 0, {}, {})   -- sitting on top of way point 2
    ADGraphManager:setWayPoints(wps)

    local ok = pcall(function() ADGraphManager:createDebugMarkers(false) end)

    lu.assertTrue(ok)
    lu.assertTrue(wps[5].foundError == true or wps[2].foundError == true,
        'a way point placed exactly on another one has to be reported as a fault')
end


------------------------------------------------------------------------------------------------------------------------
--- Deleting a way point has to take every destination standing on it
---
--- removeMapMarkerByWayPoint removed the FIRST marker whose id matched and broke out of the loop.
--- Nothing prevents two markers from sharing a way point: creating a destination does not check
--- whether the closest one already carries one, way points sit three to five metres apart, so two
--- names entered from the same spot land on the same point - and the road network debug channel
--- adds its own markers on top of existing ones as well.
---
--- The survivor did not merely linger. The renumbering that follows a deletion only rewrites markers
--- whose old id it knows about, and a marker on a DELETED point is not one of those, so its raw id
--- was left alone and, once the graph was compacted, pointed at whatever way point had moved into
--- that index. The destination did not disappear - it walked quietly down the route, and drivers
--- sent there stopped somewhere else entirely.
------------------------------------------------------------------------------------------------------------------------
TestMarkerRemoval = {}

function TestMarkerRemoval:setUp()
    TestSetup.reset()
    ADGraphManager:load()
    AutoDrive.testSettings['mapMarkerDetour'] = 0
    AutoDrive.Hud = { lastUIScale = 1 }
    AutoDrive.getAllVehicles = function() return {} end
    AutoDrive.notifyDestinationListeners = function() end

    local wps = {}
    for i = 1, 10 do
        wps[i] = TestSetup.waypoint(i, i * 10, 0,
            i < 10 and { i + 1 } or {}, i > 1 and { i - 1 } or {})
    end
    ADGraphManager:setWayPoints(wps)
end

local function markerNamed(name)
    for _, marker in pairs(ADGraphManager:getMapMarkers()) do
        if marker.name == name then return marker end
    end
    return nil
end

local function markerX(name)
    local marker = markerNamed(name)
    if marker == nil then return nil end
    local wp = ADGraphManager:getWayPointById(marker.id)
    return wp ~= nil and wp.x or nil
end

function TestMarkerRemoval:testBothDestinationsOnAPointGoWithIt()
    ADGraphManager:createMapMarker(2, 'Hof', false)
    ADGraphManager:createMapMarker(5, 'Silo', false)
    ADGraphManager:createMapMarker(5, 'SiloTip', false)
    ADGraphManager:createMapMarker(9, 'Feld', false)
    lu.assertEquals(#ADGraphManager:getMapMarkers(), 4, 'test setup')

    ADGraphManager:removeWayPoints({ 5 }, false)

    lu.assertNil(markerNamed('Silo'), 'the destination on the deleted point has to go')
    lu.assertNil(markerNamed('SiloTip'),
        'and so does the second one on that same point - it cannot stay behind aiming at a '
        .. 'way point that has moved underneath it')
end

--- And the destinations that were NOT on the deleted point keep pointing where they did.
function TestMarkerRemoval:testTheOtherDestinationsStayWhereTheyWere()
    ADGraphManager:createMapMarker(2, 'Hof', false)
    ADGraphManager:createMapMarker(5, 'Silo', false)
    ADGraphManager:createMapMarker(5, 'SiloTip', false)
    ADGraphManager:createMapMarker(9, 'Feld', false)

    ADGraphManager:removeWayPoints({ 5 }, false)

    lu.assertEquals(markerX('Hof'), 20)
    lu.assertEquals(markerX('Feld'), 90)
end

os.exit(lu.LuaUnit.run())
