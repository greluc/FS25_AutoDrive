--[[
Choosing where to join the way point network.

Many networks have a collection route running along the inside of the field border, all the way
round to the field exit. A full unloader leaving the field joins that route at the way point closest
to it - and during the first headland pass the harvester is driving on exactly that route, standing
on exactly that way point. The path search then found nothing, the full unloader stayed where it
was, the next unloader could not get past it to the harvester, and the field stopped.

The closest way point is therefore only the first candidate now. These tests pin that the normal
case is unchanged - closest point, one graph search - and that an occupied one is stepped over.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('Dubins')
require('PathFinderModule')

------------------------------------------------------------------------------------------------------------------------
--- Scene: a straight collection route along +x, way points four metres apart as in a recorded one.
------------------------------------------------------------------------------------------------------------------------
local graphSearches

local function buildRoute(count, spacing)
    local wayPoints = {}
    for i = 1, count do
        wayPoints[i] = { id = i, x = (i - 1) * spacing, y = 0, z = 0, out = {}, incoming = {} }
        if i > 1 then
            wayPoints[i].incoming = { i - 1 }
            wayPoints[i - 1].out = { i }
        end
    end
    return wayPoints
end

local function installGraph(wayPoints, reachable)
    ADGraphManager = {
        wayPoints = wayPoints,
        getWayPointById = function(_, id) return wayPoints[id] end,
        forEachWayPointNear = function(_, point, radius, callback)
            for _, wp in pairs(wayPoints) do
                if MathUtil.vector2Length(wp.x - point.x, wp.z - point.z) <= radius then
                    callback(wp)
                end
            end
        end,
        pathFromTo = function(_, fromId, _)
            graphSearches = graphSearches + 1
            if reachable ~= nil and reachable[fromId] == false then
                return nil
            end
            return { wayPoints[fromId], wayPoints[fromId + 1] or wayPoints[fromId] }
        end,
    }
end

--- The module under test, with only the parts selectNetworkEntryPoint touches.
local function pathFinderAt(x, z, closestId)
    local pf = setmetatable({}, { __index = PathFinderModule })
    pf.vehicle = TestSetup.vehicle()
    pf.vehicle.id = 1
    pf.vehicle.components = { { node = 'me' } }
    MockEngine.nodePositions['me'] = { x = x, y = 0, z = z }
    pf.vehicle.getClosestWayPoint = function() return closestId, 0 end
    return pf
end

local function vehicleAt(id, x, z)
    local v = TestSetup.vehicle()
    v.id = id
    v.components = { { node = 'v' .. id } }
    MockEngine.nodePositions['v' .. id] = { x = x, y = 0, z = z }
    return v
end

------------------------------------------------------------------------------------------------------------------------
TestNetworkEntry = {}

function TestNetworkEntry:setUp()
    TestSetup.reset()
    -- luaunit shares the class table between tests, so a scene left over from the previous
    -- one would silently become part of this one
    self.vehicles = nil
    graphSearches = 0
    installGraph(buildRoute(30, 4))
    AutoDrive.checkIsConnected = function() return false end
    -- the vehicle sits next to way point 5, at x = 16
    self.pf = pathFinderAt(16, 8, 5)
    AutoDrive.getAllVehicles = function() return self.vehicles or { self.pf.vehicle } end
end

--- Nothing in the way: the closest point, and exactly one graph search - the same work this did
--- before candidates existed.
function TestNetworkEntry:testTheClosestPointIsUsedWhenItIsFree()
    local id, path = self.pf:selectNetworkEntryPoint(30)

    lu.assertEquals(id, 5)
    lu.assertNotNil(path)
    lu.assertEquals(graphSearches, 1, 'a free closest point must not cost more searches than before')
end

--- The regression: the harvester is standing on way point 5.
function TestNetworkEntry:testAnOccupiedPointIsSteppedOver()
    self.vehicles = { self.pf.vehicle, vehicleAt(2, 16, 0) }

    local id = self.pf:selectNetworkEntryPoint(30)

    lu.assertNotEquals(id, 5, 'joining the route where the harvester stands is what deadlocked the field')
end

--- And it steps over to a neighbour rather than to the far end of the route.
function TestNetworkEntry:testItJoinsNearby()
    self.vehicles = { self.pf.vehicle, vehicleAt(2, 16, 0) }

    local id = self.pf:selectNetworkEntryPoint(30)
    local wp = ADGraphManager:getWayPointById(id)

    lu.assertTrue(MathUtil.vector2Length(wp.x - 16, wp.z - 8) < AutoDrive.NETWORK_ENTRY_SEARCH_RANGE)
end

--- A whole stretch blocked - a harvester with a long header, or a queue of vehicles.
function TestNetworkEntry:testItWalksPastSeveralOccupiedPoints()
    self.vehicles = { self.pf.vehicle,
        vehicleAt(2, 12, 0), vehicleAt(3, 16, 0), vehicleAt(4, 20, 0) }

    local id = self.pf:selectNetworkEntryPoint(30)
    local wp = ADGraphManager:getWayPointById(id)

    lu.assertTrue(math.abs(wp.x - 12) >= AutoDrive.NETWORK_ENTRY_CLEARANCE
        and math.abs(wp.x - 16) >= AutoDrive.NETWORK_ENTRY_CLEARANCE
        and math.abs(wp.x - 20) >= AutoDrive.NETWORK_ENTRY_CLEARANCE,
        'the chosen point must be clear of every vehicle, not just the nearest')
end

--- Our own trailer is parked right beside us by definition; treating it as a blocker would push
--- every departure to a needlessly distant entry point.
function TestNetworkEntry:testOurOwnTrainDoesNotOccupyAnything()
    self.vehicles = { self.pf.vehicle, vehicleAt(2, 16, 0) }
    AutoDrive.checkIsConnected = function() return true end

    lu.assertEquals(self.pf:selectNetworkEntryPoint(30), 5)
end

--- A point the network cannot route to the destination from is no use, however free it is.
function TestNetworkEntry:testAnUnreachablePointIsSkipped()
    installGraph(buildRoute(30, 4), { [5] = false })

    local id = self.pf:selectNetworkEntryPoint(30)

    lu.assertNotEquals(id, 5)
end

--- Everything blocked: fall back to the closest point rather than refuse to drive. It may not work,
--- but it is what this did before, and standing still forever is the worse failure.
function TestNetworkEntry:testItFallsBackToTheClosestPoint()
    local blockers = { self.pf.vehicle }
    for i = 1, 30 do
        blockers[#blockers + 1] = vehicleAt(100 + i, (i - 1) * 4, 0)
    end
    self.vehicles = blockers

    local id, path = self.pf:selectNetworkEntryPoint(30)

    lu.assertEquals(id, 5)
    lu.assertNil(path, 'the fallback has no verified path, so the caller has to search for one')
end

--- Each candidate costs a graph search, so the walk has to stop somewhere.
function TestNetworkEntry:testTheSearchIsBounded()
    installGraph(buildRoute(30, 4), (function()
        local none = {}
        for i = 1, 30 do none[i] = false end
        return none
    end)())

    self.pf:selectNetworkEntryPoint(30)

    lu.assertTrue(graphSearches <= AutoDrive.NETWORK_ENTRY_MAX_TRIES,
        'an unreachable destination must not search the graph once per way point in range')
end

--- A way point with no way in is an exit, not somewhere to join.
function TestNetworkEntry:testExitOnlyPointsAreNotCandidates()
    local route = buildRoute(30, 4)
    route[5].incoming = {}
    installGraph(route)

    lu.assertNotEquals(self.pf:selectNetworkEntryPoint(30), 5)
end

os.exit(lu.LuaUnit.run())
