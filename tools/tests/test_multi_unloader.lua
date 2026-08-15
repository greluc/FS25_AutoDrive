--[[
Keeping several unloaders on one field out of each other's way.

Three mechanisms, from the cheapest to the most deliberate:

  1. Route reservation - the pathfinder refuses cells another AutoDrive vehicle's route runs
     through. It used to exempt the last five way points of every foreign route, i.e. its
     destination, which is exactly where two unloaders converging on one harvester overlap.
  2. Off-network look-ahead - the existing route look-ahead only works on the road network,
     because it identifies segments by way point id. On a field it was simply off.
  3. Approach claims - a side of a harvester is reserved by the unloader that asks first, so the
     second one picks differently instead of driving at the same spot.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('PathFinderUtils')
require('CollisionDetectionUtils')
require('AutoDriveTON')
require('HarvestManager')
require('CollisionDetectionModule')

------------------------------------------------------------------------------------------------------------------------
--- 3. Approach claims
------------------------------------------------------------------------------------------------------------------------
TestApproachClaims = {}

function TestApproachClaims:setUp()
    TestSetup.reset()
    ADHarvestManager:load()
    g_time = 10000
    self.harvester = TestSetup.vehicle()
    self.first = TestSetup.vehicle()
    self.second = TestSetup.vehicle()
end

function TestApproachClaims:testFirstAskerGetsTheSide()
    lu.assertTrue(ADHarvestManager:claimApproach(self.first, self.harvester, AutoDrive.CHASEPOS_LEFT))
end

function TestApproachClaims:testSecondAskerIsRefused()
    ADHarvestManager:claimApproach(self.first, self.harvester, AutoDrive.CHASEPOS_LEFT)
    lu.assertFalse(ADHarvestManager:claimApproach(self.second, self.harvester, AutoDrive.CHASEPOS_LEFT),
        'two unloaders must not target the same spot next to a harvester')
    lu.assertTrue(ADHarvestManager:isApproachClaimedByOther(self.second, self.harvester, AutoDrive.CHASEPOS_LEFT))
end

function TestApproachClaims:testTheOtherSideIsStillFree()
    ADHarvestManager:claimApproach(self.first, self.harvester, AutoDrive.CHASEPOS_LEFT)
    lu.assertTrue(ADHarvestManager:claimApproach(self.second, self.harvester, AutoDrive.CHASEPOS_RIGHT),
        'reserving one side must not block the others')
end

function TestApproachClaims:testDifferentHarvestersAreIndependent()
    local otherHarvester = TestSetup.vehicle()
    ADHarvestManager:claimApproach(self.first, self.harvester, AutoDrive.CHASEPOS_LEFT)
    lu.assertTrue(ADHarvestManager:claimApproach(self.second, otherHarvester, AutoDrive.CHASEPOS_LEFT))
end

function TestApproachClaims:testTheHolderKeepsItByAsking()
    ADHarvestManager:claimApproach(self.first, self.harvester, AutoDrive.CHASEPOS_LEFT)
    g_time = g_time + 3000
    lu.assertTrue(ADHarvestManager:claimApproach(self.first, self.harvester, AutoDrive.CHASEPOS_LEFT),
        're-asking refreshes the claim, which is how the chase loop holds it')
    g_time = g_time + 3000
    lu.assertTrue(ADHarvestManager:isApproachClaimedByOther(self.second, self.harvester, AutoDrive.CHASEPOS_LEFT),
        'a refreshed claim must not expire under the holder')
end

--- The safety net: an unloader that stops chasing - finished, stuck, deleted - stops refreshing,
--- and the side has to become free on its own without anything noticing.
function TestApproachClaims:testAnAbandonedClaimExpires()
    ADHarvestManager:claimApproach(self.first, self.harvester, AutoDrive.CHASEPOS_LEFT)
    g_time = g_time + ADHarvestManager.CLAIM_VALID_TIME + 1
    lu.assertFalse(ADHarvestManager:isApproachClaimedByOther(self.second, self.harvester, AutoDrive.CHASEPOS_LEFT),
        'a claim nobody refreshes must not block the side forever')
    lu.assertTrue(ADHarvestManager:claimApproach(self.second, self.harvester, AutoDrive.CHASEPOS_LEFT))
end

function TestApproachClaims:testReleasingFreesEverySideOfThatUnloader()
    ADHarvestManager:claimApproach(self.first, self.harvester, AutoDrive.CHASEPOS_LEFT)
    ADHarvestManager:claimApproach(self.first, self.harvester, AutoDrive.CHASEPOS_REAR)

    ADHarvestManager:releaseApproachClaims(self.first)

    lu.assertTrue(ADHarvestManager:claimApproach(self.second, self.harvester, AutoDrive.CHASEPOS_LEFT))
    lu.assertTrue(ADHarvestManager:claimApproach(self.second, self.harvester, AutoDrive.CHASEPOS_REAR))
end

function TestApproachClaims:testReleasingLeavesOtherUnloadersAlone()
    ADHarvestManager:claimApproach(self.first, self.harvester, AutoDrive.CHASEPOS_LEFT)
    ADHarvestManager:claimApproach(self.second, self.harvester, AutoDrive.CHASEPOS_RIGHT)

    ADHarvestManager:releaseApproachClaims(self.first)

    lu.assertTrue(ADHarvestManager:isApproachClaimedByOther(self.first, self.harvester, AutoDrive.CHASEPOS_RIGHT),
        "releasing one unloader must not free the side another one holds")
end

function TestApproachClaims:testNilArgumentsAreRefused()
    lu.assertFalse(ADHarvestManager:claimApproach(nil, self.harvester, AutoDrive.CHASEPOS_LEFT))
    lu.assertFalse(ADHarvestManager:claimApproach(self.first, nil, AutoDrive.CHASEPOS_LEFT))
    lu.assertFalse(ADHarvestManager:claimApproach(self.first, self.harvester, nil))
end

------------------------------------------------------------------------------------------------------------------------
--- 2. Off-network look-ahead
------------------------------------------------------------------------------------------------------------------------
TestOffRouteLookAhead = {}

local function unloaderOnPath(id, x, z, points, onNetwork)
    local v = TestSetup.vehicle()
    v.id = id
    v.components = { { node = 'v' .. id } }
    MockEngine.nodePositions['v' .. id] = { x = x, y = 0, z = z }
    v.ad.stateModule = { isActive = function() return true end }
    v.ad.drivePathModule = {
        getWayPoints = function() return points, 1 end,
        isOnRoadNetwork = function() return onNetwork == true end,
    }
    return v
end

--- The scan is throttled to one frame in PERF_FRAMES, phase-shifted by vehicle id so the vehicles
--- do not all scan on the same frame. A test that does not put the loop index on that vehicle's
--- frame gets the cached answer instead of running the scan - and every assertFalse then passes for
--- the wrong reason, which is how the first version of these tests fooled itself.
local function detectsTrafficFor(vehicle)
    g_updateLoopIndex = (AutoDrive.PERF_FRAMES - vehicle.id) % AutoDrive.PERF_FRAMES
    local o = setmetatable({}, { __index = ADCollisionDetectionModule })
    o.vehicle = vehicle
    return o:detectAdTrafficOffRoute(), o
end

function TestOffRouteLookAhead:setUp()
    TestSetup.reset()
    AutoDrive.getAllVehicles = function() return self.vehicles or {} end
    AutoDrive.checkIsConnected = function() return false end
end

function TestOffRouteLookAhead:testPathPointsAreLimitedToTheLookAhead()
    local far = {}
    for i = 1, 40 do far[i] = { x = i * 5, y = 0, z = 0 } end
    local v = unloaderOnPath(1, 0, 0, far, false)
    local o = setmetatable({ vehicle = v }, { __index = ADCollisionDetectionModule })
    local points, distances = o:getUpcomingPathPoints(v)
    lu.assertTrue(#points > 0)
    lu.assertTrue(#points < 40, 'the look-ahead must not walk the whole path')
    lu.assertTrue(distances[#distances] <= AutoDrive.AD_TRAFFIC_LOOKAHEAD)
end

--- Two unloaders converging on one point: exactly one yields, and it is the one further away.
--- Both must reach the meeting point within the look-ahead or neither sees the conflict.
local function convergingPair()
    local meeting = { x = 50, y = 0, z = 0 }
    -- near stands 8 m before the meeting point, far 20 m - both inside the 25 m look-ahead
    local near = unloaderOnPath(1, 42, 0, { { x = 46, y = 0, z = 0 }, meeting }, false)
    local far  = unloaderOnPath(2, 30, 0, { { x = 40, y = 0, z = 0 }, meeting }, false)
    return near, far
end

function TestOffRouteLookAhead:testTheFurtherVehicleYields()
    local near, far = convergingPair()
    self.vehicles = { near, far }

    lu.assertTrue(detectsTrafficFor(far),
        'the vehicle further from the conflict point must give way')
end

--- The other half of the rule, and the reason it is distance-based rather than, say, id-based:
--- if both sides yielded the field would deadlock with two stopped unloaders.
function TestOffRouteLookAhead:testTheCloserVehicleKeepsGoing()
    local near, far = convergingPair()
    self.vehicles = { near, far }

    lu.assertFalse(detectsTrafficFor(near),
        'only one of the two may stop, otherwise nothing moves again')
end

function TestOffRouteLookAhead:testPathsThatDoNotMeetAreIgnored()
    local a = unloaderOnPath(1, 0, 0, { { x = 5, y = 0, z = 0 }, { x = 10, y = 0, z = 0 } }, false)
    local b = unloaderOnPath(2, 0, 500, { { x = 5, y = 0, z = 500 }, { x = 10, y = 0, z = 500 } }, false)
    self.vehicles = { a, b }
    lu.assertFalse(detectsTrafficFor(a))
end

--- On the road network detectAdTrafficOnRoute owns this; running both would apply two rules at once.
function TestOffRouteLookAhead:testOnNetworkIsLeftToTheExistingCheck()
    local near, far = convergingPair()
    far.ad.drivePathModule.isOnRoadNetwork = function() return true end
    self.vehicles = { near, far }
    lu.assertFalse(detectsTrafficFor(far))
end

function TestOffRouteLookAhead:testAnInactiveDriverIsNotTraffic()
    local near, far = convergingPair()
    near.ad.stateModule.isActive = function() return false end
    self.vehicles = { near, far }
    lu.assertFalse(detectsTrafficFor(far))
end

--- A trailer of our own train travels our own path; yielding to it would stop us permanently.
function TestOffRouteLookAhead:testOwnTrainIsIgnored()
    local near, far = convergingPair()
    self.vehicles = { near, far }
    AutoDrive.checkIsConnected = function() return true end
    lu.assertFalse(detectsTrafficFor(far))
end

--- Between its scan frames the module answers from the cached result rather than rescanning.
function TestOffRouteLookAhead:testTheAnswerIsHeldBetweenScanFrames()
    local near, far = convergingPair()
    self.vehicles = { near, far }

    local yields, module = detectsTrafficFor(far)
    lu.assertTrue(yields)

    g_updateLoopIndex = g_updateLoopIndex + 1
    lu.assertTrue(module:detectAdTrafficOffRoute(),
        'an off-frame call must keep the last answer, not flicker back to clear')
end

------------------------------------------------------------------------------------------------------------------------
--- 1. Route reservation at the far end of a foreign route
------------------------------------------------------------------------------------------------------------------------
TestRouteReservation = {}

--- A pathfinder route of n points along +x, one metre apart, ending at (endX, endZ).
local function pathFinderRoute(n, endX, endZ)
    local wps = {}
    for i = 1, n do
        wps[i] = { x = endX - (n - i), y = 0, z = endZ, isPathFinderPoint = true }
    end
    return wps
end

local function routedVehicle(id, route)
    local v = TestSetup.vehicle()
    v.id = id
    v.rootNode = 'root' .. id
    v.size = { width = 3, length = 6, lengthOffset = 0 }
    v.components = { { node = 'root' .. id } }
    v.ad.stateModule = { isActive = function() return true end }
    v.ad.drivePathModule = { getWayPoints = function() return route, 1 end }
    return v
end

--- A box around the second to last point of the other route - its destination approach, the stretch
--- the blanket exemption used to leave open.
local function boxAtEndOf(route)
    local wp = route[#route - 1]
    return AutoDrive.boundingBoxFromCorners(wp.x - 2, wp.z - 2, wp.x + 2, wp.z - 2,
                                            wp.x + 2, wp.z + 2, wp.x - 2, wp.z + 2)
end

function TestRouteReservation:setUp()
    TestSetup.reset()
end

--- The regression this was written for: two unloaders driving to unrelated places, and the second
--- one plans straight through where the first is about to stop.
function TestRouteReservation:testAForeignDestinationIsProtected()
    local other = routedVehicle(2, pathFinderRoute(12, 100, 0))
    local mine = routedVehicle(1, pathFinderRoute(12, 400, 400))
    AutoDrive.getAllVehicles = function() return { mine, other } end

    lu.assertTrue(AutoDrive.checkForVehiclePathInBox(boxAtEndOf(other.ad.drivePathModule:getWayPoints()), 5, mine),
        "the destination of another vehicle must be reserved like the rest of its route")
end

--- But when both are headed for the same place - two unloaders on one harvester - protecting it
--- would leave neither able to approach. There the exemption stays, and the ordering is
--- CombineUnloaderMode's job instead.
function TestRouteReservation:testASharedDestinationStaysOpen()
    local other = routedVehicle(2, pathFinderRoute(12, 100, 0))
    local mine = routedVehicle(1, pathFinderRoute(12, 105, 0))
    AutoDrive.getAllVehicles = function() return { mine, other } end

    lu.assertFalse(AutoDrive.checkForVehiclePathInBox(boxAtEndOf(other.ad.drivePathModule:getWayPoints()), 5, mine),
        'two vehicles heading for the same spot must not reserve it away from each other')
end

--- The boundary is SHARED_TARGET_RANGE, so a target just outside it counts as unrelated.
function TestRouteReservation:testTheSharedTargetRangeIsTheBoundary()
    local other = routedVehicle(2, pathFinderRoute(12, 100, 0))
    local mine = routedVehicle(1, pathFinderRoute(12, 100, AutoDrive.SHARED_TARGET_RANGE + 5))
    AutoDrive.getAllVehicles = function() return { mine, other } end

    lu.assertTrue(AutoDrive.checkForVehiclePathInBox(boxAtEndOf(other.ad.drivePathModule:getWayPoints()), 5, mine))
end

--- The middle of a foreign route was always protected and has to stay that way.
function TestRouteReservation:testTheMiddleOfAForeignRouteIsProtectedEitherWay()
    local other = routedVehicle(2, pathFinderRoute(12, 100, 0))
    local mine = routedVehicle(1, pathFinderRoute(12, 105, 0))
    AutoDrive.getAllVehicles = function() return { mine, other } end

    local wp = other.ad.drivePathModule:getWayPoints()[6]
    local box = AutoDrive.boundingBoxFromCorners(wp.x - 2, wp.z - 2, wp.x + 2, wp.z - 2,
                                                 wp.x + 2, wp.z + 2, wp.x - 2, wp.z + 2)
    lu.assertTrue(AutoDrive.checkForVehiclePathInBox(box, 5, mine),
        'a shared destination must only open the tail, not the whole route')
end

--- A vehicle with no route of its own cannot compare targets; the safe reading of "unknown" is to
--- protect, since that is the behaviour for every unrelated vehicle.
function TestRouteReservation:testAVehicleWithoutARouteProtects()
    local other = routedVehicle(2, pathFinderRoute(12, 100, 0))
    local mine = routedVehicle(1, nil)
    mine.ad.drivePathModule = { getWayPoints = function() return nil, nil end }
    AutoDrive.getAllVehicles = function() return { mine, other } end

    lu.assertTrue(AutoDrive.checkForVehiclePathInBox(boxAtEndOf(other.ad.drivePathModule:getWayPoints()), 5, mine))
end

function TestRouteReservation:testAnInactiveVehicleReservesNothing()
    local other = routedVehicle(2, pathFinderRoute(12, 100, 0))
    other.ad.stateModule.isActive = function() return false end
    local mine = routedVehicle(1, pathFinderRoute(12, 400, 400))
    AutoDrive.getAllVehicles = function() return { mine, other } end

    lu.assertFalse(AutoDrive.checkForVehiclePathInBox(boxAtEndOf(other.ad.drivePathModule:getWayPoints()), 5, mine))
end

os.exit(lu.LuaUnit.run())
