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

--- The claim map is keyed on the harvester table and then on the side. An entry that empties has to
--- go, or the map grows one entry per harvester ever worked for and holds each of those vehicles
--- alive through the key.
function TestApproachClaims:testAnEmptiedHarvesterEntryIsDropped()
    ADHarvestManager:claimApproach(self.first, self.harvester, AutoDrive.CHASEPOS_LEFT)
    lu.assertNotNil(ADHarvestManager.approachClaims[self.harvester], 'test setup: it has to be there first')

    ADHarvestManager:releaseApproachClaims(self.first)

    lu.assertNil(ADHarvestManager.approachClaims[self.harvester],
        'an empty side table keeps its harvester from ever being collected')
end

--- But an entry another unloader still holds must survive the release.
function TestApproachClaims:testAnEntryStillInUseSurvives()
    ADHarvestManager:claimApproach(self.first, self.harvester, AutoDrive.CHASEPOS_LEFT)
    ADHarvestManager:claimApproach(self.second, self.harvester, AutoDrive.CHASEPOS_RIGHT)

    ADHarvestManager:releaseApproachClaims(self.first)

    lu.assertNotNil(ADHarvestManager.approachClaims[self.harvester])
    lu.assertTrue(ADHarvestManager:isApproachClaimedByOther(self.first, self.harvester, AutoDrive.CHASEPOS_RIGHT))
end

--- Asking about a harvester nobody has ever claimed must not create an entry for it.
function TestApproachClaims:testAskingDoesNotCreateAnEntry()
    local stranger = TestSetup.vehicle()

    ADHarvestManager:isApproachClaimedByOther(self.first, stranger, AutoDrive.CHASEPOS_LEFT)

    lu.assertNil(ADHarvestManager.approachClaims[stranger],
        'a read must not populate the map, or every harvester ever asked about accumulates')
end

--- Releasing for an unloader that holds nothing at all is a no-op, not an error.
function TestApproachClaims:testReleasingWithoutAnyClaimIsHarmless()
    ADHarvestManager:releaseApproachClaims(self.second)
    lu.assertTrue(ADHarvestManager:claimApproach(self.first, self.harvester, AutoDrive.CHASEPOS_LEFT))
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
    -- moving, because giving way to somebody who is standing still is not a thing any more
    v.lastSpeedReal = 0.002
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
    g_time = 10000
    -- luaunit shares the class table between tests, so a scene left over from the previous
    -- one would silently become part of this one
    self.vehicles = nil
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

--- Giving way means letting the other one through first, which only helps if it is going
--- anywhere. A vehicle that is standing still will not clear the point we both want, so waiting for
--- it would turn one stopped vehicle into two - it gets asked to move out of the way instead.
function TestOffRouteLookAhead:testWeDoNotGiveWayToAStandingVehicle()
    local near, far = convergingPair()
    near.lastSpeedReal = 0
    self.vehicles = { near, far }

    lu.assertFalse(detectsTrafficFor(far),
        'waiting for a vehicle that is not moving would leave both of them standing')
end

--- The rule itself cannot deadlock: of two vehicles heading for one point only the further one
--- yields. But that says nothing about a vehicle stuck on something else - a gate, a tree, a parked
--- player - so the wait is capped and we drive on afterwards.
function TestOffRouteLookAhead:testTheYieldEndsAfterTheTimeout()
    local near, far = convergingPair()
    self.vehicles = { near, far }

    local yields, module = detectsTrafficFor(far)
    lu.assertTrue(yields)

    g_time = g_time + AutoDrive.AD_TRAFFIC_YIELD_TIMEOUT + 1
    lu.assertFalse(module:detectAdTrafficOffRoute(),
        'giving way has to end, or a vehicle stuck on something else blocks us indefinitely')
end

function TestOffRouteLookAhead:testTheYieldHoldsUntilTheTimeout()
    local near, far = convergingPair()
    self.vehicles = { near, far }

    local _, module = detectsTrafficFor(far)

    g_time = g_time + AutoDrive.AD_TRAFFIC_YIELD_TIMEOUT - 1
    lu.assertTrue(module:detectAdTrafficOffRoute())
end

--- The clock belongs to one conflict, not to the vehicle. Once the paths no longer meet, the next
--- encounter gets the full wait again rather than inheriting a spent one.
function TestOffRouteLookAhead:testAClearedConflictRestartsTheClock()
    local near, far = convergingPair()
    self.vehicles = { near, far }
    local _, module = detectsTrafficFor(far)

    g_time = g_time + AutoDrive.AD_TRAFFIC_YIELD_TIMEOUT - 1
    self.vehicles = { far }
    lu.assertFalse(module:detectAdTrafficOffRoute(), 'test setup: alone there is nothing to yield to')

    self.vehicles = { near, far }
    g_time = g_time + 2
    lu.assertTrue(module:detectAdTrafficOffRoute(),
        'a fresh conflict must get the full wait, not the remains of the previous one')
end

--- The cull that stands in front of the expensive part of the scan. Its bound is exact rather than
--- generous: getUpcomingPathPoints seeds from each vehicle own position and stops at the look-ahead,
--- so every point it returns is within that of its own vehicle.
function TestOffRouteLookAhead:testAFarAwayVehicleIsCulledButANearOneIsNot()
    local near, far = convergingPair()
    self.vehicles = { near, far }
    lu.assertTrue(detectsTrafficFor(far), 'test setup: this pair has to conflict')

    local bound = AutoDrive.AD_TRAFFIC_LOOKAHEAD * 2 + AutoDrive.AD_TRAFFIC_CONFLICT_RANGE
    MockEngine.nodePositions[near.components[1].node] = { x = 42, y = 0, z = bound + 20 }
    lu.assertFalse(detectsTrafficFor(far),
        'a vehicle further off than two look-aheads plus the conflict range cannot share a point with us')
end

--- trafficVehicle is shared with detectAdTrafficOnRoute, which on its own non-gate frames answers
--- from nothing but whether that field is nil. A yield that never releases it therefore makes the
--- OTHER check report traffic for the rest of the drive - and a route moving from a field onto the
--- network is how every path search ends, so it would happen on most journeys.
function TestOffRouteLookAhead:testTheTrafficVehicleIsReleasedWhenTheConflictClears()
    local near, far = convergingPair()
    self.vehicles = { near, far }

    local yields, module = detectsTrafficFor(far)
    lu.assertTrue(yields)
    lu.assertNotNil(module.trafficVehicle, 'test setup: yielding has to record the partner')

    self.vehicles = { far }
    lu.assertFalse(detectsTrafficFor(far))
    module:detectAdTrafficOffRoute()

    lu.assertNil(module.trafficVehicle,
        'a stale partner here makes the on-network check report traffic that is not there')
end

--- And the case that actually happens: the route leaves the field and joins the network, which is
--- how every path search ends. That returns from the scan before any of the clearing at the bottom.
function TestOffRouteLookAhead:testTheTrafficVehicleIsReleasedOnJoiningTheNetwork()
    local near, far = convergingPair()
    self.vehicles = { near, far }

    local yields, module = detectsTrafficFor(far)
    lu.assertTrue(yields)
    lu.assertNotNil(module.trafficVehicle)

    far.ad.drivePathModule.isOnRoadNetwork = function() return true end
    lu.assertFalse(module:detectAdTrafficOffRoute())

    lu.assertNil(module.trafficVehicle,
        'leaving the field for the network must not carry the partner into the on-network check')
    lu.assertNil(module.offRouteYieldStart, 'and the wait clock goes with it')
end

--- Same for a driver that is switched off mid-yield.
function TestOffRouteLookAhead:testTheTrafficVehicleIsReleasedWhenTheDriverStops()
    local near, far = convergingPair()
    self.vehicles = { near, far }
    local _, module = detectsTrafficFor(far)

    far.ad.stateModule.isActive = function() return false end
    module:detectAdTrafficOffRoute()

    lu.assertNil(module.trafficVehicle)
end

--- trafficVehicle has two writers and hasDetectedObstable runs the on-route check FIRST, so
--- clearing the field unconditionally wipes what that check had just recorded - one frame after it
--- recorded it, and it answers from the field on its own non-gate frames.
function TestOffRouteLookAhead:testItOnlyClearsThePartnerItSetItself()
    local near, far = convergingPair()
    self.vehicles = { near, far }
    local _, module = detectsTrafficFor(far)

    -- the on-route check records somebody of its own, as it does on every network frame
    local somebodyElse = { id = 99 }
    module.trafficVehicle = somebodyElse
    far.ad.drivePathModule.isOnRoadNetwork = function() return true end

    module:detectAdTrafficOffRoute()

    lu.assertIs(module.trafficVehicle, somebodyElse,
        'the off-route scan must not wipe a partner the on-route scan owns')
end

--- Giving up on a partner after the timeout has to drop the partner too.
function TestOffRouteLookAhead:testTheTimeoutReleasesThePartner()
    local near, far = convergingPair()
    self.vehicles = { near, far }
    local _, module = detectsTrafficFor(far)
    lu.assertNotNil(module.trafficVehicle)

    g_time = g_time + AutoDrive.AD_TRAFFIC_YIELD_TIMEOUT + 1
    lu.assertFalse(module:detectAdTrafficOffRoute())

    lu.assertNil(module.trafficVehicle,
        'driving on while still nominally held up by it is the worst of both')
end

--- A cap that restarts is not a cap. Releasing the yield clears the clock, so without remembering
--- who we gave up on, the very next gate frame found the same conflict, started a fresh ten second
--- wait, and the vehicle spent its life alternating between waiting and one frame of driving.
function TestOffRouteLookAhead:testTheTimeoutDoesNotRearmOnTheNextFrame()
    local near, far = convergingPair()
    self.vehicles = { near, far }

    local yields, module = detectsTrafficFor(far)
    lu.assertTrue(yields)

    g_time = g_time + AutoDrive.AD_TRAFFIC_YIELD_TIMEOUT + 1
    lu.assertFalse(module:detectAdTrafficOffRoute(), 'the give-up frame itself')

    -- the conflict is unchanged, so a re-arm would show up here
    for _ = 1, 4 do
        g_time = g_time + 100
        lu.assertFalse(module:detectAdTrafficOffRoute(),
            'having waited the full cap on this vehicle, it must not stop us again')
    end
end

--- But once the conflict genuinely clears, that vehicle is forgiven and may stop us next time.
function TestOffRouteLookAhead:testTheGiveUpIsForgottenWhenTheConflictClears()
    local near, far = convergingPair()
    self.vehicles = { near, far }
    local _, module = detectsTrafficFor(far)
    g_time = g_time + AutoDrive.AD_TRAFFIC_YIELD_TIMEOUT + 1
    lu.assertFalse(module:detectAdTrafficOffRoute())

    self.vehicles = { far }
    lu.assertFalse(module:detectAdTrafficOffRoute(), 'alone, nothing to yield to')

    self.vehicles = { near, far }
    lu.assertTrue(module:detectAdTrafficOffRoute(),
        'a fresh encounter with the same vehicle gets the full wait again')
end

--- The mirror of the off-route release: a partner recorded ON the network and then left behind when
--- the route turns off onto a field would sit there until the vehicle rejoined the network.
function TestOffRouteLookAhead:testTheOnRouteScanReleasesItsOwnPartnerOnLeavingTheNetwork()
    local near, far = convergingPair()
    self.vehicles = { near, far }
    local _, module = detectsTrafficFor(far)

    -- as the on-route scan would leave it
    module.trafficVehicle = near
    module.offRouteYieldPartner = nil
    far.ad.drivePathModule.isOnRoadNetwork = function() return false end

    module:detectAdTrafficOnRoute()

    lu.assertNil(module.trafficVehicle,
        'off the network the on-route scan has to let go of whoever it was holding')
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

--- The exemption has to work while a path search is running, which is the only time it is asked.
--- It used to read the searching vehicle's own route to find its target - and during a search there
--- is no such route, because reachedTarget clears it the moment the previous one ends. It found nil
--- every time, so the exemption never applied and the deadlock it prevents was back.
function TestRouteReservation:testTheSearchTargetIsTakenFromTheSearchNotTheFinishedRoute()
    local other = routedVehicle(2, pathFinderRoute(12, 100, 0))
    local mine = routedVehicle(1, nil)
    -- exactly the state a vehicle is in while its next path is being searched for
    mine.ad.drivePathModule = { getWayPoints = function() return nil, nil end }
    AutoDrive.getAllVehicles = function() return { mine, other } end

    local box = boxAtEndOf(other.ad.drivePathModule:getWayPoints())

    lu.assertTrue(AutoDrive.checkForVehiclePathInBox(box, 5, mine, nil, { x = 400, z = 400 }),
        'searching towards somewhere else must still protect the other destination')
    lu.assertFalse(AutoDrive.checkForVehiclePathInBox(box, 5, mine, nil, { x = 100, z = 0 }),
        'searching towards the same place must open it, or neither vehicle can ever arrive')
end

function TestRouteReservation:testAnInactiveVehicleReservesNothing()
    local other = routedVehicle(2, pathFinderRoute(12, 100, 0))
    other.ad.stateModule.isActive = function() return false end
    local mine = routedVehicle(1, pathFinderRoute(12, 400, 400))
    AutoDrive.getAllVehicles = function() return { mine, other } end

    lu.assertFalse(AutoDrive.checkForVehiclePathInBox(boxAtEndOf(other.ad.drivePathModule:getWayPoints()), 5, mine))
end


------------------------------------------------------------------------------------------------------------------------
--- A real crossing
---
--- Every scene above puts both vehicles on the same line with one path a subset of the other, which
--- cannot produce the case the rule exists to arbitrate. This builds the case: two vehicles meeting
--- at right angles, points three metres apart.
---
--- That geometry broke the first version of the rule. It decided on the FIRST pair of points within
--- conflict range where "my distance is greater than theirs", and the pair test is symmetric - both
--- vehicles walk the same pairs with the roles swapped. So A found its point three metres past the
--- crossing (18 m along its path) against B's point three metres before it (12 m along B's) and
--- gave way; B found the mirror of that pair and gave way too. Both stopped.
------------------------------------------------------------------------------------------------------------------------
TestCrossing = {}

--- A vehicle approaching the origin along an axis, points every three metres.
local function approaching(id, axis, startDistance, speed)
    local points = {}
    local d = startDistance - 3
    while d > -12 do
        if axis == 'x' then
            points[#points + 1] = { x = -d, y = 0, z = 0 }
        else
            points[#points + 1] = { x = 0, y = 0, z = -d }
        end
        d = d - 3
    end
    local sx, sz = 0, 0
    if axis == 'x' then sx = -startDistance else sz = -startDistance end

    local v = TestSetup.vehicle()
    v.id = id
    v.components = { { node = 'c' .. id } }
    MockEngine.nodePositions['c' .. id] = { x = sx, y = 0, z = sz }
    v.lastSpeedReal = speed or 0.002
    v.ad.stateModule = { isActive = function() return true end }
    v.ad.drivePathModule = {
        getWayPoints = function() return points, 1 end,
        isOnRoadNetwork = function() return false end,
    }
    return v
end

function TestCrossing:setUp()
    TestSetup.reset()
    g_time = 10000
    self.vehicles = nil
    AutoDrive.getAllVehicles = function() return self.vehicles or {} end
    AutoDrive.checkIsConnected = function() return false end
    self.a = approaching(1, 'x', 15)
    self.b = approaching(2, 'z', 15)
    self.vehicles = { self.a, self.b }
end

--- The whole point of a right of way rule: it has to produce one answer, not two.
function TestCrossing:testExactlyOneOfThemYields()
    local aYields = detectsTrafficFor(self.a)
    local bYields = detectsTrafficFor(self.b)

    lu.assertNotEquals(aYields, bYields,
        aYields and 'both vehicles gave way, so neither crossing gets used'
               or 'neither vehicle gave way at a crossing they both reach at the same distance')
end

--- Whoever still has further to go is the one that waits.
function TestCrossing:testTheFurtherOneYieldsAtACrossing()
    self.b = approaching(2, 'z', 27)
    self.vehicles = { self.a, self.b }

    lu.assertTrue(detectsTrafficFor(self.b), 'the vehicle twelve metres further back has to wait')
    lu.assertFalse(detectsTrafficFor(self.a))
end

--- Both answers have to be stable, not dependent on which vehicle happens to ask first.
function TestCrossing:testTheDecisionDoesNotDependOnWhoAsksFirst()
    local bFirst = detectsTrafficFor(self.b)
    local aSecond = detectsTrafficFor(self.a)

    TestSetup.reset()
    g_time = 10000
    self.a = approaching(1, 'x', 15)
    self.b = approaching(2, 'z', 15)
    self.vehicles = { self.a, self.b }
    local aFirst = detectsTrafficFor(self.a)
    local bSecond = detectsTrafficFor(self.b)

    lu.assertEquals(aFirst, aSecond)
    lu.assertEquals(bFirst, bSecond)
end

--- The wait is per partner. One shared clock would charge the time already spent waiting for one
--- vehicle to the next one met, so a driver crossing a busy yard would arrive at a fresh conflict
--- with its patience spent and drive straight through it.
function TestCrossing:testTheWaitStartsAfreshForANewPartner()
    local yields, module = detectsTrafficFor(self.b)
    lu.assertTrue(yields, 'test setup: b has to be the one waiting here')

    g_time = g_time + AutoDrive.AD_TRAFFIC_YIELD_TIMEOUT - 100

    -- the first partner leaves and a different one arrives, closer than we are, so we are the one
    -- waiting again on distance alone rather than on the identity tie break
    local c = approaching(3, 'x', 9)
    self.vehicles = { self.b, c }
    g_updateLoopIndex = (AutoDrive.PERF_FRAMES - self.b.id) % AutoDrive.PERF_FRAMES

    lu.assertTrue(module:detectAdTrafficOffRoute(),
        'the new partner has to get the full wait, not the remains of the previous one')
end


os.exit(lu.LuaUnit.run())
