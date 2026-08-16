--[[
Slowing down for a corner.

The old behaviour felt like stutter braking, and the reason was a feedback loop rather than a badly
tuned constant. Corners were looked for within getCurrentLookAheadDistance(), which is proportional
to current speed, and the corner's speed was applied in full the moment a corner fell inside that
window. So: brake for the corner, the window shrinks with the speed, the corner drops out of view,
the limit returns to road speed, accelerate, the window grows, the corner reappears. Traced with the
mod's own formulas at a 40 degree corner 70 m ahead: at 50 km/h the window is 100 m and the limit
12 km/h; at 30 km/h the window is 67 m and there is no limit at all. No stable point exists between
those, so the vehicle cannot settle - it must oscillate.

The replacement asks a different question. For each corner ahead it works out how fast we may be
going here in order to arrive there at the corner's speed, braking at a rate that does not throw the
load about, and takes the tightest answer. That falls smoothly as the corner approaches and does not
depend on how fast we happen to be going, which is what these tests pin down.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('AutoDriveTON')
require('DrivePathModule')

------------------------------------------------------------------------------------------------------------------------
--- Scene: a route along +x with way points four metres apart, as on a recorded route, bending by a
--- given angle at a given distance.
------------------------------------------------------------------------------------------------------------------------
local SPACING = 4

local function straightThenCorner(distanceToCorner, angleDeg)
    local points = {}
    local before = math.floor(distanceToCorner / SPACING) + 1
    for i = 1, before do
        points[i] = { x = (i - 1) * SPACING, y = 0, z = 0 }
    end
    local cx, cz = points[#points].x, points[#points].z
    local rad = math.rad(angleDeg)
    for i = 1, 30 do
        points[#points + 1] = { x = cx + math.cos(rad) * SPACING * i,
                                z = cz + math.sin(rad) * SPACING * i, y = 0 }
    end
    return points
end

--- The module with only what getCornerSpeedLimit touches. currentWayPoint is 2 so the way point
--- before it exists, which the angle needs.
local function moduleOn(points, speedKmh)
    local m = setmetatable({}, { __index = ADDrivePathModule })
    m.vehicle = TestSetup.vehicle()
    m.vehicle.components = { { node = 'veh' } }
    m.vehicle.lastSpeedReal = (speedKmh or 0) / 3600
    MockEngine.nodePositions['veh'] = { x = 0, y = 0, z = 0 }
    m.wayPoints = points
    m.currentWayPoint = 2
    return m
end

------------------------------------------------------------------------------------------------------------------------
TestCornerSpeedLimit = {}

function TestCornerSpeedLimit:setUp()
    TestSetup.reset()
    AutoDrive.testSettings['cornerSpeed'] = 1.0
end

--- The heart of it. The same corner at the same distance has to give the same answer whatever speed
--- the vehicle is doing, or braking removes the reason to brake.
function TestCornerSpeedLimit:testTheLimitDoesNotDependOnCurrentSpeed()
    local points = straightThenCorner(50, 40)
    local limits = {}
    local speeds = { 50, 40, 30, 20, 12, 8 }
    for i, speed in ipairs(speeds) do
        limits[i] = moduleOn(points, speed):getCornerSpeedLimit(50)
    end
    for i = 2, #limits do
        lu.assertAlmostEquals(limits[i], limits[1], 0.001,
            'a limit that changes with current speed is a feedback loop, which is what made it stutter')
    end
end

--- And it has to fall as the corner comes closer, rather than switching on.
function TestCornerSpeedLimit:testTheLimitFallsAsTheCornerApproaches()
    local previous = math.huge
    for _, distance in ipairs({ 60, 50, 40, 30, 20, 12, 8 }) do
        local limit = moduleOn(straightThenCorner(distance, 40), 40):getCornerSpeedLimit(50)
        lu.assertTrue(limit < previous,
            string.format('at %d m the limit (%.1f) has to be below the one further out (%.1f)',
                distance, limit, previous))
        previous = limit
    end
end

--- No cliff between neighbouring distances: the point of the exercise.
function TestCornerSpeedLimit:testTheLimitHasNoStepInIt()
    local last = nil
    for distance = 60, 8, -2 do
        local limit = moduleOn(straightThenCorner(distance, 40), 40):getCornerSpeedLimit(50)
        if last ~= nil then
            lu.assertTrue((last - limit) < 4,
                string.format('limit jumped %.1f km/h between %d m and %d m', last - limit, distance + 2, distance))
        end
        last = limit
    end
end

--- A corner far enough away that we could stop twice over must not slow us at all.
function TestCornerSpeedLimit:testADistantCornerDoesNotConstrainUs()
    lu.assertTrue(moduleOn(straightThenCorner(140, 40), 50):getCornerSpeedLimit(50) > 50)
end

function TestCornerSpeedLimit:testAStraightRouteHasNoLimit()
    local straight = {}
    for i = 1, 40 do straight[i] = { x = (i - 1) * SPACING, y = 0, z = 0 } end

    lu.assertEquals(moduleOn(straight, 50):getCornerSpeedLimit(50), math.huge)
end

--- A sharper corner has to constrain more than a gentle one at the same distance.
function TestCornerSpeedLimit:testASharperCornerConstrainsMore()
    local gentle = moduleOn(straightThenCorner(40, 15), 40):getCornerSpeedLimit(50)
    local sharp = moduleOn(straightThenCorner(40, 60), 40):getCornerSpeedLimit(50)

    lu.assertTrue(sharp < gentle, string.format('60 degrees gave %.1f, 15 degrees gave %.1f', sharp, gentle))
end

--- Arriving at the corner means arriving at the corner's own speed, not at something faster.
function TestCornerSpeedLimit:testItArrivesAtTheCornerSpeed()
    local m = moduleOn(straightThenCorner(4, 40), 12)
    local limit = m:getCornerSpeedLimit(50)
    -- a single 40 degree vertex between two 4 m segments is a bend of this radius
    local cornerSpeed = m:getMaxSpeedForRadius(SPACING / math.rad(40))

    lu.assertTrue(limit >= cornerSpeed, 'the ramp must not undercut the corner speed itself')
    lu.assertTrue(limit < cornerSpeed * 2.5,
        string.format('right at the corner the limit (%.1f) has to be near its speed (%.1f)', limit, cornerSpeed))
end

--- The scan is bounded by the braking distance from the limit in force, so a yard speed does not
--- pay for a road-length search.
function TestCornerSpeedLimit:testALowLimitDoesNotPayForALongSearch()
    local points = straightThenCorner(60, 40)

    lu.assertEquals(moduleOn(points, 10):getCornerSpeedLimit(10), math.huge,
        'at 10 km/h a corner 60 m off is not yet anybodys business')
    lu.assertTrue(moduleOn(points, 50):getCornerSpeedLimit(50) < math.huge,
        'at 50 km/h the same corner is')
end

--- The cornerSpeed setting still scales the result.
function TestCornerSpeedLimit:testTheCornerSpeedSettingStillApplies()
    local points = straightThenCorner(30, 40)
    local atFull = moduleOn(points, 40):getCornerSpeedLimit(50)
    AutoDrive.testSettings['cornerSpeed'] = 0.5
    local atHalf = moduleOn(points, 40):getCornerSpeedLimit(50)

    lu.assertTrue(atHalf < atFull)
end

function TestCornerSpeedLimit:testAShortRouteIsNotAnError()
    lu.assertEquals(moduleOn({ { x = 0, y = 0, z = 0 } }, 20):getCornerSpeedLimit(50), math.huge)
    local m = moduleOn({ { x = 0, y = 0, z = 0 }, { x = 4, y = 0, z = 0 } }, 20)
    lu.assertEquals(m:getCornerSpeedLimit(50), math.huge)
end

------------------------------------------------------------------------------------------------------------------------
--- The old shape, kept as a record of what was wrong
------------------------------------------------------------------------------------------------------------------------
TestTheOldFeedbackLoop = {}

function TestTheOldFeedbackLoop:setUp()
    TestSetup.reset()
    AutoDrive.testSettings['cornerSpeed'] = 1.0
end

--- getCurrentLookAheadDistance is still used - it sets how far the steering looks - and it is still
--- proportional to speed. This pins that fact, because it is the property that made the old speed
--- limit oscillate, and anything that starts deriving the limit from it again brings the stutter back.
function TestTheOldFeedbackLoop:testTheSteeringLookAheadStillShrinksWithSpeed()
    local fast = moduleOn({}, 50)
    fast.vehicle.getTotalMass = function() return 30 end
    local slow = moduleOn({}, 12)
    slow.vehicle.getTotalMass = function() return 30 end

    lu.assertTrue(fast:getCurrentLookAheadDistance() > slow:getCurrentLookAheadDistance() * 2,
        'this is the speed dependence the corner limit must no longer be built on')
end


------------------------------------------------------------------------------------------------------------------------
--- Spacing invariance
---
--- The test that would have caught the original mistake. Corner speed used to come from the angle
--- between two way points, and that angle is curvature TIMES the local spacing - so the same
--- physical bend read as gentle when recorded densely and sharp when recorded coarsely. Worse, the
--- recorder (RecordingModule.lua:186-203) deliberately places points closer together the sharper
--- the turn, so a real corner is always the dense case. Measured on a 47,264 way point network: of
--- 1,622 triples describing bends under 20 m radius, 85 percent were allowed above 25 km/h.
---
--- Every scene above uses a single sharp vertex at 4 m spacing, which is not what a recorded corner
--- looks like. These build a real arc.
------------------------------------------------------------------------------------------------------------------------
TestSpacingInvariance = {}

function TestSpacingInvariance:setUp()
    TestSetup.reset()
    AutoDrive.testSettings['cornerSpeed'] = 1.0
end

--- A straight run, then an arc of the given radius sampled at the given spacing.
local function straightThenArc(distanceToCorner, radius, spacing, sweepDeg)
    local points = {}
    local before = math.max(2, math.floor(distanceToCorner / spacing) + 1)
    for i = 1, before do
        points[i] = { x = (i - 1) * spacing, y = 0, z = 0 }
    end
    local heading = 0
    local px, pz = points[#points].x, points[#points].z
    local step = spacing / radius
    for _ = 1, math.ceil(math.rad(sweepDeg) / step) do
        heading = heading + step
        px = px + math.cos(heading) * spacing
        pz = pz + math.sin(heading) * spacing
        points[#points + 1] = { x = px, y = 0, z = pz }
    end
    return points
end

function TestSpacingInvariance:testTheSameBendGivesTheSameLimitAtAnySpacing()
    local dense = moduleOn(straightThenArc(30, 12, 1, 90), 40):getCornerSpeedLimit(50)
    local coarse = moduleOn(straightThenArc(30, 12, 4, 90), 40):getCornerSpeedLimit(50)

    lu.assertAlmostEquals(dense, coarse, 3,
        string.format('one bend, two recordings: dense gave %.1f, coarse gave %.1f', dense, coarse))
end

--- And a tight bend has to actually be slow. A twelve metre radius is a yard turn; the angle rule
--- allowed fifty km/h through one of those.
function TestSpacingInvariance:testAYardTurnIsSlow()
    local limit = moduleOn(straightThenArc(2, 12, 1, 90), 20):getCornerSpeedLimit(50)

    lu.assertTrue(limit < 25,
        string.format('a 12 m radius turn allowed %.1f km/h, which is %.1f m/s^2 sideways',
            limit, (limit / 3.6) * (limit / 3.6) / 12))
end

function TestSpacingInvariance:testASweepingBendIsNot()
    local tight = moduleOn(straightThenArc(2, 12, 1, 90), 20):getCornerSpeedLimit(50)
    local sweeping = moduleOn(straightThenArc(2, 120, 1, 90), 40):getCornerSpeedLimit(50)

    lu.assertTrue(sweeping > tight * 2,
        string.format('120 m radius gave %.1f, 12 m radius gave %.1f', sweeping, tight))
end

------------------------------------------------------------------------------------------------------------------------
--- Relaxing corner speeds off the road network
---
--- Pathfinder output is full of 45 degree steps that need not be crawled, so its corner speeds are
--- doubled. Applying that to the ramp instead of to the corner speed is the same as raising the
--- braking rate - doubling sqrt(v^2 + 2ad) gives sqrt(4v^2 + 8ad) - which puts the late hard brake
--- back exactly where the ramp was meant to remove it.
------------------------------------------------------------------------------------------------------------------------
TestOffNetworkRelaxation = {}

function TestOffNetworkRelaxation:setUp()
    TestSetup.reset()
    AutoDrive.testSettings['cornerSpeed'] = 1.0
end

function TestOffNetworkRelaxation:testScalingRaisesTheCornerSpeedNotTheBrakingRate()
    local points = straightThenCorner(40, 40)
    local plain = moduleOn(points, 40):getCornerSpeedLimit(50)
    local scaled = moduleOn(points, 40):getCornerSpeedLimit(50, 2, 12)

    lu.assertTrue(scaled > plain, 'test setup: the relaxation has to relax something')

    -- With the scaling applied where it belongs, the implied deceleration over the approach is
    -- unchanged. Applied to the ramp it would come out four times as large.
    local near = moduleOn(straightThenCorner(10, 40), 40):getCornerSpeedLimit(50, 2, 12)
    local far = moduleOn(straightThenCorner(40, 40), 40):getCornerSpeedLimit(50, 2, 12)
    local vNear, vFar = near / 3.6, far / 3.6
    local implied = (vFar * vFar - vNear * vNear) / (2 * 30)

    lu.assertAlmostEquals(implied, ADDrivePathModule.COMFORTABLE_DECELERATION, 0.3,
        string.format('the approach implies %.2f m/s^2, the comfortable rate is %.2f',
            implied, ADDrivePathModule.COMFORTABLE_DECELERATION))
end

function TestOffNetworkRelaxation:testTheFloorDoesNotOverrideAStraight()
    local straight = {}
    for i = 1, 40 do straight[i] = { x = (i - 1) * SPACING, y = 0, z = 0 } end

    lu.assertEquals(moduleOn(straight, 50):getCornerSpeedLimit(50, 2, 12), math.huge,
        'a floor of 12 must not turn "no corner" into "a corner wanting 12 km/h"')
end

------------------------------------------------------------------------------------------------------------------------
--- The steering angle limiter
---
--- The reactive half of the stutter: it used to be a switch, no limit below 95 percent of full lock
--- and 10 km/h above it, so a wheel hovering at that mark flipped the limit every gate frame.
------------------------------------------------------------------------------------------------------------------------
TestSteeringLimiter = {}

function TestSteeringLimiter:setUp()
    TestSetup.reset()
end

local function atLock(fraction)
    local m = setmetatable({}, { __index = ADDrivePathModule })
    m.vehicle = TestSetup.vehicle()
    m.vehicle.maxRotation = math.rad(60)
    m.vehicle.rotatedTime = math.rad(60 * fraction)
    return m:getSpeedLimitBySteeringAngle()
end

function TestSteeringLimiter:testItTapersInsteadOfSwitching()
    local previous = math.huge
    for _, fraction in ipairs({ 0.80, 0.85, 0.90, 0.95, 1.00 }) do
        local limit = atLock(fraction)
        lu.assertTrue(limit <= previous,
            string.format('at %.0f%% lock the limit rose to %.1f', fraction * 100, limit))
        if previous < math.huge then
            lu.assertTrue((previous - limit) < 20,
                string.format('limit stepped %.1f km/h approaching %.0f%% lock', previous - limit, fraction * 100))
        end
        previous = limit
    end
end

--- The endpoints are deliberately unchanged, so this is never more permissive than it was.
function TestSteeringLimiter:testTheEndpointsAreUnchanged()
    lu.assertEquals(atLock(0.5), math.huge, 'gentle steering must still be unconstrained')
    lu.assertEquals(atLock(0.95), 10)
    lu.assertEquals(atLock(1.0), 10)
end


------------------------------------------------------------------------------------------------------------------------
--- The indicator window, folded into the same pass
---
--- turnAngle drives the indicators and used to be accumulated by getHighestApproachingAngle, a
--- second walk over the same way point triples in the same order. The two are now one pass - but
--- they keep separate stopping rules, because the indicator window is straight line distance from
--- the current way point, capped at MAXLOOKAHEADPOINTS, and grows with speed, while the speed window
--- is path distance bounded by the braking distance. Folding them under one rule would have made the
--- indicators behave like a brake.
---
--- These reimplement the ORIGINAL rule independently and demand the same number out of the merged
--- scan, so the refactor cannot quietly move the window.
------------------------------------------------------------------------------------------------------------------------
TestTurnAngle = {}

function TestTurnAngle:setUp()
    TestSetup.reset()
    AutoDrive.testSettings['cornerSpeed'] = 1.0
end

--- getHighestApproachingAngle as it was, transcribed.
local function turnAngleTheOldWay(module)
    local wayPoints = module.wayPoints
    local index = module:getCurrentWayPointIndex()
    local turnAngle = 0
    local distanceToLookAhead = module:getCurrentLookAheadDistance()
    local x, _, z = getWorldTranslation(module.vehicle.components[1].node)

    if index + 2 >= #wayPoints then
        return 0
    end
    local anchor = wayPoints[index]
    local baseDistance = MathUtil.vector2Length(anchor.x - x, anchor.z - z)

    local point = 1
    while point <= ADDrivePathModule.MAXLOOKAHEADPOINTS do
        local ahead = wayPoints[index + point]
        if ahead == nil then
            break
        end
        local current = wayPoints[index + point - 1]
        local ref = wayPoints[index + point - 2]
        local angle = AutoDrive.angleBetween(
            {x = ahead.x - current.x, z = ahead.z - current.z},
            {x = current.x - ref.x, z = current.z - ref.z})
        turnAngle = turnAngle + math.clamp(angle, -90, 90)
        if MathUtil.vector2Length(anchor.x - ahead.x, anchor.z - ahead.z) > (distanceToLookAhead - baseDistance) then
            break
        end
        point = point + 1
    end
    return turnAngle
end

local function checkAgainstTheOldRule(self, points, speed, label)
    local m = moduleOn(points, speed)
    m.vehicle.getTotalMass = function() return 30 end
    m.distanceToLookAhead = m:getCurrentLookAheadDistance()
    local expected = turnAngleTheOldWay(m)

    m:getCornerSpeedLimit(50)

    lu.assertAlmostEquals(m.turnAngle, expected, 0.0001,
        string.format('%s: merged pass gave %.4f, the original rule gives %.4f',
            label, m.turnAngle, expected))
end

function TestTurnAngle:testALeftBendMatchesTheOriginalRule()
    checkAgainstTheOldRule(self, straightThenCorner(20, 40), 40, 'left bend')
end

function TestTurnAngle:testARightBendMatchesTheOriginalRule()
    checkAgainstTheOldRule(self, straightThenCorner(20, -40), 40, 'right bend')
end

function TestTurnAngle:testAStraightMatchesTheOriginalRule()
    local straight = {}
    for i = 1, 40 do straight[i] = { x = (i - 1) * SPACING, y = 0, z = 0 } end
    checkAgainstTheOldRule(self, straight, 40, 'straight')
end

function TestTurnAngle:testItMatchesAtEverySpeed()
    for _, speed in ipairs({ 5, 15, 30, 50 }) do
        checkAgainstTheOldRule(self, straightThenCorner(30, 35), speed,
            string.format('%d km/h', speed))
    end
end

--- The sign has to survive, or the indicators pick the wrong side.
function TestTurnAngle:testTheSignFollowsTheBend()
    local left = moduleOn(straightThenCorner(20, 40), 40)
    left.distanceToLookAhead = 100
    left:getCornerSpeedLimit(50)

    local right = moduleOn(straightThenCorner(20, -40), 40)
    right.distanceToLookAhead = 100
    right:getCornerSpeedLimit(50)

    -- Which sign means which way is angleBetween's convention, not something to assert blind. What
    -- has to hold is that the two bends come out opposite and neither is zero, because that is what
    -- the indicator code reads.
    lu.assertTrue(math.abs(left.turnAngle) > 1, string.format('left bend gave %.1f', left.turnAngle))
    lu.assertTrue(math.abs(right.turnAngle) > 1, string.format('right bend gave %.1f', right.turnAngle))
    lu.assertTrue((left.turnAngle > 0) ~= (right.turnAngle > 0),
        string.format('the two bends have to differ in sign: %.1f and %.1f', left.turnAngle, right.turnAngle))
end

--- Near the end of a route there is nothing left to look at.
function TestTurnAngle:testAShortRouteGivesZero()
    local m = moduleOn({ { x = 0, y = 0, z = 0 }, { x = 4, y = 0, z = 0 } }, 20)
    m.distanceToLookAhead = 100

    m:getCornerSpeedLimit(50)

    lu.assertEquals(m.turnAngle, 0)
end


os.exit(lu.LuaUnit.run())
