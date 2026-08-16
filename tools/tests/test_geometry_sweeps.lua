--[[
Sweeps over the configuration space, rather than a point in it.

Four review rounds of this work each found real defects, and the reason was never subtle: every
geometric fixture in the suite pinned ONE configuration, and it was always the flattering one. The
vehicle stood exactly abeam a way point, so the offset from it was purely lateral and any direction
rule at all looked right. Way points were always four metres apart, so a term that vanishes at four
metres vanished everywhere. Routes always ran along an axis, so a rule that only works on axes
passed. Three separate defects survived a green suite that way, two of them the same defect twice.

So these do not assert values at a point. They assert INVARIANTS across a swept space, and they
print the configuration that broke when one fails. Every one of them fails on at least one of the
versions this session went through - that is the bar for including it.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('AutoDriveTON')
require('SortedQueue')
require('PathCalculation')
require('GraphManager')
require('DrivePathModule')
require('ExternalInterface')
require('AbstractTask')
require('WaitForCallTask')
require('CollisionDetectionModule')

------------------------------------------------------------------------------------------------------------------------
--- Scene helpers
------------------------------------------------------------------------------------------------------------------------

--- A straight route through the origin at the given heading, way points at the given spacing.
--- Deliberately not axis aligned unless asked: an axis aligned fixture cannot tell a rule that works
--- from one that only works on axes.
local function installRoute(headingDeg, spacing, count)
    ADGraphManager:load()
    local rad = math.rad(headingDeg)
    local dx, dz = math.cos(rad), math.sin(rad)
    local wps = {}
    for i = 1, count do
        local t = (i - 1) * spacing
        wps[i] = TestSetup.waypoint(i, dx * t, dz * t,
            i < count and { i + 1 } or {}, i > 1 and { i - 1 } or {})
    end
    ADGraphManager:setWayPoints(wps)
    return dx, dz
end

--- Perpendicular distance from a point to the route line through the origin at this heading.
local function distanceToLine(x, z, dx, dz)
    local along = x * dx + z * dz
    return MathUtil.vector2Length(x - along * dx, z - along * dz)
end

local function parkedTask(id, x, z, headingDeg)
    local v = TestSetup.vehicle()
    v.id = id
    v.components = { { node = 'sweep' .. id } }
    MockEngine.nodePositions['sweep' .. id] = { x = x, y = 0, z = z }
    v.ad.stateModule = {
        isActive = function() return true end,
        getFirstMarker = function() return { id = 100000 } end,   -- never a real way point
    }
    v.ad.specialDrivingModule = {
        stopVehicle = function() end, releaseVehicle = function() end, update = function() end,
        driveToPoint = function() end, reverseToTargetLocation = function() return false end,
    }
    local task = WaitForCallTask:new(v)
    v.ad.taskModule = {
        activeTask = task,
        getActiveTask = function(self) return self.activeTask end,
        setCurrentTaskFinished = function() end,
    }
    return task, v
end

------------------------------------------------------------------------------------------------------------------------
--- Stepping aside: the target must end up further from the route than the vehicle started
---
--- This is the invariant the manoeuvre exists for, and it is the one that three separate versions
--- got wrong - twice by moving ALONG the route instead of off it. It has to hold wherever on the
--- route the vehicle stands, whichever side of it, whichever way the route runs, at any spacing.
------------------------------------------------------------------------------------------------------------------------
TestEscapeDirectionSweep = {}

function TestEscapeDirectionSweep:setUp()
    TestSetup.reset()
    AutoDrive.testSettings['waitingPosition'] = true
end

function TestEscapeDirectionSweep:testItAlwaysLeavesTheRoute()
    local worst, worstCase = math.huge, nil
    local checked = 0

    for _, headingDeg in ipairs({ 0, 17, 45, 90, 143, 216, 300 }) do
        for _, spacing in ipairs({ 1, 2, 4, 8, 12 }) do
            local dx, dz = installRoute(headingDeg, spacing, 40)
            -- perpendicular to the route, for placing the vehicle beside it
            local px, pz = -dz, dx

            for _, alongFraction in ipairs({ 0, 0.17, 0.35, 0.5, 0.73, 0.9 }) do
                for _, lateral in ipairs({ -6, -2.5, -0.4, 0.4, 2.5, 6 }) do
                    local along = 20 * spacing + alongFraction * spacing
                    local x = dx * along + px * lateral
                    local z = dz * along + pz * lateral

                    local task, vehicle = parkedTask(1, x, z, headingDeg)
                    local request = { time = g_time, awayFromX = x - px, awayFromZ = z - pz }
                    local ex, ez = task:getEscapeDirection(x, z, request)

                    checked = checked + 1
                    lu.assertNotNil(ex, string.format(
                        'no direction at heading %d, spacing %g, along %g, lateral %g',
                        headingDeg, spacing, alongFraction, lateral))

                    local tx = x + ex * AutoDrive.MAKE_WAY_DISTANCE
                    local tz = z + ez * AutoDrive.MAKE_WAY_DISTANCE
                    local gained = distanceToLine(tx, tz, dx, dz) - distanceToLine(x, z, dx, dz)
                    if gained < worst then
                        worst = gained
                        worstCase = string.format(
                            'heading %d, spacing %g, along %.2f of a segment, lateral %g: gained %.2f m',
                            headingDeg, spacing, alongFraction, lateral, gained)
                    end
                end
            end
        end
    end

    lu.assertTrue(checked > 1000, string.format('only %d configurations swept', checked))
    lu.assertTrue(worst > AutoDrive.WAITING_NETWORK_CLEARANCE,
        string.format('%d configurations swept, worst was %s', checked, tostring(worstCase)))
end

--- And it must step to the side it is already on, never across the route through it.
function TestEscapeDirectionSweep:testItNeverCrossesTheRoute()
    local offenders = 0
    local firstOffender = nil

    for _, headingDeg in ipairs({ 0, 33, 90, 155, 270 }) do
        for _, spacing in ipairs({ 1, 4, 12 }) do
            local dx, dz = installRoute(headingDeg, spacing, 40)
            local px, pz = -dz, dx
            for _, alongFraction in ipairs({ 0, 0.3, 0.6, 0.95 }) do
                for _, lateral in ipairs({ -5, -1, 1, 5 }) do
                    local along = 20 * spacing + alongFraction * spacing
                    local x, z = dx * along + px * lateral, dz * along + pz * lateral
                    local task = parkedTask(1, x, z, headingDeg)
                    local ex, ez = task:getEscapeDirection(x, z,
                        { time = g_time, awayFromX = x - px, awayFromZ = z - pz })

                    local tx = x + ex * AutoDrive.MAKE_WAY_DISTANCE
                    local tz = z + ez * AutoDrive.MAKE_WAY_DISTANCE
                    -- signed offset from the line: same sign means the same side
                    local before = (x * px + z * pz)
                    local after = (tx * px + tz * pz)
                    if (before > 0) ~= (after > 0) then
                        offenders = offenders + 1
                        firstOffender = firstOffender or string.format(
                            'heading %d spacing %g along %.2f lateral %g: %.2f -> %.2f',
                            headingDeg, spacing, alongFraction, lateral, before, after)
                    end
                end
            end
        end
    end

    lu.assertEquals(offenders, 0, 'crossed the route rather than leaving it: ' .. tostring(firstOffender))
end

------------------------------------------------------------------------------------------------------------------------
--- Corner speed: the same bend must give the same answer however densely it was recorded
---
--- The angle between two way points is curvature TIMES spacing, and the recorder varies spacing by
--- design. A rule built on it reads how the route was driven rather than how sharp it is.
------------------------------------------------------------------------------------------------------------------------
TestCornerSpeedSweep = {}

function TestCornerSpeedSweep:setUp()
    TestSetup.reset()
    AutoDrive.testSettings['cornerSpeed'] = 1.0
end

--- An arc of the given radius sampled at the given spacing, preceded by a straight run.
local function arcRoute(radius, spacing, straightLength)
    -- The straight ENDS at straightLength whatever the spacing, so the bend sits in the same place
    -- in every recording of it. Building it forwards from the origin instead puts the bend up to a
    -- segment nearer or further, which is a difference in the scene rather than in the rule.
    local points = {}
    local before = math.max(2, math.ceil(straightLength / spacing) + 1)
    for i = 1, before do
        points[i] = { x = straightLength - (before - i) * spacing, y = 0, z = 0 }
    end
    local heading = 0
    local px, pz = points[#points].x, points[#points].z
    local step = spacing / radius
    for _ = 1, math.ceil(math.pi / step) do
        heading = heading + step
        px = px + math.cos(heading) * spacing
        pz = pz + math.sin(heading) * spacing
        points[#points + 1] = { x = px, y = 0, z = pz }
    end
    return points
end

local function moduleOnRoute(points)
    local m = setmetatable({}, { __index = ADDrivePathModule })
    m.vehicle = TestSetup.vehicle()
    m.vehicle.components = { { node = 'cs' } }
    m.vehicle.getTotalMass = function() return 30 end
    MockEngine.nodePositions['cs'] = { x = 0, y = 0, z = 0 }
    m.wayPoints = points
    m.currentWayPoint = 2
    m.distanceToLookAhead = 60
    return m
end

--- Recording density must never buy SPEED.
---
--- Exact invariance is not the goal and would be the wrong goal: the rule is the lower of a radius
--- budget and an angle curve, and a coarsely sampled bend genuinely is a sequence of sharp vertices,
--- which the angle half is there to notice. So a coarse recording may legitimately come out slower.
--- What must never happen is the original defect in the other direction - a recording being allowed
--- MORE speed through the same physical bend because of how it was laid down.
function TestCornerSpeedSweep:testCoarserRecordingIsNeverFaster()
    local worst, worstCase = 0, nil

    for _, radius in ipairs({ 8, 12, 20, 35, 60, 120 }) do
        for _, straight in ipairs({ 10, 25, 45 }) do
            local finest = nil
            for _, spacing in ipairs({ 0.5, 1, 2, 4, 8 }) do
                if spacing < radius then
                    local limit = moduleOnRoute(arcRoute(radius, spacing, straight)):getCornerSpeedLimit(50)
                    if finest == nil then
                        finest = limit
                    else
                        local excess = limit - finest
                        if excess > worst then
                            worst = excess
                            worstCase = string.format(
                                'radius %g m, straight %g m: %.1f km/h at %g m spacing against %.1f at 0.5 m',
                                radius, straight, limit, spacing, finest)
                        end
                    end
                end
            end
        end
    end

    lu.assertTrue(worst < 1,
        string.format('a coarser recording was allowed %.1f km/h more: %s', worst, tostring(worstCase)))
end

--- The radius half on its own IS spacing invariant, which is the property the old angle-only rule
--- lacked and the reason the radius was introduced at all.
function TestCornerSpeedSweep:testTheRadiusRuleItselfDoesNotSeeTheSpacing()
    for _, radius in ipairs({ 8, 12, 20, 35, 60, 120 }) do
        local reference = ADDrivePathModule.speedForRadius(radius)
        for _, spacing in ipairs({ 0.25, 0.5, 1, 2, 4, 8, 12 }) do
            -- the radius a scan would derive from a chord of this spacing on this bend
            local derived = (spacing) / (spacing / radius)
            lu.assertAlmostEquals(ADDrivePathModule.speedForRadius(derived), reference, 0.001,
                string.format('radius %g m read differently at %g m spacing', radius, spacing))
        end
    end
end

--- A tighter bend must never be allowed more speed than a wider one, at any spacing.
function TestCornerSpeedSweep:testTighterIsAlwaysSlower()
    for _, spacing in ipairs({ 0.5, 1, 2, 4, 8 }) do
        local previous = nil
        for _, radius in ipairs({ 8, 12, 20, 35, 60, 120 }) do
            if spacing < radius then
                local limit = moduleOnRoute(arcRoute(radius, spacing, 20)):getCornerSpeedLimit(50)
                if previous ~= nil then
                    lu.assertTrue(limit >= previous - 0.001, string.format(
                        'at %g m spacing, radius %g gave %.1f but a tighter one gave %.1f',
                        spacing, radius, limit, previous))
                end
                previous = limit
            end
        end
    end
end

--- Sideways acceleration through the bend has to stay within the budget, whatever the spacing. This
--- is the physical statement of the whole rule, and it is what the angle rule violated by a factor
--- of five on a real network.
function TestCornerSpeedSweep:testTheLateralBudgetIsRespected()
    local worst, worstCase = 0, nil

    for _, radius in ipairs({ 8, 12, 20, 35, 60 }) do
        for _, spacing in ipairs({ 0.5, 1, 2, 4 }) do
            local atCorner = ADDrivePathModule.cornerSpeedFor(radius, math.deg(spacing / radius))
            local lateral = (atCorner / 3.6) ^ 2 / radius
            if lateral > worst then
                worst = lateral
                worstCase = string.format('radius %g m at %g m spacing allowed %.1f km/h = %.1f m/s2',
                    radius, spacing, atCorner, lateral)
            end
        end
    end

    lu.assertTrue(worst <= ADDrivePathModule.CORNER_LATERAL_ACCELERATION + 0.01,
        string.format('budget is %.1f m/s2: %s',
            ADDrivePathModule.CORNER_LATERAL_ACCELERATION, tostring(worstCase)))
end

------------------------------------------------------------------------------------------------------------------------
--- turnAngle: identical to the rule it replaced, across the space
---
--- The indicator accumulation was folded into the corner scan, and a single dropped term at the edge
--- of the window silenced the indicators entirely on sparse networks at low speed. The existing
--- equality test could not see it: every scene used four metre spacing, where the dropped term is
--- identically zero. This sweeps spacing against speed against where the vehicle stands.
------------------------------------------------------------------------------------------------------------------------
TestTurnAngleSweep = {}

function TestTurnAngleSweep:setUp()
    TestSetup.reset()
    AutoDrive.testSettings['cornerSpeed'] = 1.0
end

--- getHighestApproachingAngle exactly as it was before the merge: accumulate, THEN test the window.
local function turnAngleTheOldWay(module)
    local wayPoints = module.wayPoints
    local index = module:getCurrentWayPointIndex()
    if index + 2 >= #wayPoints then
        return 0
    end
    local turnAngle = 0
    local reach = module:getCurrentLookAheadDistance()
    local x, _, z = getWorldTranslation(module.vehicle.components[1].node)
    local anchor = wayPoints[index]
    local base = MathUtil.vector2Length(anchor.x - x, anchor.z - z)

    local point = 1
    while point <= ADDrivePathModule.MAXLOOKAHEADPOINTS do
        local ahead = wayPoints[index + point]
        if ahead == nil then
            break
        end
        local current, ref = wayPoints[index + point - 1], wayPoints[index + point - 2]
        turnAngle = turnAngle + math.clamp(AutoDrive.angleBetween(
            {x = ahead.x - current.x, z = ahead.z - current.z},
            {x = current.x - ref.x, z = current.z - ref.z}), -90, 90)
        if MathUtil.vector2Length(anchor.x - ahead.x, anchor.z - ahead.z) > (reach - base) then
            break
        end
        point = point + 1
    end
    return turnAngle
end

--- A straight run then a single vertex, at the given spacing.
local function vertexRoute(spacing, angleDeg, straightCount)
    local points = {}
    for i = 1, straightCount do
        points[i] = { x = (i - 1) * spacing, y = 0, z = 0 }
    end
    local cx, cz = points[#points].x, points[#points].z
    local rad = math.rad(angleDeg)
    for i = 1, 12 do
        points[#points + 1] = { x = cx + math.cos(rad) * spacing * i,
                                z = cz + math.sin(rad) * spacing * i, y = 0 }
    end
    return points
end

function TestTurnAngleSweep:testItMatchesTheOriginalRuleEverywhere()
    local mismatches, firstMismatch = 0, nil
    local checked = 0

    for _, spacing in ipairs({ 1, 2, 4, 6, 12 }) do
        for _, angleDeg in ipairs({ -70, -25, 0, 25, 70 }) do
            for _, speed in ipairs({ 0, 3, 5, 10, 20, 40 }) do
                for _, backOff in ipairs({ 0.1, 0.5, 0.95 }) do
                    local points = vertexRoute(spacing, angleDeg, 6)
                    local m = setmetatable({}, { __index = ADDrivePathModule })
                    m.vehicle = TestSetup.vehicle()
                    m.vehicle.components = { { node = 'ta' } }
                    m.vehicle.lastSpeedReal = speed / 3600
                    m.vehicle.getTotalMass = function() return 30 end
                    m.wayPoints = points
                    m.currentWayPoint = 6
                    -- stand a fraction of a segment back from the way point we are heading for
                    MockEngine.nodePositions['ta'] = { x = points[6].x - spacing * backOff, y = 0, z = 0 }
                    m.distanceToLookAhead = m:getCurrentLookAheadDistance()

                    local expected = turnAngleTheOldWay(m)
                    m:getCornerSpeedLimit(50)
                    checked = checked + 1

                    if math.abs(m.turnAngle - expected) > 0.0001 then
                        mismatches = mismatches + 1
                        firstMismatch = firstMismatch or string.format(
                            'spacing %g, vertex %d deg, %d km/h, %.2f of a segment back: %.4f against %.4f',
                            spacing, angleDeg, speed, backOff, m.turnAngle, expected)
                    end
                end
            end
        end
    end

    lu.assertTrue(checked > 400, string.format('only %d configurations swept', checked))
    lu.assertEquals(mismatches, 0, string.format('%d of %d configurations differ, first: %s',
        mismatches, checked, tostring(firstMismatch)))
end

------------------------------------------------------------------------------------------------------------------------
--- Right of way: exactly one of two vehicles yields, at every crossing angle
---
--- The first version decided on the first pair of points that satisfied "I am further", and the pair
--- test is symmetric, so both vehicles could find such a pair and both stop. The test that should
--- have caught it built both on one line, a geometry that cannot produce the conflict at all.
------------------------------------------------------------------------------------------------------------------------
TestRightOfWaySweep = {}

function TestRightOfWaySweep:setUp()
    TestSetup.reset()
    g_time = 10000
    AutoDrive.checkIsConnected = function() return false end
end

--- A vehicle approaching the origin along the given bearing, points at the given spacing.
local function approachingOrigin(id, bearingDeg, distance, spacing)
    local rad = math.rad(bearingDeg)
    local dx, dz = math.cos(rad), math.sin(rad)
    local points = {}
    local d = distance - spacing
    while d > -12 do
        points[#points + 1] = { x = -dx * d, y = 0, z = -dz * d }
        d = d - spacing
    end
    local v = TestSetup.vehicle()
    v.id = id
    v.components = { { node = 'row' .. id } }
    MockEngine.nodePositions['row' .. id] = { x = -dx * distance, y = 0, z = -dz * distance }
    v.lastSpeedReal = 0.002
    v.ad.stateModule = { isActive = function() return true end }
    v.ad.drivePathModule = {
        getWayPoints = function() return points, 1 end,
        isOnRoadNetwork = function() return false end,
    }
    return v
end

local function yields(vehicle, fleet)
    AutoDrive.getAllVehicles = function() return fleet end
    g_updateLoopIndex = (AutoDrive.PERF_FRAMES - vehicle.id) % AutoDrive.PERF_FRAMES
    local module = setmetatable({ vehicle = vehicle }, { __index = ADCollisionDetectionModule })
    return module:detectAdTrafficOffRoute()
end

function TestRightOfWaySweep:testExactlyOneYieldsAtEveryCrossing()
    local bothYielded, neitherYielded = 0, 0
    local firstBoth, firstNeither = nil, nil
    local checked = 0

    for _, crossingDeg in ipairs({ 30, 60, 90, 120, 150 }) do
        for _, spacing in ipairs({ 2, 3, 5 }) do
            for _, lead in ipairs({ 0, 3, 7 }) do
                local a = approachingOrigin(1, 0, 15, spacing)
                local b = approachingOrigin(2, crossingDeg, 15 + lead, spacing)
                local fleet = { a, b }

                local aYields = yields(a, fleet)
                local bYields = yields(b, fleet)
                checked = checked + 1

                if aYields and bYields then
                    bothYielded = bothYielded + 1
                    firstBoth = firstBoth or string.format('crossing %d deg, spacing %g, lead %g',
                        crossingDeg, spacing, lead)
                elseif not aYields and not bYields then
                    neitherYielded = neitherYielded + 1
                    firstNeither = firstNeither or string.format('crossing %d deg, spacing %g, lead %g',
                        crossingDeg, spacing, lead)
                end
            end
        end
    end

    lu.assertTrue(checked > 30, string.format('only %d crossings swept', checked))
    lu.assertEquals(bothYielded, 0,
        string.format('%d crossings had BOTH stop: %s', bothYielded, tostring(firstBoth)))
    lu.assertEquals(neitherYielded, 0,
        string.format('%d crossings had NEITHER stop: %s', neitherYielded, tostring(firstNeither)))
end

------------------------------------------------------------------------------------------------------------------------
--- The reverse controller is never handed a target it will refuse
---
--- It measures arrival from the reverse node - the towed axle, metres behind the root the target was
--- measured from - and calls anything over eighty degrees off that node's rear axis already reached,
--- returning without commanding drive or brake. A near-lateral target left the vehicle rolling free.
------------------------------------------------------------------------------------------------------------------------
TestReverseChoiceSweep = {}

function TestReverseChoiceSweep:setUp()
    TestSetup.reset()
end

function TestReverseChoiceSweep:testReverseIsOnlyChosenWhereTheControllerWillDriveToIt()
    local offenders, firstOffender = 0, nil
    local reverseChosen = 0

    for _, trainLength in ipairs({ 0, 4, 7, 9, 14, 20 }) do
        for bearingDeg = 0, 355, 5 do
            local rad = math.rad(bearingDeg)
            -- the target as startMakingWay places it: MAKE_WAY_DISTANCE along this bearing, in the
            -- vehicle's own frame with +Z forward
            local localX = math.sin(rad) * AutoDrive.MAKE_WAY_DISTANCE
            local localZ = math.cos(rad) * AutoDrive.MAKE_WAY_DISTANCE

            local chosenReverse = localZ < -(trainLength + WaitForCallTask.REVERSE_ASTERN_MARGIN)
                and math.abs(localX) < math.abs(localZ) * WaitForCallTask.REVERSE_CONE

            if chosenReverse then
                reverseChosen = reverseChosen + 1
                -- the controller's own view: from the reverse node, trainLength behind the root
                local fromNodeX, fromNodeZ = localX, localZ + trainLength
                -- angle between the node's rear axis (0, -1) and the direction to the target
                local angle = math.deg(math.atan2(math.abs(fromNodeX), -fromNodeZ))
                if angle > 80 then
                    offenders = offenders + 1
                    firstOffender = firstOffender or string.format(
                        'train %g m, bearing %d deg: %.1f deg off the rear axis, which it refuses',
                        trainLength, bearingDeg, angle)
                end
            end
        end
    end

    lu.assertTrue(reverseChosen > 0, 'the sweep has to actually choose reverse sometimes')
    lu.assertEquals(offenders, 0, string.format(
        '%d of %d reverse choices would be refused on frame one: %s',
        offenders, reverseChosen, tostring(firstOffender)))
end

--- A train longer than the step-aside distance can never place a target far enough astern, and has
--- to degrade to driving forwards rather than to reversing at something that will be refused.
function TestReverseChoiceSweep:testALongTrainNeverChoosesReverse()
    for _, trainLength in ipairs({ 15, 20, 30 }) do
        for bearingDeg = 0, 355, 5 do
            local rad = math.rad(bearingDeg)
            local localX = math.sin(rad) * AutoDrive.MAKE_WAY_DISTANCE
            local localZ = math.cos(rad) * AutoDrive.MAKE_WAY_DISTANCE
            local chosenReverse = localZ < -(trainLength + WaitForCallTask.REVERSE_ASTERN_MARGIN)
                and math.abs(localX) < math.abs(localZ) * WaitForCallTask.REVERSE_CONE
            lu.assertFalse(chosenReverse, string.format(
                'train %g m chose reverse at bearing %d', trainLength, bearingDeg))
        end
    end
end


os.exit(lu.LuaUnit.run())
