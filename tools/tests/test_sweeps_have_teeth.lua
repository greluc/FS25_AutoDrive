--[[
Proof that the sweeps in test_geometry_sweeps.lua would have caught the defects they were written
for.

A green test says nothing on its own; four rounds of this work were green while the code was wrong.
So each sweep is re-run here against the BROKEN rule it exists to reject, and this file fails if any
of them would have passed anyway. That is the difference between a test and a test that works.

The broken versions are transcribed from the actual commits, named in each case.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('AutoDriveTON')
require('SortedQueue')
require('PathCalculation')
require('GraphManager')
require('DrivePathModule')
require('CollisionDetectionModule')

------------------------------------------------------------------------------------------------------------------------
TestSweepsHaveTeeth = {}

function TestSweepsHaveTeeth:setUp()
    TestSetup.reset()
    AutoDrive.testSettings['cornerSpeed'] = 1.0
end

------------------------------------------------------------------------------------------------------------------------
--- 1. The step-aside direction, in both of the versions that got it wrong
------------------------------------------------------------------------------------------------------------------------

--- Commit 5824007 and e1e224f respectively: the vehicle's own axis, then away from the nearest way
--- point. Both leave the route only when the vehicle happens to sit exactly abeam a way point.
local function brokenDirectionOwnAxis(x, z, routeDX, routeDZ)
    -- a vehicle that drove the route is aligned with it, so its own axis IS the route direction
    return routeDX, routeDZ
end

local function brokenDirectionAwayFromPoint(x, z, wpX, wpZ)
    local dx, dz = x - wpX, z - wpZ
    local l = MathUtil.vector2Length(dx, dz)
    if l < 0.001 then
        return 1, 0
    end
    return dx / l, dz / l
end

local function distanceToLine(x, z, dx, dz)
    local along = x * dx + z * dz
    return MathUtil.vector2Length(x - along * dx, z - along * dz)
end

--- The sweep's invariant: after stepping, be further from the route line by at least the clearance.
--- Returns how many swept configurations violate it.
local function sweepDirection(directionFor)
    local violations = 0
    for _, headingDeg in ipairs({ 0, 17, 45, 90, 143 }) do
        local rad = math.rad(headingDeg)
        local dx, dz = math.cos(rad), math.sin(rad)
        local px, pz = -dz, dx
        for _, spacing in ipairs({ 1, 2, 4, 8, 12 }) do
            for _, alongFraction in ipairs({ 0, 0.17, 0.35, 0.5, 0.73, 0.9 }) do
                for _, lateral in ipairs({ -6, -2.5, -0.4, 0.4, 2.5, 6 }) do
                    local along = 20 * spacing + alongFraction * spacing
                    local x, z = dx * along + px * lateral, dz * along + pz * lateral
                    -- the nearest way point on this route
                    local nearestAlong = math.floor(along / spacing + 0.5) * spacing
                    local wpX, wpZ = dx * nearestAlong, dz * nearestAlong

                    local ex, ez = directionFor(x, z, wpX, wpZ, dx, dz)
                    local tx, tz = x + ex * AutoDrive.MAKE_WAY_DISTANCE, z + ez * AutoDrive.MAKE_WAY_DISTANCE
                    local gained = distanceToLine(tx, tz, dx, dz) - distanceToLine(x, z, dx, dz)
                    if gained <= AutoDrive.WAITING_NETWORK_CLEARANCE then
                        violations = violations + 1
                    end
                end
            end
        end
    end
    return violations
end

function TestSweepsHaveTeeth:testTheDirectionSweepRejectsTheOwnAxisVersion()
    local violations = sweepDirection(function(x, z, wpX, wpZ, dx, dz)
        return brokenDirectionOwnAxis(x, z, dx, dz)
    end)
    lu.assertTrue(violations > 0,
        'the own-axis version left the route in every swept configuration, which cannot be right')
end

function TestSweepsHaveTeeth:testTheDirectionSweepRejectsTheAwayFromPointVersion()
    local violations = sweepDirection(function(x, z, wpX, wpZ, dx, dz)
        return brokenDirectionAwayFromPoint(x, z, wpX, wpZ)
    end)
    lu.assertTrue(violations > 0,
        'away-from-the-nearest-point passed the sweep, so the sweep would not have caught it')
end

--- And the fixture the old tests used - exactly abeam a way point - lets the broken rule through,
--- which is why four rounds of green tests meant nothing.
function TestSweepsHaveTeeth:testTheOldPointFixtureAcceptsTheBrokenVersion()
    local dx, dz = 0, 1                 -- route along +Z, as the old fixture had it
    local x, z = 2, 20                  -- exactly abeam the way point at (0, 20)
    local ex, ez = brokenDirectionAwayFromPoint(x, z, 0, 20)
    local tx, tz = x + ex * AutoDrive.MAKE_WAY_DISTANCE, z + ez * AutoDrive.MAKE_WAY_DISTANCE
    local gained = distanceToLine(tx, tz, dx, dz) - distanceToLine(x, z, dx, dz)

    lu.assertTrue(gained > AutoDrive.WAITING_NETWORK_CLEARANCE,
        'the old single-point fixture is exactly the configuration where the broken rule looks right')
end

------------------------------------------------------------------------------------------------------------------------
--- 2. turnAngle, tested before accumulating
------------------------------------------------------------------------------------------------------------------------

--- Commit e1e224f's predecessor: the window test gated the accumulation instead of following it, so
--- the triple straddling the edge was dropped and a window shorter than one segment dropped all.
local function brokenTurnAngle(wayPoints, index, vehicleX, reach)
    local turnAngle = 0
    local anchor = wayPoints[index]
    local base = math.abs(anchor.x - vehicleX)
    local turnAngleReach = reach - base
    local i = index
    local open = (index + 2) < #wayPoints
    while (i + 1) <= #wayPoints and open do
        local ref, current, ahead = wayPoints[i - 1], wayPoints[i], wayPoints[i + 1]
        if ref == nil or ahead == nil then break end
        local signed = AutoDrive.angleBetween(
            {x = ahead.x - current.x, z = ahead.z - current.z},
            {x = current.x - ref.x, z = current.z - ref.z})
        if (i - index + 1) <= ADDrivePathModule.MAXLOOKAHEADPOINTS
            and MathUtil.vector2Length(anchor.x - ahead.x, anchor.z - ahead.z) <= turnAngleReach then
            turnAngle = turnAngle + math.clamp(signed, -90, 90)
        else
            open = false
        end
        i = i + 1
    end
    return turnAngle
end

local function correctTurnAngle(wayPoints, index, vehicleX, reach)
    local turnAngle = 0
    local anchor = wayPoints[index]
    local base = math.abs(anchor.x - vehicleX)
    local turnAngleReach = reach - base
    local i = index
    local open = (index + 2) < #wayPoints
    while (i + 1) <= #wayPoints and open do
        local ref, current, ahead = wayPoints[i - 1], wayPoints[i], wayPoints[i + 1]
        if ref == nil or ahead == nil then break end
        local signed = AutoDrive.angleBetween(
            {x = ahead.x - current.x, z = ahead.z - current.z},
            {x = current.x - ref.x, z = current.z - ref.z})
        turnAngle = turnAngle + math.clamp(signed, -90, 90)
        if (i - index + 1) >= ADDrivePathModule.MAXLOOKAHEADPOINTS
            or MathUtil.vector2Length(anchor.x - ahead.x, anchor.z - ahead.z) > turnAngleReach then
            open = false
        end
        i = i + 1
    end
    return turnAngle
end

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

function TestSweepsHaveTeeth:testTheTurnAngleSweepRejectsTheTestFirstVersion()
    local differences, atFourMetres = 0, 0
    for _, spacing in ipairs({ 1, 2, 4, 6, 12 }) do
        for _, reach in ipairs({ 5, 11, 16, 20, 40, 67 }) do
            for _, backOff in ipairs({ 0.1, 0.5, 0.95 }) do
                local points = vertexRoute(spacing, 70, 6)
                local vehicleX = points[6].x - spacing * backOff
                local broken = brokenTurnAngle(points, 6, vehicleX, reach)
                local correct = correctTurnAngle(points, 6, vehicleX, reach)
                if math.abs(broken - correct) > 0.0001 then
                    differences = differences + 1
                    if spacing == 4 then
                        atFourMetres = atFourMetres + 1
                    end
                end
            end
        end
    end

    lu.assertTrue(differences > 0, 'the two rules never differ, so the sweep proves nothing')
    lu.assertTrue(differences > atFourMetres,
        'they only differ at four metre spacing, which is what every old fixture used')
end

------------------------------------------------------------------------------------------------------------------------
--- 3. Right of way, decided on the first qualifying pair
------------------------------------------------------------------------------------------------------------------------

--- Commit 5824007: return on the first pair within range where our distance is the greater. The pair
--- test is symmetric, so both vehicles find such a pair and both stop.
local function brokenYields(ownPoints, ownDistances, otherPoints, otherDistances)
    for i, a in ipairs(ownPoints) do
        for j, b in ipairs(otherPoints) do
            local dx, dz = a.x - b.x, a.z - b.z
            if (dx * dx + dz * dz) < (AutoDrive.AD_TRAFFIC_CONFLICT_RANGE * AutoDrive.AD_TRAFFIC_CONFLICT_RANGE) then
                if ownDistances[i] > otherDistances[j] then
                    return true
                end
            end
        end
    end
    return false
end

local function pathTowardsOrigin(bearingDeg, distance, spacing)
    local rad = math.rad(bearingDeg)
    local dx, dz = math.cos(rad), math.sin(rad)
    local points, distances = {}, {}
    local d = distance - spacing
    while d > -12 do
        points[#points + 1] = { x = -dx * d, y = 0, z = -dz * d }
        distances[#distances + 1] = distance - d
        d = d - spacing
    end
    return points, distances
end

function TestSweepsHaveTeeth:testTheCrossingSweepRejectsTheFirstPairVersion()
    local bothYielded, onOneLine = 0, 0

    for _, crossingDeg in ipairs({ 30, 60, 90, 120, 150 }) do
        for _, spacing in ipairs({ 2, 3, 5 }) do
            local aP, aD = pathTowardsOrigin(0, 15, spacing)
            local bP, bD = pathTowardsOrigin(crossingDeg, 15, spacing)
            if brokenYields(aP, aD, bP, bD) and brokenYields(bP, bD, aP, aD) then
                bothYielded = bothYielded + 1
            end
        end
    end

    -- the old fixture: both on the same line, one path a subset of the other
    local aP, aD = pathTowardsOrigin(0, 15, 3)
    local bP, bD = pathTowardsOrigin(0, 27, 3)
    if brokenYields(aP, aD, bP, bD) and brokenYields(bP, bD, aP, aD) then
        onOneLine = onOneLine + 1
    end

    lu.assertTrue(bothYielded > 0,
        'no swept crossing made both vehicles yield, so the sweep would not have caught it')
    lu.assertEquals(onOneLine, 0,
        'the old same-line fixture cannot produce the conflict at all, which is why it passed')
end

------------------------------------------------------------------------------------------------------------------------
--- 4. Corner speed from the angle alone
------------------------------------------------------------------------------------------------------------------------

local function brokenSpeedForAngle(angle)
    if angle < 5 then
        return math.huge
    elseif angle < 50 then
        return 12 + 48 * (1 - math.clamp((angle - 5), 0, 25) / (30 - 5))
    end
    return 3
end

function TestSweepsHaveTeeth:testTheLateralBudgetSweepRejectsTheAngleOnlyRule()
    local worst, worstCase = 0, nil
    for _, radius in ipairs({ 8, 12, 20, 35, 60 }) do
        for _, spacing in ipairs({ 0.5, 1, 2, 4 }) do
            local anglePerTriple = math.deg(spacing / radius)
            local allowed = brokenSpeedForAngle(anglePerTriple)
            if allowed < math.huge then
                local lateral = (allowed / 3.6) ^ 2 / radius
                if lateral > worst then
                    worst = lateral
                    worstCase = string.format('radius %g m at %g m spacing: %.1f km/h = %.1f m/s2',
                        radius, spacing, allowed, lateral)
                end
            end
        end
    end

    lu.assertTrue(worst > ADDrivePathModule.CORNER_LATERAL_ACCELERATION * 2,
        string.format('the angle-only rule stayed within twice the budget, so the sweep proves nothing (%s)',
            tostring(worstCase)))
end

------------------------------------------------------------------------------------------------------------------------
--- 5. Reverse chosen for anything behind
------------------------------------------------------------------------------------------------------------------------

function TestSweepsHaveTeeth:testTheReverseSweepRejectsTheAnythingBehindVersion()
    local refused = 0
    for _, trainLength in ipairs({ 0, 4, 7, 9 }) do
        for bearingDeg = 0, 355, 5 do
            local rad = math.rad(bearingDeg)
            local localX = math.sin(rad) * AutoDrive.MAKE_WAY_DISTANCE
            local localZ = math.cos(rad) * AutoDrive.MAKE_WAY_DISTANCE
            -- the broken rule: reverse for anything at all behind us
            if localZ < 0 then
                local fromNodeX, fromNodeZ = localX, localZ + trainLength
                local angle = math.deg(math.atan2(math.abs(fromNodeX), -fromNodeZ))
                if angle > 80 then
                    refused = refused + 1
                end
            end
        end
    end

    lu.assertTrue(refused > 0,
        'no bearing produced a target the reverse controller refuses, so the sweep proves nothing')
end

os.exit(lu.LuaUnit.run())
