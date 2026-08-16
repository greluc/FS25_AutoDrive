--[[
The sweeps themselves, as callable functions over the production code.

Split out from the tests that assert on them for one reason: the file that claims these sweeps can
tell a good rule from a bad one has to run THESE functions against a bad rule, not a transcription of
them. A transcription proves only that the transcription works, which is the same mistake - asserting
a copy of the thing instead of the thing - that let four rounds of review pass on broken code.

So every function here calls the real module. test_geometry_sweeps.lua asserts the counts are zero;
test_sweeps_have_teeth.lua swaps a historical rule into the module, calls the same functions, and
asserts they are not.
]]

Sweeps = {}

------------------------------------------------------------------------------------------------------------------------
--- Scene builders
------------------------------------------------------------------------------------------------------------------------

--- A straight route through the origin at the given heading. Deliberately not axis aligned unless
--- asked: an axis aligned fixture cannot tell a rule that works from one that only works on axes.
function Sweeps.installRoute(headingDeg, spacing, count)
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

function Sweeps.distanceToLine(x, z, dx, dz)
    local along = x * dx + z * dz
    return MathUtil.vector2Length(x - along * dx, z - along * dz)
end

--- A parked task whose vehicle sits at (x, z). The driving module records nothing; these sweeps ask
--- geometry questions, not behaviour ones.
function Sweeps.parkedTask(x, z)
    local v = TestSetup.vehicle()
    v.id = 1
    v.components = { { node = 'sweep' } }
    MockEngine.nodePositions['sweep'] = { x = x, y = 0, z = z }
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
    return task
end

--- An arc of the given radius sampled at the given spacing, after a straight that ENDS at
--- straightLength whatever the spacing - so the bend sits in the same place in every recording of it.
function Sweeps.arcRoute(radius, spacing, straightLength)
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

function Sweeps.moduleOnRoute(points, speedKmh)
    local m = setmetatable({}, { __index = ADDrivePathModule })
    m.vehicle = TestSetup.vehicle()
    m.vehicle.components = { { node = 'cs' } }
    m.vehicle.lastSpeedReal = (speedKmh or 40) / 3600
    m.vehicle.getTotalMass = function() return 30 end
    MockEngine.nodePositions['cs'] = { x = 0, y = 0, z = 0 }
    m.wayPoints = points
    m.currentWayPoint = 2
    m.distanceToLookAhead = 60
    return m
end

--- A straight run then a single vertex, at the given spacing.
function Sweeps.vertexRoute(spacing, angleDeg, straightCount)
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

--- A straight run, then a vertex every three points, turning by each of the given angles in turn.
---
--- One vertex proves nothing about a rule that SUMS over a window. With a single non-zero triple -
--- and it the very first one the scan visits - every part of the rule that decides where the window
--- ENDS only ever adds zeros to the total, so the window could be truncated to one point and the
--- comparison would still match. Several vertices spread through the window put the whole rule on
--- the compared path, and one past ninety degrees - a switchback, or a hand-placed hairpin - puts
--- the clamp on it too.
function Sweeps.multiVertexRoute(spacing, angles, straightCount)
    local points = {}
    for i = 1, straightCount do
        points[i] = { x = (i - 1) * spacing, y = 0, z = 0 }
    end
    local heading = 0
    local px, pz = points[#points].x, points[#points].z
    local function run(count)
        for _ = 1, count do
            px = px + math.cos(heading) * spacing
            pz = pz + math.sin(heading) * spacing
            points[#points + 1] = { x = px, y = 0, z = pz }
        end
    end
    for _, angleDeg in ipairs(angles) do
        heading = heading + math.rad(angleDeg)
        run(3)
    end
    -- and a straight run out past the last vertex, so the window can also end on straight points
    run(8)
    return points
end

--- A vehicle approaching the origin along a bearing, with its upcoming path points.
function Sweeps.approachingOrigin(id, bearingDeg, distance, spacing)
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

------------------------------------------------------------------------------------------------------------------------
--- The sweeps
---
--- Each returns a violation count and a description of the worst case, and each calls the production
--- function it is about. A sweep that restates the rule instead of calling it cannot notice the rule
--- changing, which is exactly what has to be detectable here.
------------------------------------------------------------------------------------------------------------------------

--- How close a run from (x, z) of `distance` metres in direction (ex, ez) passes the point (ax, az).
---
--- Sampled rather than solved, deliberately. The production code has a closed form for this and the
--- side choice is made with it; a sweep that called the same closed form would share any error in it
--- and could not notice. Sampling the path is an independent measurement of the same thing.
function Sweeps.closestApproachOnRun(x, z, ex, ez, distance, ax, az)
    local closest = math.huge
    local steps = 120
    for i = 0, steps do
        local t = distance * i / steps
        local d = MathUtil.vector2Length(ax - (x + ex * t), az - (z + ez * t))
        if d < closest then
            closest = d
        end
    end
    return closest
end

--- Stepping aside has to end up clear of the ROUTE. Distance from the network, not distance gained:
--- when the asker sits on our own side the correct move crosses the line and ends up beyond it, so a
--- gain-based invariant would reject the right answer.
---
--- Measured where the manoeuvre actually ends, and by the same query that decides a vehicle is parked
--- on the network. Measuring the eighteen metre target instead - which is what this swept first -
--- asserts an invariant about a point updateMakingWay does not require the vehicle to reach, and it
--- passed while the vehicle came to rest four metres from the line. The lateral grid runs out to the
--- ten metre gate on the way-point branch, too: it used to stop at six, and every configuration that
--- broke the rule lived between the two.
function Sweeps.escapeLeavesTheRoute()
    local violations, worst, worstCase = 0, math.huge, nil
    local checked, onTheNetwork = 0, 0

    for _, headingDeg in ipairs({ 0, 17, 45, 90, 143, 216, 300 }) do
        for _, spacing in ipairs({ 1, 2, 4, 8, 12 }) do
            local dx, dz = Sweeps.installRoute(headingDeg, spacing, 40)
            local px, pz = -dz, dx
            -- includes the band under half a metre, where getEscapeDirection takes its other branch
            for _, alongFraction in ipairs({ 0, 0.17, 0.35, 0.5, 0.73, 0.9 }) do
                for _, lateral in ipairs({ -9.5, -8, -6, -2.5, -0.45, -0.1, 0, 0.1, 0.45, 2.5, 6, 8, 9.5 }) do
                    for _, askerSide in ipairs({ -1, 1 }) do
                        local along = 20 * spacing + alongFraction * spacing
                        local x = dx * along + px * lateral
                        local z = dz * along + pz * lateral
                        local task = Sweeps.parkedTask(x, z)
                        local request = { time = g_time,
                            awayFromX = x + px * askerSide * 8, awayFromZ = z + pz * askerSide * 8 }

                        local ex, ez, crossing = task:getEscapeDirection(x, z, request)
                        checked = checked + 1
                        if ex == nil then
                            violations = violations + 1
                        elseif ADGraphManager:getNearestWayPointWithin({x = x, z = z},
                                AutoDrive.WAITING_NETWORK_CLEARANCE) ~= nil then
                            -- only where we started ON the network is leaving it the thing to prove;
                            -- further out getEscapeDirection has no route to take a bearing from and
                            -- steers away from the asker instead, which is a different invariant
                            onTheNetwork = onTheNetwork + 1
                            local distance = WaitForCallTask.stepAsideDistance(crossing)
                            local tx = x + ex * distance
                            local tz = z + ez * distance
                            local clear = Sweeps.distanceToLine(tx, tz, dx, dz)
                            if clear < worst then
                                worst = clear
                                worstCase = string.format(
                                    'heading %d, spacing %g, along %.2f, lateral %g, asker side %d: %.2f m from the route after %.1f m',
                                    headingDeg, spacing, alongFraction, lateral, askerSide, clear, distance)
                            end
                            if ADGraphManager:getNearestWayPointWithin({x = tx, z = tz},
                                    AutoDrive.WAITING_NETWORK_CLEARANCE) ~= nil then
                                violations = violations + 1
                            end
                        end
                    end
                end
            end
        end
    end
    return violations, onTheNetwork, worstCase, checked
end

--- And it has to keep out of the asker's way.
---
--- Not "ends up further away", which is what this swept first and is strictly weaker than the
--- property: a run that drives straight THROUGH the asker and out the far side ends up further away
--- too, and that is precisely the failure the side choice exists to prevent. What matters is the
--- closest the run ever comes, over the whole run rather than at its end.
---
--- The bound is the smaller of where we started and the passing clearance. Starting closer than the
--- clearance is allowed - we cannot teleport - but the run must not make it worse; and a run that
--- keeps a full clearance is far enough away that which side it took is nobody's business, which is
--- what lets the rule prefer the side that also leaves the route.
function Sweeps.escapeOpensAGapForTheAsker()
    local violations, worst, worstCase = 0, math.huge, nil
    local checked = 0

    for _, headingDeg in ipairs({ 0, 33, 90, 155, 270 }) do
        for _, spacing in ipairs({ 1, 4, 12 }) do
            local dx, dz = Sweeps.installRoute(headingDeg, spacing, 40)
            local px, pz = -dz, dx
            for _, alongFraction in ipairs({ 0, 0.3, 0.6, 0.95 }) do
                for _, lateral in ipairs({ -5, -1, -0.2, 0.2, 1, 5, 9 }) do
                    -- including an asker nearly in line with us, where the sign of the across
                    -- component is decided by centimetres, and one far enough along the route that
                    -- the along component dwarfs it - the queue geometry, which the first grid had
                    -- no cell for at all
                    for _, askerLateral in ipairs({ -9, -3, -0.3, 0.3, 3, 9,
                                                    lateral - 0.2, lateral + 0.2 }) do
                        for _, askerAlong in ipairs({ -25, -15, -6, 0, 6, 15, 25 }) do
                            local along = 20 * spacing + alongFraction * spacing
                            local x = dx * along + px * lateral
                            local z = dz * along + pz * lateral
                            local ax = dx * (along + askerAlong) + px * askerLateral
                            local az = dz * (along + askerAlong) + pz * askerLateral

                            local task = Sweeps.parkedTask(x, z)
                            local ex, ez, crossing = task:getEscapeDirection(x, z,
                                { time = g_time, awayFromX = ax, awayFromZ = az })
                            checked = checked + 1
                            if ex ~= nil then
                                local distance = WaitForCallTask.stepAsideDistance(crossing)
                                local before = MathUtil.vector2Length(ax - x, az - z)
                                local closest = Sweeps.closestApproachOnRun(x, z, ex, ez, distance, ax, az)
                                local allowed = math.min(before, WaitForCallTask.ASKER_PASSING_CLEARANCE)
                                local slack = closest - allowed
                                if slack < worst then
                                    worst = slack
                                    worstCase = string.format(
                                        'heading %d spacing %g lateral %g, asker at %g across %g along: started %.1f m, passed %.1f m, allowed %.1f m',
                                        headingDeg, spacing, lateral, askerLateral, askerAlong, before, closest, allowed)
                                end
                                if closest < allowed - 0.0001 then
                                    violations = violations + 1
                                end
                            end
                        end
                    end
                end
            end
        end
    end
    return violations, checked, worstCase
end

--- Recording density must never buy SPEED through the same physical bend.
function Sweeps.coarserRecordingIsNeverFaster()
    local violations, worst, worstCase = 0, 0, nil
    local checked = 0

    for _, radius in ipairs({ 8, 12, 20, 35, 60, 120 }) do
        for _, straight in ipairs({ 10, 25, 45 }) do
            local finest = nil
            for _, spacing in ipairs({ 0.5, 1, 2, 4, 8 }) do
                if spacing < radius then
                    local limit = Sweeps.moduleOnRoute(Sweeps.arcRoute(radius, spacing, straight)):getCornerSpeedLimit(50)
                    checked = checked + 1
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
                        if excess >= 1 then
                            violations = violations + 1
                        end
                    end
                end
            end
        end
    end
    return violations, checked, worstCase
end

--- Sideways acceleration through the bend has to stay inside the budget, at every spacing. Goes
--- through getCornerSpeedLimit on a real arc rather than calling the curve directly, so a change to
--- either half of the rule shows up here.
function Sweeps.lateralBudgetIsRespected()
    local violations, worst, worstCase = 0, 0, nil
    local checked = 0

    for _, radius in ipairs({ 8, 12, 20, 35, 60 }) do
        for _, spacing in ipairs({ 0.5, 1, 2, 4 }) do
            if spacing < radius then
                -- right at the bend, so the ramp contributes nothing and the limit is the corner speed
                local points = Sweeps.arcRoute(radius, spacing, spacing)
                local limit = Sweeps.moduleOnRoute(points):getCornerSpeedLimit(50)
                local lateral = (limit / 3.6) ^ 2 / radius
                checked = checked + 1
                if lateral > worst then
                    worst = lateral
                    worstCase = string.format('radius %g m at %g m spacing allowed %.1f km/h = %.1f m/s2',
                        radius, spacing, limit, lateral)
                end
                if lateral > ADDrivePathModule.CORNER_LATERAL_ACCELERATION * 1.6 then
                    violations = violations + 1
                end
            end
        end
    end
    return violations, checked, worstCase
end

--- turnAngle has to equal the rule it replaced, everywhere - not just at the four metre spacing
--- every old fixture used, and not just on a route with one bend in it.
function Sweeps.turnAngleMatchesTheOriginal(originalRule)
    local violations, checked, worstCase = 0, 0, nil

    -- one bend, then several - including a switchback past the ninety degree clamp, and a pair that
    -- cancel, which is the shape that tells a clamped sum from an unclamped one
    local shapes = {
        { -70 }, { -25 }, { 0 }, { 25 }, { 70 },
        { 30, 30, 30, 30 },
        { 40, -40, 40, -40 },
        { 150, -100, 20 },
        { -175, 90, -20, 60 },
        { 120, 120, 120 },
    }

    for _, spacing in ipairs({ 1, 2, 4, 6, 12 }) do
        for _, angles in ipairs(shapes) do
            for _, speed in ipairs({ 0, 3, 5, 10, 20, 40 }) do
                for _, backOff in ipairs({ 0.1, 0.5, 0.95 }) do
                    local points = #angles == 1
                        and Sweeps.vertexRoute(spacing, angles[1], 6)
                        or Sweeps.multiVertexRoute(spacing, angles, 6)
                    local m = Sweeps.moduleOnRoute(points, speed)
                    m.wayPoints = points
                    m.currentWayPoint = 6
                    MockEngine.nodePositions['cs'] = { x = points[6].x - spacing * backOff, y = 0, z = 0 }
                    m.distanceToLookAhead = m:getCurrentLookAheadDistance()

                    local expected = originalRule(m)
                    m:getCornerSpeedLimit(50)
                    checked = checked + 1
                    if math.abs(m.turnAngle - expected) > 0.0001 then
                        violations = violations + 1
                        worstCase = worstCase or string.format(
                            'spacing %g, vertices %s, %d km/h, %.2f back: %.4f against %.4f',
                            spacing, table.concat(angles, '/'), speed, backOff, m.turnAngle, expected)
                    end
                end
            end
        end
    end
    return violations, checked, worstCase
end

--- Exactly one of two vehicles yields at a crossing, whatever the angle.
function Sweeps.exactlyOneYields()
    local violations, checked, worstCase = 0, 0, nil

    for _, crossingDeg in ipairs({ 30, 60, 90, 120, 150 }) do
        for _, spacing in ipairs({ 2, 3, 5 }) do
            for _, lead in ipairs({ 0, 3, 7 }) do
                local a = Sweeps.approachingOrigin(1, 0, 15, spacing)
                local b = Sweeps.approachingOrigin(2, crossingDeg, 15 + lead, spacing)
                local fleet = { a, b }
                AutoDrive.getAllVehicles = function() return fleet end
                AutoDrive.checkIsConnected = function() return false end

                local function yieldsFor(vehicle)
                    g_updateLoopIndex = (AutoDrive.PERF_FRAMES - vehicle.id) % AutoDrive.PERF_FRAMES
                    return ADCollisionDetectionModule:new(vehicle):detectAdTrafficOffRoute()
                end

                local aYields, bYields = yieldsFor(a), yieldsFor(b)
                checked = checked + 1
                if aYields == bYields then
                    violations = violations + 1
                    worstCase = worstCase or string.format(
                        'crossing %d deg, spacing %g, lead %g: both %s',
                        crossingDeg, spacing, lead, tostring(aYields))
                end
            end
        end
    end
    return violations, checked, worstCase
end

--- What the reverse controller refuses: more than this off the reverse node's rear axis and
--- checkWayPointReached returns true without commanding drive or brake.
Sweeps.REVERSE_REFUSAL_DEG = 80

--- And what the sweep demands, which is less. A target that starts ON the refusal threshold is one
--- the manoeuvre cannot finish: backing towards something off to the side SHRINKS the astern
--- distance while the sideways offset stays, so the angle only grows from wherever it starts. The
--- target has to begin far enough inside the acceptance that the whole reverse stays inside it.
---
--- Ten per cent of the threshold is the margin. It is a judgement, and worth knowing how much it
--- binds: at the shipped cone of 0.4 the worst target in this sweep sits at 56.6 deg, and the bound
--- is first crossed at a cone of 0.6. So it catches the cone being widened by half or dropped
--- outright, and would not notice it moving from 0.4 to 0.5. A tolerance that binds, not a tight one.
Sweeps.REVERSE_DRIVABLE_DEG = Sweeps.REVERSE_REFUSAL_DEG * 0.9

--- The reverse controller must never be handed a target it cannot drive to. Calls the production
--- decision, so putting the historical rule back shows up here.
---
--- Swept over the whole range of step-aside distances, not just the nominal one. At eighteen metres
--- the astern margin ALONE caps the angle at atan(18/4) = 77.5 deg, under the controller's own 80 deg
--- refusal - so a check at that threshold could not fire whatever the cone was set to, and the whole
--- suite stayed green with the cone widened twenty-five fold or deleted outright. Two things fix
--- that: a crossing run reaches twice the distance, where a target abeam sits at 83 deg, and the
--- bound is the one the manoeuvre needs rather than the one the controller happens to reject at.
function Sweeps.reverseTargetsAreDrivableTo()
    local violations, chosen, worstCase = 0, 0, nil
    local worst = 0

    for _, trainLength in ipairs({ 0, 4, 7, 9, 14, 20 }) do
        for _, crossing in ipairs({ 0, 4, 8, 10, 16, 30 }) do
            local distance = WaitForCallTask.stepAsideDistance(crossing)
            for bearingDeg = 0, 355, 5 do
                local rad = math.rad(bearingDeg)
                local localX = math.sin(rad) * distance
                local localZ = math.cos(rad) * distance

                if WaitForCallTask.shouldReverseTo(localX, localZ, trainLength) then
                    chosen = chosen + 1
                    -- the controller's own view, from the reverse node trainLength behind the root
                    local fromNodeX, fromNodeZ = localX, localZ + trainLength
                    local angle = math.deg(math.atan2(math.abs(fromNodeX), -fromNodeZ))
                    if angle > worst then
                        worst = angle
                        worstCase = string.format(
                            'train %g m, %.1f m out, bearing %d deg: %.1f deg off the rear axis',
                            trainLength, distance, bearingDeg, angle)
                    end
                    if angle > Sweeps.REVERSE_DRIVABLE_DEG then
                        violations = violations + 1
                    end
                end
            end
        end
    end
    return violations, chosen, worstCase
end

--- Crossing the route is a cost, and it is only worth paying to get out of the asker's way.
---
--- The side we are already on is the one that leaves the route; the other one has to drive back
--- across it first. The rule this replaced flipped on the bare sign of a dot product, so an asker
--- twenty centimetres to one side and twenty-five metres along the route - which passes at
--- twenty-five metres whichever way we go - forced the crossing anyway. Nothing measured that,
--- because both sides satisfy every invariant about the asker: the waste is the whole of the harm,
--- so the waste is what has to be asserted.
function Sweeps.escapeDoesNotCrossWithoutReason()
    local violations, checked, worstCase = 0, 0, nil

    for _, headingDeg in ipairs({ 0, 33, 90, 155, 270 }) do
        for _, spacing in ipairs({ 1, 4, 12 }) do
            local dx, dz = Sweeps.installRoute(headingDeg, spacing, 40)
            local px, pz = -dz, dx
            for _, alongFraction in ipairs({ 0, 0.3, 0.6, 0.95 }) do
                -- only where our own side is well defined; under half a metre from the line
                -- getEscapeDirection takes its other branch and neither side is "ours"
                for _, lateral in ipairs({ -9, -5, -1, 1, 5, 9 }) do
                    for _, askerLateral in ipairs({ -9, -3, -0.3, 0.3, 3, 9,
                                                    lateral - 0.2, lateral + 0.2 }) do
                        for _, askerAlong in ipairs({ -25, -15, -6, 0, 6, 15, 25 }) do
                            local along = 20 * spacing + alongFraction * spacing
                            local x = dx * along + px * lateral
                            local z = dz * along + pz * lateral
                            local ax = dx * (along + askerAlong) + px * askerLateral
                            local az = dz * (along + askerAlong) + pz * askerLateral

                            local side = lateral > 0 and 1 or -1
                            local ownX, ownZ = px * side, pz * side
                            local ownPass = Sweeps.closestApproachOnRun(x, z, ownX, ownZ,
                                AutoDrive.MAKE_WAY_DISTANCE, ax, az)

                            local task = Sweeps.parkedTask(x, z)
                            local ex, ez, crossing = task:getEscapeDirection(x, z,
                                { time = g_time, awayFromX = ax, awayFromZ = az })
                            checked = checked + 1
                            -- a hair of slack, because the sweep samples the run and the rule solves
                            -- it, and the two need not agree to the last centimetre on the boundary
                            if ex ~= nil and ownPass >= WaitForCallTask.ASKER_PASSING_CLEARANCE + 0.05
                                and (crossing or 0) > 0 then
                                violations = violations + 1
                                worstCase = worstCase or string.format(
                                    'heading %d spacing %g lateral %g, asker at %g across %g along: own side passes at %.1f m and it crossed anyway',
                                    headingDeg, spacing, lateral, askerLateral, askerAlong, ownPass)
                            end
                        end
                    end
                end
            end
        end
    end
    return violations, checked, worstCase
end

--- getHighestApproachingAngle exactly as it was before it was folded into the corner scan:
--- accumulate, THEN test the window. Lives here so both the sweep assertions and the proof that the
--- sweep has teeth compare against the same transcription.
function Sweeps.theOriginalTurnAngleRule(module)
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

return Sweeps
