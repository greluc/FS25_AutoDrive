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

--- Stepping aside has to end up clear of the ROUTE. Distance from the line, not distance gained:
--- when the asker sits on our own side the correct move crosses the line and ends up beyond it, so a
--- gain-based invariant would reject the right answer.
function Sweeps.escapeLeavesTheRoute()
    local violations, worst, worstCase = 0, math.huge, nil
    local checked = 0

    for _, headingDeg in ipairs({ 0, 17, 45, 90, 143, 216, 300 }) do
        for _, spacing in ipairs({ 1, 2, 4, 8, 12 }) do
            local dx, dz = Sweeps.installRoute(headingDeg, spacing, 40)
            local px, pz = -dz, dx
            -- includes the band under half a metre, where getEscapeDirection takes its other branch
            for _, alongFraction in ipairs({ 0, 0.17, 0.35, 0.5, 0.73, 0.9 }) do
                for _, lateral in ipairs({ -6, -2.5, -0.45, -0.1, 0, 0.1, 0.45, 2.5, 6 }) do
                    for _, askerSide in ipairs({ -1, 1 }) do
                        local along = 20 * spacing + alongFraction * spacing
                        local x = dx * along + px * lateral
                        local z = dz * along + pz * lateral
                        local task = Sweeps.parkedTask(x, z)
                        local request = { time = g_time,
                            awayFromX = x + px * askerSide * 8, awayFromZ = z + pz * askerSide * 8 }

                        local ex, ez = task:getEscapeDirection(x, z, request)
                        checked = checked + 1
                        if ex == nil then
                            violations = violations + 1
                        else
                            local tx = x + ex * AutoDrive.MAKE_WAY_DISTANCE
                            local tz = z + ez * AutoDrive.MAKE_WAY_DISTANCE
                            local clear = Sweeps.distanceToLine(tx, tz, dx, dz)
                            if clear < worst then
                                worst = clear
                                worstCase = string.format(
                                    'heading %d, spacing %g, along %.2f, lateral %g, asker side %d: %.2f m from the route',
                                    headingDeg, spacing, alongFraction, lateral, askerSide, clear)
                            end
                            if clear <= AutoDrive.WAITING_NETWORK_CLEARANCE then
                                violations = violations + 1
                            end
                        end
                    end
                end
            end
        end
    end
    return violations, checked, worstCase
end

--- And it has to open a gap for whoever asked, not close one.
function Sweeps.escapeOpensAGapForTheAsker()
    local violations, worst, worstCase = 0, math.huge, nil
    local checked = 0

    for _, headingDeg in ipairs({ 0, 33, 90, 155, 270 }) do
        for _, spacing in ipairs({ 1, 4, 12 }) do
            local dx, dz = Sweeps.installRoute(headingDeg, spacing, 40)
            local px, pz = -dz, dx
            for _, alongFraction in ipairs({ 0, 0.3, 0.6, 0.95 }) do
                for _, lateral in ipairs({ -5, -1, -0.2, 0.2, 1, 5 }) do
                    for _, askerLateral in ipairs({ -9, -3, 3, 9 }) do
                        for _, askerAlong in ipairs({ -6, 0, 6 }) do
                            local along = 20 * spacing + alongFraction * spacing
                            local x = dx * along + px * lateral
                            local z = dz * along + pz * lateral
                            local ax = dx * (along + askerAlong) + px * askerLateral
                            local az = dz * (along + askerAlong) + pz * askerLateral

                            local task = Sweeps.parkedTask(x, z)
                            local ex, ez = task:getEscapeDirection(x, z,
                                { time = g_time, awayFromX = ax, awayFromZ = az })
                            checked = checked + 1
                            if ex ~= nil then
                                local tx = x + ex * AutoDrive.MAKE_WAY_DISTANCE
                                local tz = z + ez * AutoDrive.MAKE_WAY_DISTANCE
                                local before = MathUtil.vector2Length(ax - x, az - z)
                                local after = MathUtil.vector2Length(ax - tx, az - tz)
                                local gained = after - before
                                if gained < worst then
                                    worst = gained
                                    worstCase = string.format(
                                        'heading %d spacing %g lateral %g, asker at %g across %g along: %.1f m -> %.1f m',
                                        headingDeg, spacing, lateral, askerLateral, askerAlong, before, after)
                                end
                                if after <= before then
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
--- every old fixture used.
function Sweeps.turnAngleMatchesTheOriginal(originalRule)
    local violations, checked, worstCase = 0, 0, nil

    for _, spacing in ipairs({ 1, 2, 4, 6, 12 }) do
        for _, angleDeg in ipairs({ -70, -25, 0, 25, 70 }) do
            for _, speed in ipairs({ 0, 3, 5, 10, 20, 40 }) do
                for _, backOff in ipairs({ 0.1, 0.5, 0.95 }) do
                    local points = Sweeps.vertexRoute(spacing, angleDeg, 6)
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
                            'spacing %g, vertex %d deg, %d km/h, %.2f back: %.4f against %.4f',
                            spacing, angleDeg, speed, backOff, m.turnAngle, expected)
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
                    local module = setmetatable({ vehicle = vehicle }, { __index = ADCollisionDetectionModule })
                    return module:detectAdTrafficOffRoute()
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

--- The reverse controller must never be handed a target it refuses. Calls the production decision,
--- so putting the historical rule back shows up here.
function Sweeps.reverseTargetsAreDrivableTo()
    local violations, chosen, worstCase = 0, 0, nil

    for _, trainLength in ipairs({ 0, 4, 7, 9, 14, 20 }) do
        for bearingDeg = 0, 355, 5 do
            local rad = math.rad(bearingDeg)
            local localX = math.sin(rad) * AutoDrive.MAKE_WAY_DISTANCE
            local localZ = math.cos(rad) * AutoDrive.MAKE_WAY_DISTANCE

            if WaitForCallTask.shouldReverseTo(localX, localZ, trainLength) then
                chosen = chosen + 1
                -- the controller's own view, from the reverse node trainLength behind the root
                local fromNodeX, fromNodeZ = localX, localZ + trainLength
                local angle = math.deg(math.atan2(math.abs(fromNodeX), -fromNodeZ))
                if angle > 80 then
                    violations = violations + 1
                    worstCase = worstCase or string.format(
                        'train %g m, bearing %d deg: %.1f deg off the rear axis, which it refuses',
                        trainLength, bearingDeg, angle)
                end
            end
        end
    end
    return violations, chosen, worstCase
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
