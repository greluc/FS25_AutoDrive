--[[
Proof that the sweeps can tell a good rule from the bad one it replaced.

A green test says nothing on its own; several rounds of this work were green while the code was
wrong. So each sweep is run again here with a HISTORICAL rule swapped into the production module in
place of the current one, and this file fails if the sweep would have passed anyway.

The distinction that matters, and which the first attempt at this file got wrong: it runs the SAME
functions from sweep-lib.lua, against the SAME module, with one function replaced. It does not
re-implement the sweep. A re-implementation proves only that the re-implementation works - which is
the same mistake, asserting a copy of the thing instead of the thing, that this whole exercise is
about.

Each broken rule below is transcribed from the commit named above it.
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
require('ExternalInterface')
require('AbstractTask')
require('WaitForCallTask')
require('sweep-lib')

------------------------------------------------------------------------------------------------------------------------
TestSweepsHaveTeeth = {}

function TestSweepsHaveTeeth:setUp()
    TestSetup.reset()
    g_time = 10000
    AutoDrive.testSettings['cornerSpeed'] = 1.0
    AutoDrive.testSettings['waitingPosition'] = true
end

--- Swap a function into a table, run something, put the original back whatever happens.
local function withReplaced(holder, name, replacement, body)
    local original = holder[name]
    holder[name] = replacement
    local ok, a, b, c = pcall(body)
    holder[name] = original
    if not ok then
        error(a, 0)
    end
    return a, b, c
end

------------------------------------------------------------------------------------------------------------------------
--- The step-aside direction, in both versions that got it wrong
------------------------------------------------------------------------------------------------------------------------

--- Commit 5824007: the target went along the vehicle's own axis. A driver that has just driven the
--- route is aligned with it, so that is along the route.
local function directionAlongOwnAxis(self, x, z, request)
    local fx, _, fz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local l = MathUtil.vector2Length(fx, fz)
    if l < 0.001 then
        return nil, nil
    end
    return fx / l, fz / l
end

--- Commit e1e224f: away from the nearest way point. Correct only when the vehicle happens to stand
--- exactly abeam one, which is what every fixture of the day did.
local function directionAwayFromNearestPoint(self, x, z, request)
    local wp = ADGraphManager:getNearestWayPointWithin({x = x, z = z}, AutoDrive.WAITING_NETWORK_CLEARANCE)
    local ax, az = request.awayFromX, request.awayFromZ
    if wp ~= nil then
        ax, az = wp.x, wp.z
    end
    local dx, dz = x - ax, z - az
    local l = MathUtil.vector2Length(dx, dz)
    if l < 0.001 then
        return 1, 0
    end
    return dx / l, dz / l
end

--- Commit cadce5f: across the route, but always to the side the vehicle happened to be on, with no
--- regard for where the asker stands.
local function directionAcrossIgnoringTheAsker(self, x, z, request)
    local wp = ADGraphManager:getNearestWayPointWithin({x = x, z = z}, AutoDrive.WAITING_NETWORK_CLEARANCE)
    if wp == nil then
        return directionAwayFromNearestPoint(self, x, z, request)
    end
    local neighbour = nil
    if wp.out ~= nil and wp.out[1] ~= nil then
        neighbour = ADGraphManager:getWayPointById(wp.out[1])
    end
    if neighbour == nil and wp.incoming ~= nil and wp.incoming[1] ~= nil then
        neighbour = ADGraphManager:getWayPointById(wp.incoming[1])
    end
    if neighbour == nil then
        return directionAwayFromNearestPoint(self, x, z, request)
    end
    local rx, rz = neighbour.x - wp.x, neighbour.z - wp.z
    local rl = MathUtil.vector2Length(rx, rz)
    if rl < 0.001 then
        return directionAwayFromNearestPoint(self, x, z, request)
    end
    rx, rz = rx / rl, rz / rl
    local ox, oz = x - wp.x, z - wp.z
    local along = ox * rx + oz * rz
    local acrossX, acrossZ = ox - along * rx, oz - along * rz
    local acrossLength = MathUtil.vector2Length(acrossX, acrossZ)
    if acrossLength > 0.5 then
        return acrossX / acrossLength, acrossZ / acrossLength
    end
    return -rz, rx
end

function TestSweepsHaveTeeth:testTheRouteSweepRejectsTheOwnAxisVersion()
    local violations = withReplaced(WaitForCallTask, 'getEscapeDirection', directionAlongOwnAxis,
        function() return Sweeps.escapeLeavesTheRoute() end)

    lu.assertTrue(violations > 0,
        'the own-axis version left the route in every swept configuration, which cannot be right')
end

function TestSweepsHaveTeeth:testTheRouteSweepRejectsTheAwayFromPointVersion()
    local violations = withReplaced(WaitForCallTask, 'getEscapeDirection', directionAwayFromNearestPoint,
        function() return Sweeps.escapeLeavesTheRoute() end)

    lu.assertTrue(violations > 0,
        'away-from-the-nearest-point passed the route sweep, so the sweep would not have caught it')
end

--- The version that leaves the route correctly but drives at whoever asked. Only the asker sweep
--- can see this one, which is why both exist.
function TestSweepsHaveTeeth:testTheAskerSweepRejectsTheVersionThatIgnoresTheAsker()
    local routeViolations = withReplaced(WaitForCallTask, 'getEscapeDirection', directionAcrossIgnoringTheAsker,
        function() return Sweeps.escapeLeavesTheRoute() end)
    local askerViolations = withReplaced(WaitForCallTask, 'getEscapeDirection', directionAcrossIgnoringTheAsker,
        function() return Sweeps.escapeOpensAGapForTheAsker() end)

    lu.assertEquals(routeViolations, 0,
        'this version does leave the route - if the route sweep caught it, the two sweeps are not independent')
    lu.assertTrue(askerViolations > 0,
        'driving straight at the asker passed the asker sweep, so that sweep proves nothing')
end

--- Commit 2761361: the side was flipped whenever the asker had ANY positive component along it,
--- which is true for an asker essentially in line with us far down the route - where both sides
--- serve it identically and only one keeps us off the network.
local function sideOnTheRawDotSign(self, acrossX, acrossZ, x, z, request)
    if request == nil or request.awayFromX == nil or request.awayFromZ == nil then
        return acrossX, acrossZ
    end
    local toAskerX, toAskerZ = request.awayFromX - x, request.awayFromZ - z
    if (acrossX * toAskerX + acrossZ * toAskerZ) > 0 then
        return -acrossX, -acrossZ
    end
    return acrossX, acrossZ
end

--- This one is worth the extra sweep, and worth recording WHY the other two cannot see it.
---
--- The raw sign rule never drives at the asker - flipping away from it is always safe for the asker -
--- so the asker sweep is satisfied. And since a crossing run is now lengthened by what it has to
--- undo, it still ends up clear of the route, so the route sweep is satisfied too. Its whole harm is
--- the crossing itself: a longer run, across the very line the asker wants, bought for nothing. A
--- defect that costs only waste needs a sweep that measures waste.
function TestSweepsHaveTeeth:testOnlyTheCrossingSweepCatchesTheRawSignRule()
    local routeViolations = withReplaced(WaitForCallTask, 'sideAwayFrom', sideOnTheRawDotSign,
        function() return Sweeps.escapeLeavesTheRoute() end)
    local askerViolations = withReplaced(WaitForCallTask, 'sideAwayFrom', sideOnTheRawDotSign,
        function() return Sweeps.escapeOpensAGapForTheAsker() end)
    local crossViolations = withReplaced(WaitForCallTask, 'sideAwayFrom', sideOnTheRawDotSign,
        function() return Sweeps.escapeDoesNotCrossWithoutReason() end)

    lu.assertEquals(routeViolations, 0,
        'the raw sign rule does still leave the route - if this caught it, the sweeps are not independent')
    lu.assertEquals(askerViolations, 0,
        'and it never drives at the asker, so the asker sweep cannot be the one that catches it')
    lu.assertTrue(crossViolations > 0,
        'so the crossing sweep has to catch it, or nothing in the suite covers the rule at all')
end

--- Commit 2761361 again: the run was a flat eighteen metres whichever way it went, so a crossing
--- from nine metres out ended nine metres out on the other side - still on the network it was
--- clearing. Only visible once the route sweep measures where the manoeuvre ends rather than where
--- its target sits, and only in the lateral band that sweep used not to reach.
local function flatStepAsideDistance()
    return AutoDrive.MAKE_WAY_DISTANCE
end

function TestSweepsHaveTeeth:testTheRouteSweepRejectsTheUnextendedCrossingRun()
    local violations = withReplaced(WaitForCallTask, 'stepAsideDistance', flatStepAsideDistance,
        function() return Sweeps.escapeLeavesTheRoute() end)

    lu.assertTrue(violations > 0,
        'a flat eighteen metre crossing run left the route everywhere, which it does not')
end

--- And the fixture the old tests used - exactly abeam a way point - lets the broken rule through.
--- This is the entire explanation for the rounds of green tests over wrong code.
function TestSweepsHaveTeeth:testTheOldPointFixtureAcceptsTheBrokenVersion()
    local dx, dz = Sweeps.installRoute(90, 4, 40)          -- route along +Z, as the old fixture had it
    local x, z = 2, 20                                      -- exactly abeam the way point at (0, 20)
    local task = Sweeps.parkedTask(x, z)

    local ex, ez = withReplaced(WaitForCallTask, 'getEscapeDirection', directionAwayFromNearestPoint,
        function() return task:getEscapeDirection(x, z, { time = g_time, awayFromX = 2, awayFromZ = 40 }) end)

    local tx, tz = x + ex * AutoDrive.MAKE_WAY_DISTANCE, z + ez * AutoDrive.MAKE_WAY_DISTANCE
    lu.assertTrue(Sweeps.distanceToLine(tx, tz, dx, dz) > AutoDrive.WAITING_NETWORK_CLEARANCE,
        'the old single-point fixture is exactly the configuration where the broken rule looks right')
end

------------------------------------------------------------------------------------------------------------------------
--- turnAngle, tested before accumulating
------------------------------------------------------------------------------------------------------------------------

--- The version before commit cadce5f: the window test gated the accumulation instead of following
--- it, so the triple straddling the edge was dropped, and a window shorter than one segment dropped
--- every one.
local function cornerLimitWithTestFirstTurnAngle(self, currentLimit, cornerScale, cornerFloor)
    local wayPoints = self.wayPoints
    local index = self:getCurrentWayPointIndex()
    self.turnAngle = 0
    if wayPoints == nil or index == nil or index < 2 or (index + 1) > #wayPoints then
        return math.huge
    end
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    local anchor = wayPoints[index]
    local baseDistance = MathUtil.vector2Length(anchor.x - x, anchor.z - z)
    local turnAngleReach = (self.distanceToLookAhead or 0) - baseDistance
    local turnAngleOpen = (index + 2) < #wayPoints
    local i = index

    while (i + 1) <= #wayPoints and turnAngleOpen do
        local ref, current, ahead = wayPoints[i - 1], wayPoints[i], wayPoints[i + 1]
        if ref == nil or ahead == nil then
            break
        end
        local _, _, signed = self:getCornerRadius(ref, current, ahead)
        if (i - index + 1) <= ADDrivePathModule.MAXLOOKAHEADPOINTS
            and MathUtil.vector2Length(anchor.x - ahead.x, anchor.z - ahead.z) <= turnAngleReach then
            if signed ~= nil then
                self.turnAngle = self.turnAngle + math.clamp(signed, -90, 90)
            end
        else
            turnAngleOpen = false
        end
        i = i + 1
    end
    return math.huge
end

function TestSweepsHaveTeeth:testTheTurnAngleSweepRejectsTheTestFirstVersion()
    local violations = withReplaced(ADDrivePathModule, 'getCornerSpeedLimit', cornerLimitWithTestFirstTurnAngle,
        function() return Sweeps.turnAngleMatchesTheOriginal(Sweeps.theOriginalTurnAngleRule) end)

    lu.assertTrue(violations > 0,
        'testing the window before accumulating passed the sweep, so the sweep proves nothing')
end

--- The two mutations above swap the PRODUCTION rule. These two swap the transcription it is compared
--- against, which is the only way to ask whether the fixture puts a given part of the rule on the
--- compared path at all.
---
--- It did not, until this commit. Every route the sweep built had exactly one bend in it, and that
--- bend was the first triple the scan visits - so the window rules and the ninety degree clamp only
--- ever contributed zeros to the compared total, and the whole suite stayed green with the clamp
--- deleted outright. A fixture can be swept over five spacings and six speeds and still be degenerate
--- in the one dimension the rule lives in.

--- The transcription with the clamp taken out. If no swept vertex exceeds ninety degrees, the two
--- agree anyway and the clamp is not being compared.
local function unclampedOriginalRule(module)
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
        turnAngle = turnAngle + AutoDrive.angleBetween(
            {x = ahead.x - current.x, z = ahead.z - current.z},
            {x = current.x - ref.x, z = current.z - ref.z})
        if MathUtil.vector2Length(anchor.x - ahead.x, anchor.z - ahead.z) > (reach - base) then
            break
        end
        point = point + 1
    end
    return turnAngle
end

--- And the transcription cut off after the first triple. If the fixture has only one bend and it is
--- that triple, the two agree anyway and the window is not being compared.
local function firstTripleOnlyRule(module)
    local wayPoints = module.wayPoints
    local index = module:getCurrentWayPointIndex()
    if index + 2 >= #wayPoints then
        return 0
    end
    local ahead, current, ref = wayPoints[index + 1], wayPoints[index], wayPoints[index - 1]
    if ahead == nil or ref == nil then
        return 0
    end
    return math.clamp(AutoDrive.angleBetween(
        {x = ahead.x - current.x, z = ahead.z - current.z},
        {x = current.x - ref.x, z = current.z - ref.z}), -90, 90)
end

function TestSweepsHaveTeeth:testTheTurnAngleFixtureActuallyReachesTheClamp()
    local violations = Sweeps.turnAngleMatchesTheOriginal(unclampedOriginalRule)

    lu.assertTrue(violations > 0,
        'no swept route bends past ninety degrees, so the clamp is not on the compared path')
end

function TestSweepsHaveTeeth:testTheTurnAngleFixtureActuallyReachesBeyondTheFirstTriple()
    local violations = Sweeps.turnAngleMatchesTheOriginal(firstTripleOnlyRule)

    lu.assertTrue(violations > 0,
        'every swept route has one bend and it is the first triple, so the window is not on the compared path')
end

------------------------------------------------------------------------------------------------------------------------
--- Corner speed from the angle alone
------------------------------------------------------------------------------------------------------------------------

--- Before commit 06ba93e: corner speed straight from the angle between two way points, which is
--- curvature TIMES spacing and therefore reads a densely recorded bend as gentle.
local function angleOnlyCornerSpeed(radius, angle)
    if angle == nil or angle < 5 then
        return math.huge
    elseif angle < 50 then
        return 12 + 48 * (1 - math.clamp((angle - 5), 0, 25) / (30 - 5))
    end
    return 3
end

function TestSweepsHaveTeeth:testTheLateralBudgetSweepRejectsTheAngleOnlyRule()
    local violations = withReplaced(ADDrivePathModule, 'cornerSpeedFor', angleOnlyCornerSpeed,
        function() return Sweeps.lateralBudgetIsRespected() end)

    lu.assertTrue(violations > 0,
        'the angle-only rule stayed inside the lateral budget on every swept bend, which it does not')
end

--- And which sweep does NOT catch it, recorded so nobody assumes one covers the other.
---
--- The angle rule's density dependence runs the other way: finer recording gives smaller angles and
--- therefore MORE speed, so a coarser recording is never the faster one and coarserRecordingIsNeverFaster
--- sees nothing. It is the lateral budget that catches this, because the danger is the fine end being
--- allowed a speed the bend cannot take. Two sweeps, two different defects; the density one exists
--- for the opposite mistake, a rule that grows cautious only when the points happen to be close.
function TestSweepsHaveTeeth:testTheDensitySweepAloneWouldNotHaveCaughtTheAngleRule()
    local densityViolations = withReplaced(ADDrivePathModule, 'cornerSpeedFor', angleOnlyCornerSpeed,
        function() return Sweeps.coarserRecordingIsNeverFaster() end)
    local budgetViolations = withReplaced(ADDrivePathModule, 'cornerSpeedFor', angleOnlyCornerSpeed,
        function() return Sweeps.lateralBudgetIsRespected() end)

    lu.assertEquals(densityViolations, 0,
        'the angle rule is slower when coarse, so this sweep cannot be the one that catches it')
    lu.assertTrue(budgetViolations > 0,
        'and the budget sweep has to be, or nothing covers the defect at all')
end

------------------------------------------------------------------------------------------------------------------------
--- Right of way decided on the first qualifying pair
------------------------------------------------------------------------------------------------------------------------

--- Commit 5824007: return on the first pair within range where our distance is the greater. The pair
--- test is symmetric, so both vehicles find such a pair and both stop.
local function firstQualifyingPairYield(self)
    if not self.vehicle.ad.stateModule:isActive() then
        return false
    end
    if self.vehicle.ad.drivePathModule:isOnRoadNetwork() then
        return false
    end
    local ownPoints, ownDistances = self:getUpcomingPathPoints(self.vehicle)
    if ownPoints == nil or #ownPoints < 2 then
        return false
    end
    for _, other in pairs(AutoDrive.getAllVehicles()) do
        if other ~= self.vehicle and other.ad ~= nil and other.ad.drivePathModule ~= nil then
            local otherPoints, otherDistances = self:getUpcomingPathPoints(other)
            if otherPoints ~= nil and #otherPoints >= 2 then
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
            end
        end
    end
    return false
end

function TestSweepsHaveTeeth:testTheCrossingSweepRejectsTheFirstPairVersion()
    local violations = withReplaced(ADCollisionDetectionModule, 'detectAdTrafficOffRoute', firstQualifyingPairYield,
        function() return Sweeps.exactlyOneYields() end)

    lu.assertTrue(violations > 0,
        'deciding on the first qualifying pair produced one yielder at every crossing, which it does not')
end

------------------------------------------------------------------------------------------------------------------------
--- Reverse chosen for anything behind
------------------------------------------------------------------------------------------------------------------------

--- Commit e1e224f: reverse for any target at all behind the vehicle root, which hands the reverse
--- controller targets it refuses on frame one.
local function reverseForAnythingBehind(targetLocalX, targetLocalZ, trainLength)
    return targetLocalZ < 0
end

function TestSweepsHaveTeeth:testTheReverseSweepRejectsTheAnythingBehindVersion()
    local violations = withReplaced(WaitForCallTask, 'shouldReverseTo', reverseForAnythingBehind,
        function() return Sweeps.reverseTargetsAreDrivableTo() end)

    lu.assertTrue(violations > 0,
        'reversing to anything behind produced no refused target, so the sweep proves nothing')
end

--- The rule has two clauses and the mutation above removes both, so passing it said nothing about
--- either one on its own. The astern margin alone bounded the angle below the sweep's threshold at
--- every distance the sweep used, so the cone was pinned by nothing in the entire suite: it could be
--- widened twenty-five fold, or deleted, with 552 tests green.
function TestSweepsHaveTeeth:testTheReverseSweepRejectsTheConeBeingDroppedOnItsOwn()
    local widened = withReplaced(WaitForCallTask, 'REVERSE_CONE', 10,
        function() return Sweeps.reverseTargetsAreDrivableTo() end)
    local gone = withReplaced(WaitForCallTask, 'shouldReverseTo',
        function(targetLocalX, targetLocalZ, trainLength)
            return targetLocalZ < -((trainLength or 0) + WaitForCallTask.REVERSE_ASTERN_MARGIN)
        end,
        function() return Sweeps.reverseTargetsAreDrivableTo() end)

    lu.assertTrue(widened > 0, 'a cone widened twenty-five fold passed the sweep, so the cone is unconstrained')
    lu.assertTrue(gone > 0, 'the cone clause can be deleted and the sweep still passes')
end

os.exit(lu.LuaUnit.run())
