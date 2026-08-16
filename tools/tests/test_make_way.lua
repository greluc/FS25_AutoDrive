--[[
Asking a parked vehicle to clear the spot it is standing on.

A vehicle waiting for its next job used to stop where it happened to be and stay there, which on a
field is regularly the spot another vehicle - or the harvester - needs next. Nothing moved it: the
blocked party either routed around it or, after fifteen seconds of standing still, reversed out and
approached again from somewhere else, with the parked vehicle still in the same place.

These tests cover the other half: the blocked party asks, and the parked one steps aside.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('AutoDriveTON')
require('SortedQueue')
require('PathCalculation')
require('GraphManager')
require('ExternalInterface')
require('AbstractTask')
require('WaitForCallTask')

------------------------------------------------------------------------------------------------------------------------
--- Scene
------------------------------------------------------------------------------------------------------------------------
local driveCalls

--- A vehicle at (x, z), facing +Z, whose driving module records what it was told to do instead of
--- moving anything. Position changes are made by the tests directly.
local function vehicleAt(id, x, z, activeTask)
    local v = TestSetup.vehicle()
    v.id = id
    v.components = { { node = 'n' .. id } }
    MockEngine.nodePositions['n' .. id] = { x = x, y = 0, z = z }
    v.ad.stateModule = { isActive = function() return true end }
    v.ad.taskModule = {
        activeTask = activeTask,
        getActiveTask = function(self) return self.activeTask end,
        setCurrentTaskFinished = function() end,
    }
    v.ad.specialDrivingModule = {
        stopVehicle = function() driveCalls[#driveCalls + 1] = { v = id, what = 'stop' } end,
        releaseVehicle = function() end,
        update = function() end,
        driveToPoint = function(_, _, point) driveCalls[#driveCalls + 1] = { v = id, what = 'forward', point = point } end,
        reverseToTargetLocation = function(_, _, point)
            driveCalls[#driveCalls + 1] = { v = id, what = 'reverse', point = point }
            -- the real one returns true when it considers itself there, which it also does for a
            -- target too far off its own rear axis to drive to
            return MockEngine.reverseReportsArrived == true
        end,
    }
    return v
end

local function moveTo(vehicle, x, z)
    MockEngine.nodePositions[vehicle.components[1].node] = { x = x, y = 0, z = z }
end

local function lastDrive()
    return driveCalls[#driveCalls]
end

------------------------------------------------------------------------------------------------------------------------
--- Who may be asked
------------------------------------------------------------------------------------------------------------------------
TestRequest = {}

function TestRequest:setUp()
    TestSetup.reset()
    driveCalls = {}
    g_time = 10000
    ADHarvestManager = { registerAsUnloader = function() end }
    self.parked = vehicleAt(1, 0, 0, { canMakeWay = true })
    self.asker = vehicleAt(2, 0, -20, { canMakeWay = false })
end

function TestRequest:testAParkedVehicleTakesTheRequest()
    lu.assertTrue(AutoDrive:requestMakeWay(self.parked, self.asker))
    lu.assertNotNil(AutoDrive.getMakeWayRequest(self.parked))
end

--- A driving vehicle is following a path it was given. Shoving it off that path is the pathfinder's
--- business, and a task that is mid-manoeuvre has no way to resume where it left off.
function TestRequest:testADrivingVehicleIsNotAsked()
    local driving = vehicleAt(3, 5, 5, { canMakeWay = false })
    lu.assertFalse(AutoDrive:requestMakeWay(driving, self.asker))
    lu.assertNil(AutoDrive.getMakeWayRequest(driving))
end

function TestRequest:testAVehicleWithNoTaskIsNotAsked()
    local idle = vehicleAt(4, 5, 5, nil)
    lu.assertFalse(AutoDrive:requestMakeWay(idle, self.asker))
end

function TestRequest:testAnInactiveVehicleIsNotAsked()
    self.parked.ad.stateModule.isActive = function() return false end
    lu.assertFalse(AutoDrive:requestMakeWay(self.parked, self.asker))
end

function TestRequest:testNilAndSelfAreRefused()
    lu.assertFalse(AutoDrive:requestMakeWay(nil, self.asker))
    lu.assertFalse(AutoDrive:requestMakeWay(self.parked, nil))
    lu.assertFalse(AutoDrive:requestMakeWay(self.parked, self.parked))
end

--- The asker may have driven off, or been deleted, long before the parked one got round to moving.
function TestRequest:testARequestExpires()
    AutoDrive:requestMakeWay(self.parked, self.asker)
    g_time = g_time + AutoDrive.MAKE_WAY_VALID_TIME + 1
    lu.assertNil(AutoDrive.getMakeWayRequest(self.parked))
end

function TestRequest:testARequestSurvivesUntilItExpires()
    AutoDrive:requestMakeWay(self.parked, self.asker)
    g_time = g_time + AutoDrive.MAKE_WAY_VALID_TIME - 1
    lu.assertNotNil(AutoDrive.getMakeWayRequest(self.parked))
end

------------------------------------------------------------------------------------------------------------------------
--- Finding somebody to ask
------------------------------------------------------------------------------------------------------------------------
TestAskNearby = {}

function TestAskNearby:setUp()
    TestSetup.reset()
    driveCalls = {}
    g_time = 10000
    ADHarvestManager = { registerAsUnloader = function() end }
    AutoDrive.checkIsConnected = function() return false end
    self.stuck = vehicleAt(1, 0, 0, { canMakeWay = false })
end

function TestAskNearby:testANearbyParkedVehicleIsAsked()
    local parked = vehicleAt(2, 0, 10, { canMakeWay = true })
    AutoDrive.getAllVehicles = function() return { self.stuck, parked } end

    lu.assertEquals(AutoDrive:askNearbyVehiclesToMakeWay(self.stuck), 1)
    lu.assertNotNil(AutoDrive.getMakeWayRequest(parked))
end

function TestAskNearby:testAVehicleBeyondTheRangeIsLeftAlone()
    local far = vehicleAt(2, 0, AutoDrive.MAKE_WAY_ASK_RANGE + 10, { canMakeWay = true })
    AutoDrive.getAllVehicles = function() return { self.stuck, far } end

    lu.assertEquals(AutoDrive:askNearbyVehiclesToMakeWay(self.stuck), 0)
end

--- Our own trailer stands right next to us by definition and cannot go anywhere on its own.
function TestAskNearby:testOurOwnTrainIsNotAsked()
    local trailer = vehicleAt(2, 0, 8, { canMakeWay = true })
    AutoDrive.getAllVehicles = function() return { self.stuck, trailer } end
    AutoDrive.checkIsConnected = function() return true end

    lu.assertEquals(AutoDrive:askNearbyVehiclesToMakeWay(self.stuck), 0)
end

function TestAskNearby:testWeDoNotAskOurself()
    self.stuck.ad.taskModule.activeTask = { canMakeWay = true }
    AutoDrive.getAllVehicles = function() return { self.stuck } end

    lu.assertEquals(AutoDrive:askNearbyVehiclesToMakeWay(self.stuck), 0)
end

------------------------------------------------------------------------------------------------------------------------
--- Stepping aside
------------------------------------------------------------------------------------------------------------------------
TestMakingWay = {}

function TestMakingWay:setUp()
    TestSetup.reset()
    driveCalls = {}
    g_time = 10000
    ADHarvestManager = { registerAsUnloader = function() end }
    self.task = WaitForCallTask:new(vehicleAt(1, 0, 0, nil))
    self.parked = self.task.vehicle
    self.parked.ad.taskModule.activeTask = self.task
    self.task:setUp()
end

function TestMakingWay:testItStandsStillWithoutARequest()
    self.task:update(16)

    lu.assertEquals(self.task.state, WaitForCallTask.STATE_WAITING)
    lu.assertEquals(lastDrive().what, 'stop')
end

--- Away from whoever asked. The asker is ahead of us here, so the only direction that frees the
--- spot is backwards.
function TestMakingWay:testItReversesFromSomebodyInFront()
    local asker = vehicleAt(2, 0, 20, nil)
    AutoDrive:requestMakeWay(self.parked, asker)

    self.task:update(16)

    lu.assertEquals(self.task.state, WaitForCallTask.STATE_MAKING_WAY)
    lu.assertEquals(lastDrive().what, 'reverse')
    lu.assertTrue(lastDrive().point.z < 0, 'the target has to lie away from the asker, not towards it')
end

function TestMakingWay:testItPullsForwardFromSomebodyBehind()
    local asker = vehicleAt(2, 0, -20, nil)
    AutoDrive:requestMakeWay(self.parked, asker)

    self.task:update(16)

    lu.assertEquals(lastDrive().what, 'forward')
    lu.assertTrue(lastDrive().point.z > 0)
end

function TestMakingWay:testItParksAgainOnceItIsOutOfTheWay()
    AutoDrive:requestMakeWay(self.parked, vehicleAt(2, 0, 20, nil))
    self.task:update(16)
    lu.assertEquals(self.task.state, WaitForCallTask.STATE_MAKING_WAY)

    moveTo(self.parked, 0, -AutoDrive.MAKE_WAY_DISTANCE)
    self.task:update(16)

    lu.assertEquals(self.task.state, WaitForCallTask.STATE_WAITING)
    lu.assertEquals(lastDrive().what, 'stop')
    lu.assertNil(AutoDrive.getMakeWayRequest(self.parked), 'a served request must not linger')
end

--- The point is to free the spot, not to complete the manoeuvre. Insisting on the full distance
--- would walk a vehicle that cannot get there into whatever is stopping it, over and over.
function TestMakingWay:testItGivesUpAfterTheTimeout()
    AutoDrive:requestMakeWay(self.parked, vehicleAt(2, 0, 20, nil))
    self.task:update(16)

    self.task:update(WaitForCallTask.MAKE_WAY_TIMEOUT + 1)

    lu.assertEquals(self.task.state, WaitForCallTask.STATE_WAITING)
end

--- Being called to a harvester outranks stepping aside; the request must not survive into the next
--- job, where a stale one would send the vehicle shuffling off the moment it parks again.
function TestMakingWay:testBeingCalledClearsTheRequest()
    AutoDrive:requestMakeWay(self.parked, vehicleAt(2, 0, 20, nil))
    self.task:update(16)

    self.task:finished()

    lu.assertNil(AutoDrive.getMakeWayRequest(self.parked))
end

function TestMakingWay:testAbortClearsTheRequest()
    AutoDrive:requestMakeWay(self.parked, vehicleAt(2, 0, 20, nil))
    self.task:update(16)

    self.task:abort()

    lu.assertNil(AutoDrive.getMakeWayRequest(self.parked))
end

------------------------------------------------------------------------------------------------------------------------
--- Parking clear of the way point network
---
--- Many networks have a collection route running along the inside of the field border, round to the
--- field exit. A driver that comes to rest on it blocks every full trailer heading that way, and
--- nobody has to complain for that to be true - so it checks its own spot and moves clear.
------------------------------------------------------------------------------------------------------------------------
TestParkedOnNetwork = {}

--- A collection route along +x, way points four metres apart as in a recorded one.
local function installRoute()
    ADGraphManager:load()
    local wps = {}
    for i = 1, 40 do
        wps[i] = TestSetup.waypoint(i, (i - 1) * 4, 0, i < 40 and { i + 1 } or {}, i > 1 and { i - 1 } or {})
    end
    ADGraphManager:setWayPoints(wps)
end

function TestParkedOnNetwork:setUp()
    TestSetup.reset()
    driveCalls = {}
    g_time = 10000
    -- The network check is throttled to one frame in PERF_FRAMES, phase shifted by vehicle id.
    -- Without putting the loop index on that vehicle frame the scan simply never runs and every
    -- assertion below passes or fails for the wrong reason.
    g_updateLoopIndex = AutoDrive.PERF_FRAMES - 1
    ADHarvestManager = { registerAsUnloader = function() end }
    installRoute()
    AutoDrive.testSettings['waitingPosition'] = true
    self.task = WaitForCallTask:new(vehicleAt(1, 20, 0, nil))
    self.parked = self.task.vehicle
    self.parked.ad.taskModule.activeTask = self.task
end

--- The regression: parked right on the route.
function TestParkedOnNetwork:testItMovesOffTheNetwork()
    self.task:setUp()

    self.task:update(16)

    lu.assertEquals(self.task.state, WaitForCallTask.STATE_MAKING_WAY,
        'a vehicle parked on a collection route blocks everything using it')
end

--- Well away from any route: stay put. Moving for no reason would be its own nuisance.
function TestParkedOnNetwork:testItStaysPutAwayFromTheNetwork()
    moveTo(self.parked, 20, 200)
    self.task:setUp()

    self.task:update(16)

    lu.assertEquals(self.task.state, WaitForCallTask.STATE_WAITING)
    lu.assertEquals(lastDrive().what, 'stop')
end

--- The boundary is WAITING_NETWORK_CLEARANCE.
function TestParkedOnNetwork:testTheClearanceIsTheBoundary()
    moveTo(self.parked, 20, AutoDrive.WAITING_NETWORK_CLEARANCE + 1)
    self.task:setUp()

    self.task:update(16)

    lu.assertEquals(self.task.state, WaitForCallTask.STATE_WAITING)
end

function TestParkedOnNetwork:testTheSettingTurnsItOff()
    AutoDrive.testSettings['waitingPosition'] = false
    self.task:setUp()

    self.task:update(16)

    lu.assertEquals(self.task.state, WaitForCallTask.STATE_WAITING)
    lu.assertEquals(lastDrive().what, 'stop')
end

--- Away from the route, not along it. The route runs through z = 0 and the vehicle faces +Z, so
--- backing off is the move that increases the distance.
function TestParkedOnNetwork:testItMovesAwayFromTheRoute()
    moveTo(self.parked, 20, -3)
    self.task:setUp()

    self.task:update(16)

    lu.assertEquals(lastDrive().what, 'reverse')
    lu.assertTrue(lastDrive().point.z < -3, 'the target has to lie further from the route, not nearer')
end

--- On a densely recorded field the clearance may not exist anywhere within reach. A vehicle
--- wandering the field looking for it is worse than one parked slightly awkwardly, so it gives up.
function TestParkedOnNetwork:testItGivesUpAfterTheCappedAttempts()
    self.task:setUp()

    for _ = 1, AutoDrive.WAITING_CLEARANCE_MAX_TRIES do
        self.task:update(16)
        lu.assertEquals(self.task.state, WaitForCallTask.STATE_MAKING_WAY)
        -- the move fails to help: still on the route when it finishes
        self.task:update(WaitForCallTask.MAKE_WAY_TIMEOUT + 1)
        lu.assertEquals(self.task.state, WaitForCallTask.STATE_WAITING)
    end

    self.task:update(16)

    lu.assertEquals(self.task.state, WaitForCallTask.STATE_WAITING,
        'it must stop trying rather than shuffle across the field forever')
end

--- Somebody asking outranks our own housekeeping and must not be dropped for it.
function TestParkedOnNetwork:testAnAskedForMoveStillHappensWhenTheSettingIsOff()
    AutoDrive.testSettings['waitingPosition'] = false
    self.task:setUp()
    AutoDrive:requestMakeWay(self.parked, vehicleAt(2, 20, 20, nil))

    self.task:update(16)

    lu.assertEquals(self.task.state, WaitForCallTask.STATE_MAKING_WAY)
end

------------------------------------------------------------------------------------------------------------------------
--- Stepping aside has to leave the route, not travel along it
---
--- The first version put the target on the vehicle's own axis. A driver standing on a collection
--- route is aligned with that route - it got there by driving it - so eighteen metres along its own
--- axis is eighteen metres further down the very route it is blocking. It cleared one spot and
--- landed on the next, and the clearance check fired again, and again, until its attempts ran out.
------------------------------------------------------------------------------------------------------------------------
TestSteppingOffTheRoute = {}

--- A route running along +Z, which is the direction the mock vehicles face - so a vehicle on it is
--- aligned with it, exactly as one that has just driven it would be.
local function installRouteAlongZ()
    ADGraphManager:load()
    local wps = {}
    for i = 1, 40 do
        wps[i] = TestSetup.waypoint(i, 0, (i - 1) * 4, i < 40 and { i + 1 } or {}, i > 1 and { i - 1 } or {})
    end
    ADGraphManager:setWayPoints(wps)
end

local function distanceToRoute(x, z)
    local wp = ADGraphManager:getNearestWayPointWithin({x = x, z = z}, 500)
    if wp == nil then
        return math.huge
    end
    return MathUtil.vector2Length(wp.x - x, wp.z - z)
end

function TestSteppingOffTheRoute:setUp()
    TestSetup.reset()
    driveCalls = {}
    g_time = 10000
    g_updateLoopIndex = AutoDrive.PERF_FRAMES - 1
    ADHarvestManager = { registerAsUnloader = function() end }
    installRouteAlongZ()
    AutoDrive.testSettings['waitingPosition'] = true
    -- two metres to the side of the route, aligned with it
    self.task = WaitForCallTask:new(vehicleAt(1, 2, 20, nil))
    self.parked = self.task.vehicle
    self.parked.ad.taskModule.activeTask = self.task
    self.parked.ad.stateModule.getFirstMarker = function() return { id = 400 } end
    self.task:setUp()
end

--- The regression, stated as the property that was violated: after the move the vehicle must be
--- further from the route than it started, not the same distance further along it.
function TestSteppingOffTheRoute:testTheTargetLeavesTheRoute()
    self.task:update(16)

    lu.assertEquals(self.task.state, WaitForCallTask.STATE_MAKING_WAY, 'test setup: it has to move at all')
    local target = self.task.makeWayTarget
    lu.assertNotNil(target)

    lu.assertTrue(distanceToRoute(target.x, target.z) > AutoDrive.WAITING_NETWORK_CLEARANCE,
        string.format('the target sits %.1f m from the route, which is still on it',
            distanceToRoute(target.x, target.z)))
end

--- And the direction is across the route, not along it: the route runs in Z here, so a target that
--- only moved in Z would be the old behaviour.
function TestSteppingOffTheRoute:testTheMoveIsAcrossTheRouteNotAlongIt()
    self.task:update(16)
    local target = self.task.makeWayTarget

    lu.assertTrue(math.abs(target.x - 2) > math.abs(target.z - 20),
        string.format('moved %.1f across and %.1f along; along is the route',
            math.abs(target.x - 2), math.abs(target.z - 20)))
end

--- Where the asker stands on OUR side, the escape crosses the route - and a crossing run has to be
--- long enough to be worth making. It used to be a flat eighteen metres whichever way it went, so a
--- vehicle nine metres out on one side ended nine metres out on the other: the same distance from
--- the very route it was clearing.
function TestSteppingOffTheRoute:testACrossingRunIsLongEnoughToActuallyClearTheRoute()
    moveTo(self.parked, 9.5, 20)
    self.task:setUp()
    AutoDrive.setMakeWayRequest(self.parked, 17.5, 20)   -- on our side, further out

    self.task:update(16)

    local target = self.task.makeWayTarget
    lu.assertNotNil(target)
    lu.assertTrue(target.x < 0, 'test setup: this geometry has to cross the route')
    lu.assertTrue(distanceToRoute(target.x, target.z) > AutoDrive.WAITING_NETWORK_CLEARANCE,
        string.format('the crossing target sits %.1f m from the route, which is still on it',
            distanceToRoute(target.x, target.z)))
end

--- And the manoeuvre must not report success before it gets there.
---
--- It used to end at three quarters of the distance with nothing checking where that was. On a
--- crossing run three quarters of the way lands back inside the clearance on the far side, so it
--- finished standing in exactly the state that had it asked to move in the first place - and spent
--- one of its two attempts doing it.
function TestSteppingOffTheRoute:testItDoesNotStopWhileStillOnTheRoute()
    moveTo(self.parked, 6, 20)
    self.task:setUp()
    AutoDrive.setMakeWayRequest(self.parked, 14, 20)

    self.task:update(16)
    lu.assertEquals(self.task.state, WaitForCallTask.STATE_MAKING_WAY, 'test setup: it has to move at all')
    lu.assertTrue(self.task.makeWayTarget.x < 0, 'test setup: this geometry has to cross the route')

    -- three quarters of the way across: seven and a half metres past the line, still on the network
    local threeQuarters = 6 - AutoDrive.MAKE_WAY_DISTANCE * 0.75
    moveTo(self.parked, threeQuarters, 20)
    self.task:update(16)
    lu.assertEquals(self.task.state, WaitForCallTask.STATE_MAKING_WAY, string.format(
        'it stopped %.1f m from the route, inside the %g m clearance it was moving out of',
        distanceToRoute(threeQuarters, 20), AutoDrive.WAITING_NETWORK_CLEARANCE))

    -- and it does finish once it is genuinely clear
    moveTo(self.parked, 6 - AutoDrive.MAKE_WAY_DISTANCE, 20)
    self.task:update(16)
    lu.assertEquals(self.task.state, WaitForCallTask.STATE_WAITING,
        'once it is off the route the manoeuvre is done, target reached or not')
end

--- The early finish is still an early finish where nothing is crossed. Requiring the full distance
--- everywhere would walk every vehicle four metres further than it needs to go.
function TestSteppingOffTheRoute:testARunAwayFromTheRouteStillFinishesEarly()
    moveTo(self.parked, 6, 20)
    self.task:setUp()
    AutoDrive.setMakeWayRequest(self.parked, -14, 20)    -- the far side, so our own side is the escape

    self.task:update(16)
    lu.assertTrue(self.task.makeWayTarget.x > 6, 'test setup: this geometry must not cross the route')

    moveTo(self.parked, 6 + AutoDrive.MAKE_WAY_DISTANCE * 0.75, 20)
    self.task:update(16)

    lu.assertEquals(self.task.state, WaitForCallTask.STATE_WAITING,
        'three quarters of the way out is clear of the route, and clear is what the manoeuvre is for')
end

--- Standing exactly on a way point leaves no direction to work from, so it steps across its own
--- axis instead - which for a vehicle aligned with the route is still off it.
function TestSteppingOffTheRoute:testStandingExactlyOnTheRouteStillLeavesIt()
    moveTo(self.parked, 0, 20)
    self.task:setUp()

    self.task:update(16)

    local target = self.task.makeWayTarget
    lu.assertNotNil(target, 'it has to pick some direction rather than give up')
    lu.assertTrue(distanceToRoute(target.x, target.z) > AutoDrive.WAITING_NETWORK_CLEARANCE)
end

--- The place the player sent it to is not somewhere to tidy away from. A marker IS a way point, so
--- without this the check fires on every driver that finishes its route.
function TestSteppingOffTheRoute:testItStaysOnTheMarkerThePlayerChose()
    self.parked.ad.stateModule.getFirstMarker = function() return { id = 6 } end   -- (0, 20)
    self.task:setUp()

    self.task:update(16)

    lu.assertEquals(self.task.state, WaitForCallTask.STATE_WAITING,
        'a driver parked on its own destination must be left where it was sent')
    lu.assertEquals(lastDrive().what, 'stop')
end

--- But a driver parked on the network somewhere that is NOT its marker still moves.
function TestSteppingOffTheRoute:testAwayFromItsMarkerItStillMoves()
    self.parked.ad.stateModule.getFirstMarker = function() return { id = 40 } end  -- far along the route
    self.task:setUp()

    self.task:update(16)

    lu.assertEquals(self.task.state, WaitForCallTask.STATE_MAKING_WAY)
end

--- Being asked outranks our own tidying, and the same direction rule applies to it.
function TestSteppingOffTheRoute:testAnAskedForMoveGoesAwayFromTheAsker()
    local asker = vehicleAt(2, 2, 40, nil)
    AutoDrive:requestMakeWay(self.parked, asker)

    self.task:update(16)

    local target = self.task.makeWayTarget
    lu.assertNotNil(target)
    local before = MathUtil.vector2Length(2 - 2, 20 - 40)
    local after = MathUtil.vector2Length(target.x - 2, target.z - 40)
    lu.assertTrue(after > before,
        string.format('ended up %.1f m from the asker, started %.1f m away', after, before))
end


--- The driving module advances its own stopped timer with this frame's dt. Advancing it a second
--- time made a manoeuvre that met any resistance read as stuck in half the time.
function TestSteppingOffTheRoute:testTheDrivingModuleIsAdvancedOnce()
    local updates = 0
    self.parked.ad.specialDrivingModule.update = function() updates = updates + 1 end

    self.task:update(16)
    lu.assertEquals(self.task.state, WaitForCallTask.STATE_MAKING_WAY, 'test setup: it has to be moving')
    lu.assertEquals(updates, 0, 'the driving call owns the frame while a manoeuvre is running')

    self.task:stopMakingWay()
    -- otherwise the next frame simply starts another attempt, since we are still on the route
    AutoDrive.testSettings['waitingPosition'] = false
    self.task:update(16)
    lu.assertEquals(updates, 1, 'and standing still still needs its one update')
end

--- The mode reads this to hold its stuck detection off. Meeting resistance is what a nudge out of a
--- tight spot does; the manoeuvre has its own timeout to end it.
function TestSteppingOffTheRoute:testItReportsWhenItIsManoeuvring()
    lu.assertFalse(self.task:isMakingWay())

    self.task:update(16)

    lu.assertTrue(self.task:isMakingWay())
end

function TestSteppingOffTheRoute:testItStopsReportingOnceTheManoeuvreEnds()
    self.task:update(16)
    self.task:update(WaitForCallTask.MAKE_WAY_TIMEOUT + 1)

    lu.assertFalse(self.task:isMakingWay())
end


--- Reverse is only for a target genuinely astern of the whole train.
---
--- The reverse controller measures arrival from the reverse node - the towed implement's axle, well
--- behind the root the target was measured from - and calls anything over eighty degrees off that
--- node's rear axis already reached. It then returns without commanding drive OR brake, and since
--- starting a manoeuvre releases the vehicle, a target out to the side left it rolling free for the
--- whole twenty second timeout.
function TestSteppingOffTheRoute:testALateralTargetIsDrivenToForwards()
    -- the asker is abeam, so the escape target is too
    local asker = vehicleAt(2, 20, 20, nil)
    AutoDrive:requestMakeWay(self.parked, asker)

    self.task:update(16)

    lu.assertEquals(lastDrive().what, 'forward',
        'a target out to the side has to be steered to, not reversed at')
end

--- Away from any route there is no line to cross, so the asker's own position is the direction -
--- and an asker straight ahead puts the target straight behind.
function TestSteppingOffTheRoute:testATargetDeadAsternIsReversedTo()
    AutoDrive.getTractorTrainLength = function() return 0 end
    moveTo(self.parked, 500, 500)
    local asker = vehicleAt(2, 500, 520, nil)
    AutoDrive:requestMakeWay(self.parked, asker)

    self.task:update(16)

    lu.assertEquals(lastDrive().what, 'reverse')
end

--- A train longer than the step-aside distance can never put a target far enough astern, and that
--- has to degrade to driving forwards rather than to reversing at something it will refuse.
function TestSteppingOffTheRoute:testALongTrainNeverReverses()
    AutoDrive.getTractorTrainLength = function() return 30 end
    moveTo(self.parked, 500, 500)
    local asker = vehicleAt(2, 500, 520, nil)
    AutoDrive:requestMakeWay(self.parked, asker)

    self.task:update(16)

    lu.assertEquals(lastDrive().what, 'forward')
end

--- And the backstop: whatever the geometry, a reverse controller that reports itself arrived has
--- nothing more to give, so the manoeuvre ends and the vehicle is braked again.
function TestSteppingOffTheRoute:testAnArrivedReverseEndsTheManoeuvre()
    AutoDrive.getTractorTrainLength = function() return 0 end
    moveTo(self.parked, 500, 500)
    AutoDrive:requestMakeWay(self.parked, vehicleAt(2, 500, 520, nil))
    self.task:update(16)
    lu.assertEquals(self.task.state, WaitForCallTask.STATE_MAKING_WAY)

    MockEngine.reverseReportsArrived = true
    self.task:update(16)

    lu.assertEquals(self.task.state, WaitForCallTask.STATE_WAITING,
        'nothing else would brake the vehicle for the rest of the timeout')
    lu.assertEquals(lastDrive().what, 'stop')
end


--- Every frame has to leave the vehicle with either a drive command or a brake, never neither.
--- stopMakingWay only raises the stop flag; it is specialDrivingModule:update that applies it, so
--- the frame a manoeuvre ends on has to reach that call too.
function TestSteppingOffTheRoute:testTheEndingFrameStillBrakes()
    local updates = 0
    self.parked.ad.specialDrivingModule.update = function() updates = updates + 1 end
    self.task:update(16)
    lu.assertEquals(self.task.state, WaitForCallTask.STATE_MAKING_WAY, 'test setup: it has to be moving')
    updates = 0
    driveCalls = {}

    -- the frame the timeout lands on
    self.task:update(WaitForCallTask.MAKE_WAY_TIMEOUT + 1)

    lu.assertEquals(self.task.state, WaitForCallTask.STATE_WAITING)
    lu.assertEquals(updates, 1, 'the ending frame has to apply the stop it just asked for')
    lu.assertEquals(lastDrive().what, 'stop')
end

--- Same on the other way out of the manoeuvre: having moved far enough.
function TestSteppingOffTheRoute:testTheFrameItArrivesOnStillBrakes()
    local updates = 0
    self.task:update(16)
    self.parked.ad.specialDrivingModule.update = function() updates = updates + 1 end
    moveTo(self.parked, 2 + AutoDrive.MAKE_WAY_DISTANCE, 20)

    self.task:update(16)

    lu.assertEquals(self.task.state, WaitForCallTask.STATE_WAITING)
    lu.assertEquals(updates, 1)
end


--- The case the earlier fixtures could not express.
---
--- Every direction assertion so far placed the vehicle exactly abeam a way point, so the offset from
--- that point was purely across the route BY CONSTRUCTION and any rule at all would have looked
--- right. A vehicle parked BETWEEN way points has the nearest one mostly ahead of or behind it, and
--- "away from the nearest point" is then mostly along the route - the very thing this exists to
--- stop. What has to be left is the line, so the direction is across it wherever we stand on it.
function TestSteppingOffTheRoute:testTheDirectionIsAcrossTheRouteWhereverWeStandAlongIt()
    -- way points sit at z = 0, 4, 8...; park midway between two of them and barely off the line
    for _, offsetAlong in ipairs({ 0, 1, 2, 3 }) do
        moveTo(self.parked, 1, 20 + offsetAlong)
        self.task:setUp()

        self.task:update(16)

        local target = self.task.makeWayTarget
        lu.assertNotNil(target, string.format('no manoeuvre at offset %d', offsetAlong))
        local across = math.abs(target.x - 1)
        local along = math.abs(target.z - (20 + offsetAlong))
        lu.assertTrue(across > along * 3,
            string.format('parked %d m along between way points: moved %.1f across and %.1f along',
                offsetAlong, across, along))
    end
end

--- And it must actually end up clear of the line, not merely point away from a point on it.
function TestSteppingOffTheRoute:testItEndsUpClearOfTheRouteFromAnywhereAlongIt()
    for _, offsetAlong in ipairs({ 0, 1, 2, 3 }) do
        moveTo(self.parked, 1, 20 + offsetAlong)
        self.task:setUp()

        self.task:update(16)

        local target = self.task.makeWayTarget
        lu.assertTrue(distanceToRoute(target.x, target.z) > AutoDrive.WAITING_NETWORK_CLEARANCE,
            string.format('offset %d: target still %.1f m from the route',
                offsetAlong, distanceToRoute(target.x, target.z)))
    end
end

--- A request arriving mid-manoeuvre used to be accepted, never read, and then deleted by
--- stopMakingWay - so the vehicle that sent it had spent its one ask per stuck episode on nothing,
--- and fell through to reversing itself out fifteen seconds later.
function TestSteppingOffTheRoute:testARequestArrivingMidManoeuvreSurvivesIt()
    self.task:update(16)
    lu.assertEquals(self.task.state, WaitForCallTask.STATE_MAKING_WAY, 'test setup: it has to be moving')

    lu.assertTrue(AutoDrive:requestMakeWay(self.parked, vehicleAt(2, 2, 40, nil)),
        'a vehicle that is manoeuvring can still be asked, it just cannot act yet')

    self.task:update(WaitForCallTask.MAKE_WAY_TIMEOUT + 1)

    lu.assertEquals(self.task.state, WaitForCallTask.STATE_WAITING)
    lu.assertNotNil(AutoDrive.getMakeWayRequest(self.parked),
        'the newer request has never been read and must not be thrown away unread')
end

function TestSteppingOffTheRoute:testTheServedRequestIsStillCleared()
    self.task:update(16)
    local served = AutoDrive.getMakeWayRequest(self.parked)
    lu.assertNotNil(served)

    self.task:update(WaitForCallTask.MAKE_WAY_TIMEOUT + 1)

    lu.assertNil(AutoDrive.getMakeWayRequest(self.parked),
        'the one it actually served has to go, or it repeats forever')
end


--- Driving the task with BOTH clocks moving, which nothing did before.
---
--- The engine gives code two notions of time: the dt handed to update, and the g_time timestamps are
--- taken from. Every test here drove the timeout by passing a large dt while g_time stayed at its
--- setUp value, so anything measured in g_time was invisible - and a request expiring in g_time
--- while the manoeuvre timed out in dt went unnoticed through several rounds of review.
local function run(task, milliseconds, step)
    step = step or 100
    local elapsed = 0
    while elapsed < milliseconds do
        task:update(MockEngine.advance(step))
        elapsed = elapsed + step
    end
end

--- The request that arrives mid-manoeuvre has to still be there when the manoeuvre ends, however
--- long that takes. It is only read on a waiting frame, so it has to outlive the whole timeout.
function TestSteppingOffTheRoute:testAMidManoeuvreRequestOutlivesTheWholeTimeout()
    self.task:update(MockEngine.advance(16))
    lu.assertEquals(self.task.state, WaitForCallTask.STATE_MAKING_WAY, 'test setup: it has to be moving')

    -- an ask landing early in an obstructed manoeuvre is the worst case for the expiry
    run(self.task, 1000)
    lu.assertTrue(AutoDrive:requestMakeWay(self.parked, vehicleAt(2, 2, 40, nil)))
    local theAsk = self.parked.ad.makeWayRequest

    run(self.task, WaitForCallTask.MAKE_WAY_TIMEOUT + 500)

    -- the manoeuvre in flight ended and the waiting frame after it picked the ask up, which is the
    -- whole point: it is acted on rather than deleted unread
    lu.assertIs(self.task.servedRequest, theAsk,
        'the ask was accepted, never read during the first manoeuvre, and has to be served after it')
end

--- The request the manoeuvre actually served still has to go, or it repeats forever.
function TestSteppingOffTheRoute:testTheServedRequestGoesEvenWithTheClockRunning()
    self.task:update(MockEngine.advance(16))
    local served = self.parked.ad.makeWayRequest
    lu.assertNotNil(served)

    -- long enough for both clearance attempts to run out, so nothing new starts afterwards
    run(self.task, (WaitForCallTask.MAKE_WAY_TIMEOUT + 500) * AutoDrive.WAITING_CLEARANCE_MAX_TRIES)

    lu.assertEquals(self.task.state, WaitForCallTask.STATE_WAITING)
    lu.assertNotEquals(self.parked.ad.makeWayRequest, served,
        'the request it served has to go, or the same manoeuvre repeats forever')
end

--- A request has to outlive a manoeuvre by construction, not by luck of the timings.
function TestSteppingOffTheRoute:testTheRequestOutlivesTheManoeuvreByConstruction()
    lu.assertTrue(AutoDrive.MAKE_WAY_VALID_TIME > WaitForCallTask.MAKE_WAY_TIMEOUT,
        string.format('a request lives %d ms and a manoeuvre runs %d ms, so one can expire unread',
            AutoDrive.MAKE_WAY_VALID_TIME, WaitForCallTask.MAKE_WAY_TIMEOUT))
end

--- Stepping aside towards the vehicle that asked frees nothing and stalls on our own front sensor.
--- Which side of the route the two happen to be on must not decide it.
function TestSteppingOffTheRoute:testItStepsAwayFromTheAskerWhicheverSideItIsOn()
    for _, askerLateral in ipairs({ -8, -3, 3, 8 }) do
        moveTo(self.parked, 2, 20)
        self.task:setUp()
        local asker = vehicleAt(2, askerLateral, 20, nil)
        AutoDrive:requestMakeWay(self.parked, asker)

        self.task:update(16)

        local target = self.task.makeWayTarget
        lu.assertNotNil(target, string.format('no manoeuvre with the asker at %g', askerLateral))
        local before = MathUtil.vector2Length(askerLateral - 2, 0)
        local after = MathUtil.vector2Length(askerLateral - target.x, 20 - target.z)
        lu.assertTrue(after > before, string.format(
            'asker at %g: closed from %.1f m to %.1f m', askerLateral, before, after))
    end
end


os.exit(lu.LuaUnit.run())
