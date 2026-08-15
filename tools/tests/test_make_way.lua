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
        reverseToTargetLocation = function(_, _, point) driveCalls[#driveCalls + 1] = { v = id, what = 'reverse', point = point } end,
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

os.exit(lu.LuaUnit.run())
