--[[
Leaving the field when the way out is occupied.

ExitFieldTask decides between two things: plan a path off the field, or declare the job already done
because the road network is within a driver radius. The shortcut is right for a vehicle that can
drive and wrong for one that has just proved it cannot.

Measured in game across a whole session, both full unloaders queued behind a stationary harvester:
zero pathfinder runs between them. Every ExitFieldTask ended on the shortcut, handed the vehicle to
the network follower, and the follower drove it back into the machine in front. The stuck recovery
reversed each of them out three times; each time they rejoined at the NEAREST way point, which lies
on the very field course they had just been queued on, and drove straight back into the queue.
]]

lu = require('luaunit')
require('test-setup')
require('AbstractTask')
require('AutoDriveTON')
require('ExitFieldTask')

-- The graph manager is a whole subsystem this task only reads two functions from, so it is
-- stubbed rather than loaded.
ADGraphManager = ADGraphManager or {}

--- Everything ExitFieldTask:startPathPlanning reaches for, and a record of what it did.
local function exiting(mustPlan, routeLength, closestDistance)
    local record = { planned = 0, target = nil, vector = nil, finished = 0, aborted = 0 }

    local route = {}
    for i = 1, routeLength do
        route[i] = { id = i, x = i * 10, y = 0, z = 0 }
    end

    local vehicle = TestSetup.vehicle({ id = 5, components = { { node = 'exitV' } } })
    MockEngine.nodePositions['exitV'] = { x = 0, y = 0, z = 0 }
    vehicle.getClosestWayPoint = function() return 1, closestDistance end
    vehicle.ad.stateModule = {
        getSecondWayPoint = function() return routeLength end,
        getFirstWayPoint = function() return 1 end,
        getName = function() return 'driver' end,
    }
    vehicle.ad.pathFinderModule = {
        reset = function() end,
        startPathPlanningTo = function(_, target, vector)
            record.planned = record.planned + 1
            record.target = target
            record.vector = vector
        end,
    }
    vehicle.ad.taskModule = {
        abortAllTasks = function() record.aborted = record.aborted + 1 end,
        addTask = function() end,
        setCurrentTaskFinished = function() record.finished = record.finished + 1 end,
    }

    ADGraphManager.getWayPointById = function(_, id) return route[id] end
    ADGraphManager.pathFromTo = function(_, from, to)
        local out = {}
        for i = from, to do out[#out + 1] = route[i] end
        return out
    end
    AutoDrive.getDriverRadius = function() return 10 end
    -- the pathfinder's own precondition, in metres: it refuses a target under three grid
    -- cells and the task must not aim at one
    AutoDrive.minPlannableDistance = function() return 30 end
    AutoDrive.getSetting = function(name) if name == 'exitField' then return ExitFieldTask.STRATEGY_CLOSEST end return 0 end

    local task = ExitFieldTask:new(vehicle, mustPlan)
    task.nextExitStrategy = ExitFieldTask.STRATEGY_CLOSEST
    return task, record, route
end

TestExitField = {}

function TestExitField:setUp()
    TestSetup.reset()
    self.saved = {
        byId = ADGraphManager.getWayPointById,
        pathFromTo = ADGraphManager.pathFromTo,
        radius = AutoDrive.getDriverRadius,
        setting = AutoDrive.getSetting,
        minPlan = AutoDrive.minPlannableDistance,
    }
end

function TestExitField:tearDown()
    ADGraphManager.getWayPointById = self.saved.byId
    ADGraphManager.pathFromTo = self.saved.pathFromTo
    AutoDrive.getDriverRadius = self.saved.radius
    AutoDrive.getSetting = self.saved.setting
    AutoDrive.minPlannableDistance = self.saved.minPlan
end

--- Unchanged: a driver that is genuinely already on the network does not plan anything.
function TestExitField:testAnOrdinaryExitCloseToTheNetworkStillShortCircuits()
    local task, record = exiting(false, 20, 3)

    task:startPathPlanning()

    lu.assertEquals(record.planned, 0)
    lu.assertEquals(record.finished, 1, 'already on the network is already out of the field')
end

--- Unchanged: far from the network it plans, as it always did.
function TestExitField:testAnOrdinaryExitFarFromTheNetworkPlans()
    local task, record = exiting(false, 20, 40)

    task:startPathPlanning()

    lu.assertEquals(record.planned, 1)
end

--- The fix. A vehicle just recovered from a standstill plans even though the network is right there,
--- because being near a way point says nothing about whether it can drive.
function TestExitField:testARecoveredVehiclePlansEvenWhenTheNetworkIsRightThere()
    local task, record = exiting(true, 20, 3)

    task:startPathPlanning()

    lu.assertEquals(record.planned, 1, 'the shortcut is what made the recovery pointless')
    lu.assertEquals(record.finished, 0)
end

--- And it rejoins far enough away that the search will actually run.
---
--- setupNew refuses a target under three grid cells and returns completelyBlocked before its first
--- expansion, so aiming nearer is not a hard search - it is a guaranteed failure, and this task
--- answers failure by planning again. Measured: 527 searches by one vehicle, every one with the
--- identical start and target one cell apart, every one refused before it began, zero paths found.
function TestExitField:testARecoveredVehicleAimsPastThePathfindersMinimum()
    local task, record, route = exiting(true, 20, 3)

    task:startPathPlanning()

    -- way points sit at x = 10, 20, 30 ... and the minimum is 30 m
    lu.assertEquals(record.target, route[3], 'the first point the search is allowed to be given')
    lu.assertAlmostEquals(record.vector.x, route[4].x - route[3].x, 0.001,
        'and the heading has to be the one at the point it actually aims for')
end

--- Nearer points are skipped, however many of them there are.
function TestExitField:testPointsInsideTheMinimumAreSkipped()
    local task, record, route = exiting(true, 20, 3)
    AutoDrive.minPlannableDistance = function() return 75 end

    task:startPathPlanning()

    lu.assertEquals(record.target, route[8], 'x = 80 is the first at or past 75 m')
end

--- And a route entirely inside the minimum is not planned at all. There is nothing to plan and
--- nothing to retry - which is the difference between finishing and looping five hundred times.
function TestExitField:testARouteShorterThanThePathfinderCanPlanIsJustDriven()
    local task, record = exiting(true, 3, 3)

    task:startPathPlanning()

    lu.assertEquals(record.planned, 0, 'a search that cannot run must not be started')
    lu.assertEquals(record.finished, 1)
end

--- The flag is off by default, so every caller that does not ask for it is untouched.
function TestExitField:testPlanningIsNotForcedByDefault()
    local vehicle = TestSetup.vehicle({ id = 6 })

    lu.assertFalse(ExitFieldTask:new(vehicle).mustPlan)
    lu.assertFalse(ExitFieldTask:new(vehicle, false).mustPlan)
    lu.assertTrue(ExitFieldTask:new(vehicle, true).mustPlan)
end

os.exit(lu.LuaUnit.run())
