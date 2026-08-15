--[[
Regression test for the pathfinder completion gate in PathFinderModule:update.

The bug this pins down was introduced by the fix for A3 and found in the game, not here.

A3 was: dubinsDone meant two different things - "a Dubins path was accepted" and "gave up on
Dubins" - and the second meaning forced smoothDone, throwing away an A* path that had just been
found. The fix narrowed the gate to "dubinsDone AND wayPoints exist".

That left a hole. When NEITHER Dubins nor the A* produced anything, nothing sets smoothDone any
more, so update() takes the "isFinished" branch and returns on every following frame without ever
finishing. self.steps stops climbing, the task keeps reporting "Berechne Pfad 1 / 2 - 2 / 1200",
and the driver stands still forever. The old code terminated here - badly, by discarding a path,
but it terminated.

So the rule is: a finished run must always reach smoothDone, whether it found something or not.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('SortedQueue')
require('Dubins')
require('PathFinderUtils')
require('AutoDriveTON')
require('Scheduler')
require('PathFinderModule')

--- A module in the state update() sees after the search has ended.
local function finishedModule(overrides)
    local o = setmetatable({}, { __index = PathFinderModule })
    o.vehicle = TestSetup.vehicle()
    o.vehicle.ad.pathFinderModule = o
    -- update() calls abort() when startCell is nil, which would mask everything below
    o.startCell = { x = 0, z = 0 }
    o.targetCell = { x = 5, z = 0 }
    o.isFinished = true
    o.smoothDone = false
    o.dubinsDone = false
    o.dubinsAborted = false
    o.delayTime = 0
    o.steps = 2
    o.max_pathfinder_steps = 1200
    o.path = {}
    o.wayPoints = {}
    o.grid = {}
    o.isNewPF = true
    o.smoothStep = 0
    if overrides then
        for k, v in pairs(overrides) do o[k] = v end
    end
    return o
end

TestCompletion = {}

function TestCompletion:setUp()
    TestSetup.reset()
    ADScheduler:load()
end

--- The exact situation from the game: finished, gave up on Dubins, no A* path either.
function TestCompletion:testFinishedRunWithNothingFoundTerminates()
    local pf = finishedModule({ dubinsDone = false, dubinsAborted = true })

    -- Drive update() the way the game does. A correct module settles within a few frames; a
    -- hanging one never does, so cap it rather than looping forever inside the test.
    for _ = 1, 50 do
        pf:update(16)
        if pf.smoothDone then break end
    end

    lu.assertTrue(pf.smoothDone,
        'a finished run that found nothing must still complete - otherwise update() returns early '
        .. 'on every frame, self.steps freezes and the driver reports "calculating path" forever')
end

function TestCompletion:testStepCounterDoesNotFreeze()
    local pf = finishedModule({ dubinsAborted = true })
    local stepsBefore = pf.steps
    for _ = 1, 20 do
        pf:update(16)
    end
    lu.assertTrue(pf.smoothDone,
        'the run must have completed rather than sitting at step ' .. tostring(stepsBefore))
end

--- The A3 fix itself must not regress: a real path may never be discarded.
function TestCompletion:testAnAcceptedDubinsPathIsKept()
    local pf = finishedModule({
        dubinsDone = true,
        wayPoints = { { x = 0, y = 0, z = 0 }, { x = 1, y = 0, z = 1 } },
    })
    pf:update(16)
    lu.assertTrue(pf.smoothDone)
    lu.assertEquals(#pf.wayPoints, 2, 'an accepted Dubins path must survive completion')
end

--- And giving up on Dubins must not shortcut an A* path that does exist.
function TestCompletion:testAnAStarPathIsStillBuiltAfterDubinsGaveUp()
    local pf = finishedModule({
        dubinsDone = false,
        dubinsAborted = true,
        path = { { x = 0, z = 0 }, { x = 1, z = 0 }, { x = 2, z = 0 } },
    })
    pf.gridLocationToWorldLocation = function(_, cell)
        return { x = cell.x, y = 0, z = cell.z }
    end

    for _ = 1, 50 do
        pf:update(16)
        if pf.smoothDone then break end
    end

    lu.assertTrue(pf.smoothDone, 'the run must complete')
    lu.assertTrue(#pf.wayPoints > 0,
        'the A* path found after Dubins gave up must be turned into waypoints - that was the '
        .. 'whole point of the A3 fix')
end

function TestCompletion:testAbortAlwaysTerminates()
    local pf = finishedModule()
    pf:abort()
    lu.assertTrue(pf.smoothDone)
    lu.assertTrue(pf.isFinished)
    lu.assertEquals(#pf.wayPoints, 0)
end

os.exit(lu.LuaUnit.run())
