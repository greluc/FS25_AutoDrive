--[[
Regression test: the "give up on Dubins" brake must survive a fallback restart.

Found in the game. The unloader stood at the field edge reporting "Drescher Entladebereich
blockiert" while the log showed 352 fallbacks and 433 completed Dubins sweeps for 15 path
requests - the pathfinder was looping.

Mechanism: update() gives up on Dubins after 4 fruitless sweeps (dubinsCount > 4 -> dubinsAborted).
autoRestart(), which runs on EVERY fallback, called resetDubins() and cleared exactly those two
fields. So the brake was released before it could ever engage:

    Dubins sweeps 4x -> dubinsAborted -> A* runs -> step budget exhausted -> fallback
      -> autoRestart -> resetDubins -> dubinsAborted = false -> Dubins sweeps again -> ...

A fallback relaxes the field and fruit restrictions. It does not move the start or the target, so
the Dubins geometry is unchanged and a sweep that just failed will fail again.
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

local function moduleInRestart()
    local o = setmetatable({}, { __index = PathFinderModule })
    o.vehicle = TestSetup.vehicle()
    o.startCell = { x = 0, z = 0, direction = 0 }
    o.targetCell = { x = 5, z = 0, direction = 0 }
    o.grid = {}
    o.path = {}
    o.wayPoints = {}
    o.steps = 50
    o.dubinsDone = false
    o.dubinsAborted = false
    o.dubinsCount = 0
    o.dubinsPending = false
    o.dubinsCandidate = nil
    o.dubinsSampleIndex = nil
    return o
end

TestDubinsBrake = {}

function TestDubinsBrake:setUp()
    TestSetup.reset()
    ADScheduler:load()
end

function TestDubinsBrake:testFallbackRestartKeepsTheGiveUpDecision()
    local pf = moduleInRestart()
    pf.dubinsAborted = true       -- the brake has engaged
    pf.dubinsCount = 5

    pf:autoRestart()

    lu.assertTrue(pf.dubinsAborted,
        'a fallback restart must not re-enable Dubins - the geometry it failed on is unchanged, '
        .. 'and clearing this is what made the pathfinder loop')
    lu.assertTrue(pf.dubinsCount >= 5,
        'the sweep counter must keep accumulating across fallbacks, otherwise the brake never engages')
end

function TestDubinsBrake:testFallbackRestartKeepsAnAcceptedDubinsPath()
    local pf = moduleInRestart()
    pf.dubinsDone = true
    pf:autoRestart()
    lu.assertTrue(pf.dubinsDone, 'an accepted Dubins result must not be forgotten by a restart')
end

--- CORRECTED after a second finding in the game.
---
--- This first asserted that autoRestart must CLEAR the resume state, on the assumption that an
--- interrupted sweep would continue into a grid that autoRestart rebuilds. getDubinsPath works on
--- its own dubinsNodes / dubinsFromCell and never reads self.grid, so there was nothing to
--- protect - and clearing it meant a sweep interrupted by the frame budget restarted from
--- candidate 0 on every fallback. Measured: 918 sweep starts, 918 budget interruptions, zero
--- completions. The sweep could never finish, so dubinsCount never rose and the give-up brake
--- never engaged: the same loop as before, reached by a different route.
function TestDubinsBrake:testFallbackRestartResumesAnInterruptedSweep()
    local pf = moduleInRestart()
    pf.dubinsCandidate = 3
    pf.dubinsSampleIndex = 17
    pf.dubinsPending = true

    pf:autoRestart()

    lu.assertEquals(pf.dubinsCandidate, 3,
        'an interrupted sweep must continue where it left off, or it never reaches the end')
    lu.assertEquals(pf.dubinsSampleIndex, 17)
    lu.assertTrue(pf.dubinsPending)
end

--- The property that actually matters: a sweep spread over several frames, with a fallback
--- restart in between, must still be able to complete.
function TestDubinsBrake:testASweepSurvivesAFallbackAndCanComplete()
    local pf = moduleInRestart()

    -- frame 1: budget runs out half way through the sweep
    pf.dubinsCandidate = 2
    pf.dubinsSampleIndex = 9
    pf.dubinsPending = true

    pf:autoRestart()          -- the A* fell back in the same frame

    -- frame 2: the sweep continues and reaches the last candidate
    lu.assertNotNil(pf.dubinsCandidate, 'progress must survive the restart')
    pf.dubinsCandidate = 6
    pf.dubinsSampleIndex = nil
    pf.dubinsPending = false

    pf:autoRestart()

    lu.assertEquals(pf.dubinsCandidate, 6,
        'a sweep that has almost finished must not be thrown away by another fallback')
end

--- A genuinely new request is a different matter: there the whole state must be fresh.
function TestDubinsBrake:testAFreshRequestDoesResetEverything()
    local pf = moduleInRestart()
    pf.dubinsAborted = true
    pf.dubinsCount = 9
    pf.dubinsDone = true

    pf:resetDubins()

    lu.assertFalse(pf.dubinsAborted, 'a new path request starts with a clean slate')
    lu.assertEquals(pf.dubinsCount, 0)
    lu.assertFalse(pf.dubinsDone)
end

--- The brake itself: four fruitless sweeps and Dubins is done for this request.
function TestDubinsBrake:testBrakeEngagesAfterFourSweeps()
    local pf = moduleInRestart()
    for _ = 1, 4 do
        pf.dubinsCount = pf.dubinsCount + 1
        if pf.fallBackMode3 or pf.dubinsCount > 4 then
            pf.dubinsAborted = true
        end
        pf:autoRestart()   -- a fallback between every sweep, as in the game
    end
    pf.dubinsCount = pf.dubinsCount + 1
    if pf.dubinsCount > 4 then pf.dubinsAborted = true end

    lu.assertTrue(pf.dubinsAborted,
        'five sweeps with a fallback between each must still reach the give-up decision')
end

os.exit(lu.LuaUnit.run())
