--[[
Tests for scripts/Manager/Scheduler.lua concurrency.

Covers finding:
  P5 only one vehicle was ever allowed to search for a path; with five unloaders the last one
     could wait ~15 s before its search even started.

The property that must hold: adding searchers must not add per-frame work. The step budget is
split between them, so two searchers cost the same per frame as one.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('AutoDriveTON')
require('Scheduler')

local function vehicleWithPathfinder()
    local v = TestSetup.vehicle()
    v.ad.pathFinderModule = {
        delay = nil,
        addDelayTimer = function(self, ms) self.delay = ms end,
    }
    return v
end

TestConcurrency = {}

function TestConcurrency:setUp()
    TestSetup.reset()
    ADScheduler:load()
end

------------------------------------------------------------------------------------------------------------------------
--- How many searchers the measured frame rate allows
------------------------------------------------------------------------------------------------------------------------
function TestConcurrency:testUnmeasuredFrameRateBehavesLikeBefore()
    ADScheduler.actual_fps = 0
    lu.assertEquals(ADScheduler:getMaxConcurrentPathfinders(), 1,
        'with nothing measured the scheduler must not get more adventurous than it used to be')
end

function TestConcurrency:testLowFrameRateKeepsASingleSearcher()
    ADScheduler.actual_fps = 22
    lu.assertEquals(ADScheduler:getMaxConcurrentPathfinders(), 1)
end

function TestConcurrency:testHealthyFrameRateAllowsTwo()
    ADScheduler.actual_fps = 45
    lu.assertEquals(ADScheduler:getMaxConcurrentPathfinders(), 2)
end

function TestConcurrency:testHighFrameRateAllowsTheMaximum()
    ADScheduler.actual_fps = 75
    lu.assertEquals(ADScheduler:getMaxConcurrentPathfinders(),
        ADScheduler.MAX_CONCURRENT_PATHFINDERS)
end

------------------------------------------------------------------------------------------------------------------------
--- The budget is split, not multiplied
------------------------------------------------------------------------------------------------------------------------
function TestConcurrency:testStepBudgetIsSharedBetweenSearchers()
    ADScheduler.stepsPerFrame = 8

    ADScheduler.activeCount = 1
    local oneSearcher = ADScheduler:getStepsPerFrame()

    ADScheduler.activeCount = 2
    local twoSearchers = ADScheduler:getStepsPerFrame()

    lu.assertEquals(oneSearcher, 8)
    lu.assertEquals(twoSearchers, 4,
        'two searchers must each get half the budget, so the frame cost stays the same')
    lu.assertEquals(twoSearchers * 2, oneSearcher)
end

function TestConcurrency:testEachSearcherKeepsTheMinimumBudget()
    ADScheduler.stepsPerFrame = 2
    ADScheduler.activeCount = 3
    lu.assertEquals(ADScheduler:getStepsPerFrame(), ADScheduler.MIN_STEPS_PER_FRAME,
        'splitting must never starve a searcher below the minimum step count')
end

function TestConcurrency:testActiveCountIsNeverZero()
    ADScheduler.stepsPerFrame = 8
    ADScheduler.activeCount = 0
    lu.assertEquals(ADScheduler:getStepsPerFrame(), 8)
end

------------------------------------------------------------------------------------------------------------------------
--- Activation
------------------------------------------------------------------------------------------------------------------------
function TestConcurrency:testSeveralVehiclesAreReleasedAtOnce()
    ADScheduler.actual_fps = 75   -- allows MAX_CONCURRENT_PATHFINDERS
    local vehicles = {}
    for i = 1, 5 do
        vehicles[i] = vehicleWithPathfinder()
        ADScheduler:addPathfinderVehicle(vehicles[i])
    end

    ADScheduler:updateActiveVehicle()

    local released = 0
    for _, v in ipairs(vehicles) do
        if v.ad.pathFinderModule.delay == 0 then released = released + 1 end
    end
    lu.assertEquals(released, ADScheduler.MAX_CONCURRENT_PATHFINDERS,
        'the frame rate allows three searchers, so three must actually be released')
    lu.assertEquals(ADScheduler.activeCount, ADScheduler.MAX_CONCURRENT_PATHFINDERS)
end

function TestConcurrency:testLowFrameRateStillReleasesOnlyOne()
    ADScheduler.actual_fps = 21
    local vehicles = {}
    for i = 1, 4 do
        vehicles[i] = vehicleWithPathfinder()
        ADScheduler:addPathfinderVehicle(vehicles[i])
    end

    ADScheduler:updateActiveVehicle()

    local released = 0
    for _, v in ipairs(vehicles) do
        if v.ad.pathFinderModule.delay == 0 then released = released + 1 end
    end
    lu.assertEquals(released, 1, 'a struggling frame rate must fall back to the old behaviour')
end

function TestConcurrency:testTheFirstReleasedVehicleHoldsTheRotationSlot()
    ADScheduler.actual_fps = 75
    local a, b = vehicleWithPathfinder(), vehicleWithPathfinder()
    ADScheduler:addPathfinderVehicle(a)
    ADScheduler:addPathfinderVehicle(b)

    ADScheduler:updateActiveVehicle()
    lu.assertEquals(ADScheduler.activePathFinderVehicle, a,
        'the rotation has to keep working, so the head of the queue owns the active slot')
end

function TestConcurrency:testEmptyQueueIsHarmless()
    ADScheduler.actual_fps = 75
    ADScheduler:updateActiveVehicle()
    lu.assertEquals(ADScheduler.activeCount, 1)
    lu.assertNil(ADScheduler.activePathFinderVehicle)
end

os.exit(lu.LuaUnit.run())
