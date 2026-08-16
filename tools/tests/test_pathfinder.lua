--[[
Tests for scripts/Modules/PathFinderModule.lua

Covers findings:
  A3  dubinsDone had two meanings, giving up on Dubins threw away the A* path
  A4  cell.incomming typo skipped slope/water and vehicle checks for every Dubins sample
  A16 the drivability test ran before the closed set was consulted
  A17 pop_best_node scanned the whole open set linearly
  A18 getDubinsPath evaluated all candidate curves within one frame
  A39 self.fruitAreas grew for the whole session and was never read
  A40 fallback mode 1 was a no-op because the field restriction stayed active
  A41 dead duplicatePointDirection branch in testNextCells
  B8  MAX_PATHFINDER_STEPS_PER_FRAME was defined but never read

PathFinderModule loads standalone - the file only assigns to its own table - so the behaviour is
tested for real wherever the function under test can be reached with a handful of stubs. The
module is however tightly coupled to the vehicle and to the game, so a few functions are only
reachable with an instance built by hand. Where a fix is a pure removal (a dead constant, a dead
branch, dead state) the source itself is asserted on, the way Courseplay's
AutoDriveInterfaceTest.lua pins an interface.
]]

lu = require('luaunit')
require('test-setup')

require('Dubins')
require('PathFinderModule')

local SOURCE_PATH = '../../scripts/Modules/PathFinderModule.lua'

local function readSource()
    local f = io.open(SOURCE_PATH, 'r')
    if not f then
        error('test_pathfinder: cannot read ' .. SOURCE_PATH .. ' - run tests from tools/tests')
    end
    local src = f:read('*a')
    f:close()
    return src
end

--- The body of one module function, so a contract can be asserted on the right function instead
--- of on the whole file.
local function sourceOfFunction(name)
    local src = readSource()
    local startPos = src:find('function PathFinderModule:' .. name .. '%(')
    lu.assertNotNil(startPos, 'function PathFinderModule:' .. name .. ' not found')
    local endPos = src:find('\nfunction PathFinderModule[:%.]', startPos + 1) or #src
    return src:sub(startPos, endPos)
end

------------------------------------------------------------------------------------------------------------------------
--- Environment
------------------------------------------------------------------------------------------------------------------------
--- The engine functions PathFinderModule reaches for that the shared mock does not provide.
function netGetTime() return 0 end

--- The scheduler paces the pathfinder. Only the step budget matters here.
ADScheduler = {
    stepsPerFrame = 2,
    getStepsPerFrame = function(self) return self.stepsPerFrame end,
    addPathfinderVehicle = function() end,
    removePathfinderVehicle = function() end,
}

--- Reinstalls the AutoDrive functions the module calls. TestSetup.reset() wipes the mock state
--- between tests, the stubs are cheap enough to just rebuild every time.
local function installAutoDriveStubs()
    AutoDrive.debugPrint = function() end
    AutoDrive.debugMsg = function(_, text, ...) Logging.info(text, ...) end
    AutoDrive.isEditorModeEnabled = function() return false end
    AutoDrive.getDebugChannelIsSet = function() return false end
    AutoDrive.getDriverRadius = function() return 7 end
    AutoDrive.normalizeAngle = function(angle)
        while angle > 2 * math.pi do angle = angle - 2 * math.pi end
        while angle < 0 do angle = angle + 2 * math.pi end
        return angle
    end
    AutoDrive.checkIsOnField = function() return true end
    AutoDrive.checkForVehiclePathInBox = function() return false end
    AutoDrive.getFruitValue = function() return 0 end
end

--- A vehicle with the size the module reads (size.height is mandatory, reset() would fail on nil).
local function testVehicle()
    return TestSetup.vehicle({ size = { width = 3, length = 6, height = 4, lengthOffset = 0 } })
end

--- An instance that inherits the module methods. The direction constants and the Dubins state are
--- taken from the module itself by running its own reset(), so the test can never assert against
--- a stale copy of them.
local function newPF(pathfinderSetting)
    local pf = setmetatable({}, { __index = PathFinderModule })
    pf.vehicle = testVehicle()
    AutoDrive.testSettings['Pathfinder'] = pathfinderSetting or 1
    PathFinderModule.reset(pf)
    pf.delayTime = 0
    pf.startCell = { x = 0, z = 0, direction = pf.PP_UP, steps = 0, bordercells = 0 }
    pf.minTurnRadius = 7
    pf.vehicleMinHeight = 5
    pf.targetVector = { x = 1, z = 0.001 }
    pf.vectorX = { x = 7, z = 0 }
    pf.vectorZ = { x = 0, z = 7 }
    pf.target = { x = 0, z = 0 }
    pf.targetCell = { x = 0, z = 0, direction = pf.PP_UP }
    return pf
end

------------------------------------------------------------------------------------------------------------------------
--- A3 - the Dubins shortcut must not swallow the A* result
------------------------------------------------------------------------------------------------------------------------
TestDubinsHandover = {}

function TestDubinsHandover:setUp()
    TestSetup.reset()
    installAutoDriveStubs()
end

--- Finished with a path from the A* while the Dubins shortcut had given up: the path has to be
--- turned into wayPoints. Before the fix both meanings shared one flag, smoothDone was forced to
--- true and the caller got an empty path plus "Could not calculate path".
function TestDubinsHandover:testAbortedDubinsStillBuildsWayPointsFromAstarPath()
    local pf = newPF()
    pf.isNewPF = true
    pf.isFinished = true
    pf.smoothDone = false
    pf.wayPoints = {}
    pf.dubinsDone = false
    pf.dubinsAborted = true
    local created = false
    pf.createWayPointsNew = function(this)
        created = true
        this.wayPoints = { { x = 1, y = 0, z = 1 } }
        this.smoothDone = true
    end

    PathFinderModule.update(pf, 16)

    lu.assertTrue(created, 'giving up on Dubins must not skip the wayPoint creation')
    lu.assertEquals(#pf.wayPoints, 1)
end

--- The narrow meaning still has to work: an accepted Dubins path already carries its wayPoints
--- and must not be smoothed a second time.
function TestDubinsHandover:testAcceptedDubinsPathIsNotRebuilt()
    local pf = newPF()
    pf.isNewPF = true
    pf.isFinished = true
    pf.smoothDone = false
    pf.wayPoints = { { x = 1, y = 0, z = 1 }, { x = 2, y = 0, z = 2 } }
    pf.dubinsDone = true
    pf.dubinsAborted = false
    local created = false
    pf.createWayPointsNew = function() created = true end

    PathFinderModule.update(pf, 16)

    lu.assertFalse(created, 'the wayPoints of an accepted Dubins path must be kept')
    lu.assertTrue(pf.smoothDone)
    lu.assertEquals(#pf.wayPoints, 2)
end

--- CORRECTED after a finding in the game.
---
--- This used to assert that autoRestart clears the whole Dubins state, i.e. "a restarted search
--- must try Dubins again". That assumption is what made the pathfinder loop: update() gives up on
--- Dubins after 4 fruitless sweeps, autoRestart runs on EVERY fallback, and clearing dubinsAborted
--- there released the brake before it could ever engage. Measured in a real session: 352 fallbacks
--- and 433 completed sweeps for 15 path requests, with the unloader standing still.
---
--- A fallback relaxes the field and fruit restrictions. It does not move the start or the target,
--- so the geometry Dubins failed on is unchanged and a repeat sweep fails the same way.
function TestDubinsHandover:testAutoRestartKeepsTheGiveUpDecision()
    local pf = newPF()
    pf.grid = {}
    pf.dubinsAborted = true
    pf.dubinsCount = 5

    PathFinderModule.autoRestart(pf)

    lu.assertTrue(pf.dubinsAborted,
        'a fallback restart must not re-enable Dubins, or the give-up brake never engages')
    lu.assertTrue(pf.dubinsCount >= 5,
        'the sweep counter has to keep accumulating across fallbacks')
end

function TestDubinsHandover:testAutoRestartKeepsAnAcceptedDubinsPath()
    local pf = newPF()
    pf.grid = {}
    pf.dubinsDone = true

    PathFinderModule.autoRestart(pf)

    lu.assertTrue(pf.dubinsDone, 'an accepted Dubins result must survive a restart')
end

--- CORRECTED after the sweep was found never completing in the game.
---
--- The resume state must SURVIVE a restart. getDubinsPath works on its own dubinsNodes and never
--- reads self.grid, so there is nothing for a rebuild to invalidate. Clearing it made every
--- fallback restart the sweep at candidate 0: 918 starts, 918 budget interruptions, zero
--- completions, and the give-up brake could never engage because dubinsCount never rose.
function TestDubinsHandover:testAutoRestartKeepsAnInterruptedSweep()
    local pf = newPF()
    pf.grid = {}
    pf.dubinsCandidate = 4
    pf.dubinsSampleIndex = 17
    pf.dubinsPending = true

    PathFinderModule.autoRestart(pf)

    lu.assertEquals(pf.dubinsCandidate, 4,
        'an interrupted sweep must resume, otherwise it can never reach its last candidate')
    lu.assertEquals(pf.dubinsSampleIndex, 17)
    lu.assertTrue(pf.dubinsPending)
end

--- A genuinely new request is different: there everything starts fresh. startPathPlanningTo cannot
--- be driven without most of the game, so its reset is pinned on the source.
function TestDubinsHandover:testStartPathPlanningToResetsTheDubinsState()
    lu.assertStrContains(sourceOfFunction('startPathPlanningTo'), 'self:resetDubins()',
        'a new path request must clear dubinsDone / dubinsAborted / dubinsCount')
end

--- And resetDubins itself still has to do the full clear, since that is what a new request uses.
function TestDubinsHandover:testResetDubinsClearsEverything()
    local pf = newPF()
    pf.dubinsDone = true
    pf.dubinsAborted = true
    pf.dubinsCount = 9

    PathFinderModule.resetDubins(pf)

    lu.assertFalse(pf.dubinsDone)
    lu.assertFalse(pf.dubinsAborted)
    lu.assertEquals(pf.dubinsCount, 0)
end

------------------------------------------------------------------------------------------------------------------------
--- A17 - the open set is a heap now, and it still pops the cheapest node
------------------------------------------------------------------------------------------------------------------------
TestOpenSetQueue = {}

function TestOpenSetQueue:setUp()
    TestSetup.reset()
    installAutoDriveStubs()
    self.pf = newPF()
    self.pf.openset = {}
    self.pf.openHeap = {}
    self.pf.f_score = {}
end

local function pushNode(pf, x, score)
    local node = { x = x, z = 0, cost = 0 }
    pf.f_score[node] = score
    pf:push_open_node(node)
    return node
end

function TestOpenSetQueue:testPopsInScoreOrder()
    local pf = self.pf
    local scores = { 5, 1, 9, 3, 3, 7, 0.5, 12, 2, 8, 4 }
    for i, s in ipairs(scores) do
        pushNode(pf, i, s)
    end

    local popped = {}
    while next(pf.openset) do
        local node = pf:pop_best_node(pf.openset, pf.f_score)
        lu.assertNotNil(node, 'a filled open set must always yield a node')
        table.insert(popped, pf.f_score[node])
    end

    local expected = {}
    for i, s in ipairs(scores) do expected[i] = s end
    table.sort(expected)
    lu.assertEquals(popped, expected)
end

--- A neighbour that is reached on a cheaper route is pushed again with its new score. The old
--- entry must not resurface, and the node must be handed out exactly once.
function TestOpenSetQueue:testCheaperScoreWinsAndOutdatedEntriesAreDropped()
    local pf = self.pf
    local a = pushNode(pf, 1, 10)
    local b = pushNode(pf, 2, 5)
    pf.f_score[a] = 1
    pf:push_open_node(a)

    lu.assertIs(pf:pop_best_node(pf.openset, pf.f_score), a)
    lu.assertIs(pf:pop_best_node(pf.openset, pf.f_score), b)
    lu.assertNil(pf:pop_best_node(pf.openset, pf.f_score),
        'the outdated heap entry of the improved node must not come back')
end

function TestOpenSetQueue:testEmptySetYieldsNothing()
    lu.assertNil(self.pf:pop_best_node(self.pf.openset, self.pf.f_score))
end

--- Safety net: whatever happens to the heap, a filled open set must never leave the A* loop
--- without a node to expand.
function TestOpenSetQueue:testFallsBackToTheScanWhenTheHeapIsEmpty()
    local pf = self.pf
    local cheap = { x = 1, z = 0 }
    local expensive = { x = 2, z = 0 }
    pf.f_score[cheap] = 2
    pf.f_score[expensive] = 8
    pf.openset[cheap] = true
    pf.openset[expensive] = true

    lu.assertIs(pf:pop_best_node(pf.openset, pf.f_score), cheap)
    lu.assertIs(pf:pop_best_node(pf.openset, pf.f_score), expensive)
end

------------------------------------------------------------------------------------------------------------------------
--- A16 - closed nodes are recognised before the expensive drivability test
------------------------------------------------------------------------------------------------------------------------
TestAstarExpansion = {}

function TestAstarExpansion:setUp()
    TestSetup.reset()
    installAutoDriveStubs()
end

--- An instance sitting right before the first A* expansion of the start node.
local function astarPF()
    local pf = newPF()
    pf.isNewPF = true
    pf.isFinished = false
    pf.initNew = true
    pf.completelyBlocked = false
    pf.targetBlocked = false
    pf.steps = 0
    pf.max_pathfinder_steps = 400
    pf.diffOverallNetTime = 0
    -- far apart, so update() goes straight to the A* instead of trying Dubins
    pf.start = { x = 0, z = 0 }
    pf.target = { x = 200, z = 0 }
    pf.cachedNodes = {}
    pf.openset, pf.openHeap, pf.closedset = {}, {}, {}
    pf.came_from, pf.g_score, pf.h_score, pf.f_score = {}, {}, {}, {}

    pf.nodeBehindStart = pf:get_node(-1, 0)
    pf.nodeBehindStart.direction = pf.PP_UP
    pf.nodeStart = pf:get_node(0, 0)
    pf.nodeStart.direction = pf.PP_UP
    pf.nodeGoal = pf:get_node(30, 0)
    pf.nodeGoal.direction = pf.PP_UP

    pf.g_score[pf.nodeStart] = 0
    pf.h_score[pf.nodeStart] = 0
    pf.f_score[pf.nodeStart] = 0
    pf.came_from[pf.nodeStart] = pf.nodeBehindStart
    pf.closedset[pf.nodeBehindStart] = true
    pf:push_open_node(pf.nodeStart)
    return pf
end

function TestAstarExpansion:testClosedNeighbourIsNeverDrivabilityTested()
    local pf = astarPF()
    local tested = {}
    pf.isDriveableAstar = function(_, cell)
        tested[cell] = (tested[cell] or 0) + 1
        return true
    end
    -- one of the three cells the start node expands into is already closed
    local closedCell = pf:get_node(1, 1)
    pf.closedset[closedCell] = true

    PathFinderModule.update(pf, 16)

    lu.assertNil(tested[closedCell],
        'a closed node must be recognised before the drivability test is paid for')
    lu.assertNotNil(tested[pf:get_node(1, 0)], 'open neighbours are still tested')
    lu.assertNotNil(tested[pf:get_node(1, -1)])
    lu.assertEquals(Logging.countMatching('ERROR'), 0)
end

function TestAstarExpansion:testUndrivableNeighboursAreNotAddedToTheOpenSet()
    local pf = astarPF()
    pf.isDriveableAstar = function() return false end

    PathFinderModule.update(pf, 16)

    lu.assertTrue(pf.completelyBlocked, 'without a drivable neighbour the search runs dry')
    lu.assertNil(next(pf.openset))
end

------------------------------------------------------------------------------------------------------------------------
--- A18 / A4 - the Dubins sweep is paced, and the samples know where they come from
------------------------------------------------------------------------------------------------------------------------
TestDubinsSweep = {}

function TestDubinsSweep:setUp()
    TestSetup.reset()
    installAutoDriveStubs()
    ADScheduler.stepsPerFrame = 2
end

function TestDubinsSweep:tearDown()
    ADScheduler.stepsPerFrame = 2
end

--- An instance with a real Dubins solver and a straight 20 m hop in front of the vehicle.
local function dubinsPF()
    local pf = newPF()
    pf.dubins = ADDubins:new()
    pf.dubinsNodes = {}
    pf.q0 = { 0, 0, 0 }
    pf.q1 = { 20, 0, 0 }
    pf.restrictToField = false
    pf.avoidFruitSetting = false
    pf.fallBackMode3 = false
    return pf
end

--- Runs getDubinsPath until the sweep is over, returning the path and the number of frames used.
local function runSweep(pf)
    local path, frames = nil, 0
    for _ = 1, 200 do
        frames = frames + 1
        path = pf:getDubinsPath()
        if path ~= nil or not pf.dubinsPending then
            break
        end
    end
    return path, frames
end

--- The sample of every candidate is a full drivability test. It must be spread over frames with
--- the scheduler budget instead of being paid for in one spike.
function TestDubinsSweep:testSweepRespectsTheFrameBudget()
    local pf = dubinsPF()
    local budget = ADScheduler:getStepsPerFrame() * PathFinderModule.NEW_PF_STEP_FACTOR
    local checks, perFrame, maxPerFrame = 0, 0, 0
    pf.isDriveableDubins = function()
        checks = checks + 1
        perFrame = perFrame + 1
        return true
    end

    local path, frames = nil, 0
    for _ = 1, 200 do
        perFrame = 0
        frames = frames + 1
        path = pf:getDubinsPath()
        maxPerFrame = math.max(maxPerFrame, perFrame)
        if path ~= nil or not pf.dubinsPending then
            break
        end
    end

    lu.assertNotNil(path, 'a drivable straight hop must produce a Dubins path')
    lu.assertTrue(#path > budget,
        'the fixture must be long enough to exceed one frame budget, got ' .. tostring(#path))
    lu.assertTrue(frames > 1, 'a path longer than the budget has to be spread over frames')
    lu.assertTrue(maxPerFrame <= budget,
        'at most ' .. tostring(budget) .. ' drivability tests per frame, got ' .. tostring(maxPerFrame))
    lu.assertEquals(checks, #path, 'a resumed sweep must not retest samples it already checked')
end

--- The budget must not eat the tries: the sweep is one try no matter over how many frames it ran.
function TestDubinsSweep:testPendingSweepDoesNotCountAsAFailedTry()
    local pf = dubinsPF()
    pf.isNewPF = true
    pf.isFinished = false
    pf.initNew = true
    pf.completelyBlocked = false
    pf.targetBlocked = false
    pf.steps = 0
    pf.max_pathfinder_steps = 400
    pf.start = { x = 0, z = 0 }
    pf.target = { x = 10, z = 0 }   -- close enough for the Dubins shortcut
    pf.isDriveableDubins = function() return true end
    -- update() falls through to the A* in the same frame, an empty open set ends it right away
    pf.openset, pf.openHeap, pf.closedset = {}, {}, {}
    pf.came_from, pf.g_score, pf.h_score, pf.f_score = {}, {}, {}, {}

    PathFinderModule.update(pf, 16)

    lu.assertTrue(pf.dubinsPending, 'the fixture is meant to run out of budget in the first frame')
    lu.assertEquals(pf.dubinsCount, 0, 'an unfinished sweep must not use up one of the five tries')
    lu.assertFalse(pf.dubinsAborted)
end

--- The pacing must not change which curve is chosen: the shortest path is still tried first.
function TestDubinsSweep:testShortestPathStillWins()
    local pf = dubinsPF()
    ADScheduler.stepsPerFrame = 100
    pf.isDriveableDubins = function() return true end

    local reference = ADDubins:new()
    reference.outPath = {}
    lu.assertEquals(reference:dubins_shortest_path(ADDubins.DubinsPath, pf.q0, pf.q1, pf.minTurnRadius),
        ADDubins.EDUBOK)
    reference:dubins_path_sample_many(ADDubins.DubinsPath, 1, reference.createWayPoints)

    local path = runSweep(pf)

    lu.assertNotNil(path)
    lu.assertEquals(#path, #reference.outPath)
    for i, point in ipairs(path) do
        lu.assertAlmostEquals(point.x, reference.outPath[i].x, 1e-6)
        lu.assertAlmostEquals(point.z, reference.outPath[i].z, 1e-6)
    end
end

--- When the first candidates are not drivable the sweep walks the candidates in the same order as
--- before and returns the curve of the first one that is.
function TestDubinsSweep:testCandidatesAreTriedInOrder()
    local pf = dubinsPF()
    ADScheduler.stepsPerFrame = 100
    local seen, winner = {}, nil
    pf.isDriveableDubins = function(this)
        if seen[#seen] ~= this.dubinsCandidate then
            table.insert(seen, this.dubinsCandidate)
        end
        winner = this.dubinsCandidate
        return this.dubinsCandidate >= 3
    end

    local path = runSweep(pf)

    lu.assertNotNil(path, 'one of the single word curves must be accepted')
    lu.assertEquals(seen[1], 0, 'the shortest path is candidate 0 and is tried first')
    for i = 2, #seen do
        lu.assertTrue(seen[i] > seen[i - 1], 'candidates must be walked in ascending order')
    end
    lu.assertTrue(winner >= 3)

    local reference = ADDubins:new()
    reference.outPath = {}
    lu.assertEquals(reference:dubins_path(ADDubins.DubinsPath, pf.q0, pf.q1, pf.minTurnRadius, winner),
        ADDubins.EDUBOK)
    reference:dubins_path_sample_many(ADDubins.DubinsPath, 1, reference.createWayPoints)
    lu.assertEquals(#path, #reference.outPath)
    for i, point in ipairs(path) do
        lu.assertAlmostEquals(point.x, reference.outPath[i].x, 1e-6)
    end
end

--- A4: the sample cells are chained through 'incoming'. The typo 'incomming' left the field unset,
--- which silently skipped the slope/water check and the other vehicle check for every sample.
function TestDubinsSweep:testSamplesAreChainedThroughIncoming()
    local pf = dubinsPF()
    ADScheduler.stepsPerFrame = 100
    local samples, chained, typo = 0, 0, false
    pf.isDriveableDubins = function(_, cell)
        samples = samples + 1
        if cell.incoming ~= nil then chained = chained + 1 end
        if cell.incomming ~= nil then typo = true end
        return true
    end

    local path = runSweep(pf)

    lu.assertNotNil(path)
    lu.assertTrue(samples > 1)
    lu.assertEquals(chained, samples - 1, 'every sample but the first knows its predecessor')
    lu.assertFalse(typo, 'the misspelled field must not be written any more')
end

--- The consequence of A4, with the real drivability test: the checks that depend on 'incoming'
--- have to run for the Dubins samples.
function TestDubinsSweep:testSlopeAndVehicleChecksRunForDubinsSamples()
    local pf = dubinsPF()
    ADScheduler.stepsPerFrame = 100
    local slopeChecks, vehicleChecks = 0, 0
    pf.checkSlopeAngle = function()
        slopeChecks = slopeChecks + 1
        return false, 0
    end
    AutoDrive.checkForVehiclePathInBox = function()
        vehicleChecks = vehicleChecks + 1
        return false
    end

    local path = runSweep(pf)

    lu.assertNotNil(path)
    lu.assertTrue(slopeChecks > 0, 'the slope / water check was skipped for every Dubins sample')
    lu.assertTrue(vehicleChecks > 0, 'the other vehicle check was skipped for every Dubins sample')
end

------------------------------------------------------------------------------------------------------------------------
--- A40 - fallback mode 1 really widens the search beyond the field border
------------------------------------------------------------------------------------------------------------------------
TestFieldBorderFallback = {}

function TestFieldBorderFallback:setUp()
    TestSetup.reset()
    installAutoDriveStubs()
    AutoDrive.checkIsOnField = function() return false end   -- the whole cell area is off field
    self.pf = newPF(2)                                       -- the legacy pathfinder owns this code
    self.pf.grid = {}
    self.pf.restrictToField = true
    self.pf.avoidFruitSetting = false
    self.pf.checkSlopeAngle = function() return false, 0 end
end

local function offFieldCell(pf)
    return {
        x = 1, z = 0, direction = pf.PP_UP, steps = 1, bordercells = 0,
        isRestricted = false, hasCollision = false, hasFruit = false,
        incoming = { x = 0, z = 0, direction = pf.PP_UP, steps = 0, bordercells = 0 },
    }
end

function TestFieldBorderFallback:testWithoutFallbackTheFieldRestrictionApplies()
    local pf = self.pf
    pf.fallBackMode1 = false
    pf.fallBackMode2 = false
    local cell = offFieldCell(pf)

    pf:checkGridCell(cell)

    lu.assertTrue(cell.isRestricted, 'restrictToField keeps the search on the field')
end

--- The point of fallback 1: cells off the field become usable, limited by the bordercell counter
--- (findClosestCell drops everything beyond MAX_FIELDBORDER_CELLS) instead of by the restriction.
function TestFieldBorderFallback:testFallbackOneAllowsBorderCells()
    local pf = self.pf
    pf.fallBackMode1 = true
    pf.fallBackMode2 = false
    local cell = offFieldCell(pf)

    pf:checkGridCell(cell)

    lu.assertFalse(cell.isRestricted,
        'fallback 1 must lift the field restriction, otherwise it searches the same area again')
    lu.assertEquals(cell.bordercells, 1, 'the cell counts as one cell beyond the field border')
end

function TestFieldBorderFallback:testFallbackTwoDropsEveryFieldLimit()
    local pf = self.pf
    pf.fallBackMode1 = true
    pf.fallBackMode2 = true
    local cell = offFieldCell(pf)

    pf:checkGridCell(cell)

    lu.assertFalse(cell.isRestricted)
    lu.assertEquals(cell.bordercells, 0, 'fallback 2 stops counting border cells altogether')
end

--- The border allowance must stay usable, i.e. below the limit the cell survives findClosestCell.
function TestFieldBorderFallback:testBorderCellsBelowTheLimitAreStillReachable()
    local pf = self.pf
    pf.fallBackMode1 = true
    pf.fallBackMode2 = false
    local cell = offFieldCell(pf)
    pf:checkGridCell(cell)

    lu.assertTrue(cell.bordercells < PathFinderModule.MAX_FIELDBORDER_CELLS)
    lu.assertIs(pf:findClosestCell({ cell }, math.huge), cell)
end

------------------------------------------------------------------------------------------------------------------------
--- A41 - every newly created cell is checked, the dead shortcut branch is gone
------------------------------------------------------------------------------------------------------------------------
TestTestNextCells = {}

function TestTestNextCells:setUp()
    TestSetup.reset()
    installAutoDriveStubs()
    self.pf = newPF(2)
    self.pf.grid = {}
end

local function outCell(pf, x, z, direction)
    return { x = x, z = z, direction = direction, steps = 1, bordercells = 0,
             isRestricted = false, hasCollision = false, hasFruit = false }
end

function TestTestNextCells:testNewCellsAreChecked()
    local pf = self.pf
    local checked = {}
    pf.checkGridCell = function(_, cell) table.insert(checked, cell) end
    local cell = { x = 0, z = 0, direction = pf.PP_UP, steps = 0, bordercells = 0, out = {
        outCell(pf, 1, -1, pf.PP_UP_LEFT), outCell(pf, 1, 0, pf.PP_UP), outCell(pf, 1, 1, pf.PP_UP_RIGHT),
    } }

    pf:testNextCells(cell)

    lu.assertEquals(#checked, 3)
    lu.assertTrue(cell.visited)
    for _, out in ipairs(cell.out) do
        lu.assertIs(pf.grid[PathFinderModule.gridKeyOf(out.x, out.z, out.direction)], out)
    end
end

--- A cell that is already in the grid with another direction gets a full check as a new cell:
--- coming from another direction means another collision box. The deleted branch copied the old
--- result instead, but it could never run because its flag was never set.
function TestTestNextCells:testSameCellWithAnotherDirectionIsCheckedAgain()
    local pf = self.pf
    local known = outCell(pf, 1, 0, pf.PP_UP)
    pf.grid[PathFinderModule.gridKeyOf(known.x, known.z, known.direction)] = known
    local checked = {}
    pf.checkGridCell = function(_, cell) table.insert(checked, cell) end
    local fromOther = outCell(pf, 1, 0, pf.PP_UP_RIGHT)
    local cell = { x = 0, z = 0, direction = pf.PP_UP, steps = 0, bordercells = 0, out = { fromOther } }

    pf:testNextCells(cell)

    lu.assertEquals(#checked, 1)
    lu.assertIs(checked[1], fromOther)
    lu.assertIs(pf.grid[PathFinderModule.gridKeyOf(1, 0, pf.PP_UP_RIGHT)], fromOther)
    lu.assertIs(pf.grid[PathFinderModule.gridKeyOf(1, 0, pf.PP_UP)], known, 'the known cell stays')
end

function TestTestNextCells:testKnownCellWithSameDirectionIsNotRecreated()
    local pf = self.pf
    local known = outCell(pf, 1, 0, pf.PP_UP)
    known.steps = 9
    pf.grid[PathFinderModule.gridKeyOf(known.x, known.z, known.direction)] = known
    local checked = 0
    pf.checkGridCell = function() checked = checked + 1 end
    local cell = { x = 0, z = 0, direction = pf.PP_UP, steps = 0, bordercells = 0,
                   out = { outCell(pf, 1, 0, pf.PP_UP) } }

    pf:testNextCells(cell)

    lu.assertEquals(checked, 0)
    lu.assertEquals(known.steps, 1, 'the shortcut through the cheaper cell is still taken')
    lu.assertIs(known.incoming, cell)
end

------------------------------------------------------------------------------------------------------------------------
--- Removals: dead constant (B8), dead state (A39), dead branch (A41)
------------------------------------------------------------------------------------------------------------------------
TestDeadCodeIsGone = {}

function TestDeadCodeIsGone:setUp()
    TestSetup.reset()
    installAutoDriveStubs()
end

function TestDeadCodeIsGone:testStepsPerFrameConstantIsGone()
    lu.assertNil(PathFinderModule.MAX_PATHFINDER_STEPS_PER_FRAME,
        'the steps per frame come from ADScheduler:getStepsPerFrame(), the constant was never read')
    lu.assertNotStrContains(readSource(), 'MAX_PATHFINDER_STEPS_PER_FRAME',
        'the header comment must not document the removed constant either')
    lu.assertNotStrContains(readSource(), '294000',
        'the search area derived from the removed constant must be gone')
end

function TestDeadCodeIsGone:testHeaderDocumentsTheSchedulerRange()
    local header = readSource():sub(1, 3000)
    lu.assertStrContains(header, 'ADScheduler.MIN_STEPS_PER_FRAME')
    lu.assertStrContains(header, 'ADScheduler.MAX_STEPS_PER_FRAME')
end

function TestDeadCodeIsGone:testFruitAreasAccumulationIsGone()
    lu.assertNotStrContains(readSource(), 'fruitAreas',
        'the corner tables were collected for the whole session and never read')
end

function TestDeadCodeIsGone:testDuplicatePointDirectionBranchIsGone()
    lu.assertNotStrContains(readSource(), 'duplicatePointDirection',
        'the branch could never run, its only assignment was commented out')
end

--- The remaining steps-per-frame users must all go through the scheduler.
function TestDeadCodeIsGone:testPacingGoesThroughTheScheduler()
    lu.assertStrContains(sourceOfFunction('getDubinsPath'), 'ADScheduler:getStepsPerFrame()',
        'the Dubins sweep must be paced like the rest of the module')
end

os.exit(lu.LuaUnit.run())
