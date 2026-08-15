--[[
Regression test: do not drive at the chase point while the harvester is turning away from us.

Found in the game. The unloader drove into the JAGUAR from behind during its turn.

Measured in the log of that run: 419 followChasePoint calls in STATE_WAIT_FOR_TURN, all with
chaseSide 3 (CHASEPOS_REAR), and getAngleToChasePos steady at 177 degrees. The harvester had swung
around, dragging its rear chase position BEHIND the unloader - so "drive towards the chase point"
meant turning into a harvester that was swinging back at the same time.

waitForChasePos does not protect against this: it is only recomputed in the chasing state, so
during the turn it keeps whatever value it had when the turn began.

This is pre-existing behaviour, not a regression - our diff in that branch was one debug guard.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('AutoDriveTON')
require('FollowCombineTask')

local function task(angleToChasePos)
    local o = setmetatable({}, { __index = FollowCombineTask })
    o.vehicle = TestSetup.vehicle()
    o.getAngleToChasePos = function() return angleToChasePos end
    return o
end

TestTurnChaseGuard = {}

function TestTurnChaseGuard:setUp() TestSetup.reset() end

function TestTurnChaseGuard:testTheMeasuredFailureAngleIsRefused()
    lu.assertFalse(task(177):isChasePointReachableDuringTurn(),
        '177 degrees is the angle measured while the unloader drove into the harvester - the chase '
        .. 'point was directly behind it')
end

function TestTurnChaseGuard:testAChasePointAheadIsStillFollowed()
    lu.assertTrue(task(0):isChasePointReachableDuringTurn())
    lu.assertTrue(task(30):isChasePointReachableDuringTurn(),
        'a harvester turning slightly away must not stop the unloader dead - that would make it '
        .. 'lose contact on every headland')
    lu.assertTrue(task(89):isChasePointReachableDuringTurn())
end

function TestTurnChaseGuard:testTheBoundaryIsInclusive()
    lu.assertTrue(task(90):isChasePointReachableDuringTurn())
    lu.assertFalse(task(91):isChasePointReachableDuringTurn())
end

function TestTurnChaseGuard:testSidewaysIsTheCutOff()
    -- the rule in one sentence: anything no longer in front of us is refused
    for _, angle in ipairs({ 95, 120, 150, 180 }) do
        lu.assertFalse(task(angle):isChasePointReachableDuringTurn(),
            angle .. ' degrees is behind the unloader and must not be driven to during a turn')
    end
end

--- Source level: both followChasePoint calls in the turn branch must sit behind the guard, or the
--- drive command is issued before anything checks the angle.
function TestTurnChaseGuard:testBothTurnBranchesAreGuarded()
    local src = io.open('../../scripts/Tasks/FollowCombineTask.lua', 'r'):read('*a')
    local turnBranch = src:match('STATE_WAIT_FOR_TURN then(.-)elseif self%.state == FollowCombineTask%.STATE_REVERSING')
    lu.assertNotNil(turnBranch, 'could not isolate the STATE_WAIT_FOR_TURN branch')

    local calls = select(2, turnBranch:gsub('self:followChasePoint%(dt%)', ''))
    local guards = select(2, turnBranch:gsub('isChasePointReachableDuringTurn%(%)', ''))
    lu.assertTrue(calls > 0, 'expected followChasePoint calls in the turn branch')
    lu.assertEquals(guards, calls,
        string.format('%d followChasePoint call(s) in the turn branch but %d guard(s)', calls, guards))
end

os.exit(lu.LuaUnit.run())
