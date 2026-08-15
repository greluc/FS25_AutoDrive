--[[
C2: a Courseplay harvester that cannot reverse out of its turn asks the unloader to move.

Courseplay's checkBlockingUnloader looks for a Courseplay drive strategy on the blocking vehicle
and calls requestToBackupForReversingCombine on it. An AutoDrive unloader has no such strategy, so
the request reached nobody: the harvester could not reverse, the unloader never learned it was in
the way, and the pair sat there until something else timed out.

AutoDrive:requestBackupForReversingCombine records the request; FollowCombineTask acts on it.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('AutoDriveTON')
require('FollowCombineTask')

local function task(state, isChopper)
    local o = setmetatable({}, { __index = FollowCombineTask })
    o.vehicle = TestSetup.vehicle()
    o.combine = TestSetup.vehicle()
    o.combine.ad.isChopper = isChopper or false
    o.state = state or FollowCombineTask.STATE_CHASING
    o.getStateName = function() return 'test' end
    FollowCombineTask.setStateNames(o)
    return o
end

TestBackupRequest = {}

function TestBackupRequest:setUp()
    TestSetup.reset()
    g_time = 10000
end

function TestBackupRequest:testNoRequestMeansNothingToDo()
    local t = task()
    lu.assertFalse(t:hasPendingBackupRequest())
end

function TestBackupRequest:testAFreshRequestIsPending()
    local t = task()
    t.vehicle.ad.reverseForCombineRequest = g_time
    lu.assertTrue(t:hasPendingBackupRequest())
end

--- A stale request must not make the unloader retreat from a harvester that has driven off.
function TestBackupRequest:testAStaleRequestIsDropped()
    local t = task()
    t.vehicle.ad.reverseForCombineRequest = g_time - 10000
    lu.assertFalse(t:hasPendingBackupRequest())
    lu.assertNil(t.vehicle.ad.reverseForCombineRequest, 'a stale request must be cleared, not kept')
end

function TestBackupRequest:testTheHarvesterGetsRoom()
    local t = task(FollowCombineTask.STATE_CHASING)
    t.vehicle.ad.reverseForCombineRequest = g_time

    t:startBackupForReversingCombine()

    lu.assertEquals(t.state, FollowCombineTask.STATE_REVERSING,
        'the unloader must actually back away, or the harvester stays blocked')
    lu.assertNotNil(t.reverseStartLocation, 'reversing needs a reference point')
    lu.assertEquals(t.reverseDistance, FollowCombineTask.RETREAT_DISTANCE,
        'a short retreat is enough - this is not the "give up and leave" case')
    lu.assertTrue(t.resumeChasingAfterReverse,
        'after making room the unloader must pick the chase back up')
end

function TestBackupRequest:testAChopperUsesItsOwnReverseState()
    local t = task(FollowCombineTask.STATE_CHASING, true)
    t.vehicle.ad.reverseForCombineRequest = g_time
    t:startBackupForReversingCombine()
    lu.assertEquals(t.state, FollowCombineTask.STATE_REVERSING_FROM_CHOPPER)
end

function TestBackupRequest:testTheRequestIsConsumedOnce()
    local t = task()
    t.vehicle.ad.reverseForCombineRequest = g_time
    t:startBackupForReversingCombine()
    lu.assertNil(t.vehicle.ad.reverseForCombineRequest,
        'the request must be consumed, or every following frame restarts the retreat')
end

--- Already moving away: a request must not restart the manoeuvre from scratch each frame.
function TestBackupRequest:testAlreadyReversingIsLeftAlone()
    for _, state in ipairs({ FollowCombineTask.STATE_REVERSING,
                             FollowCombineTask.STATE_REVERSING_FROM_CHOPPER }) do
        local t = task(state)
        t.vehicle.ad.reverseForCombineRequest = g_time
        t.reverseStartLocation = { x = 1, y = 0, z = 1 }

        t:startBackupForReversingCombine()

        lu.assertEquals(t.state, state, 'we are already getting out of the way')
        lu.assertEquals(t.reverseStartLocation.x, 1, 'the running retreat must not be restarted')
    end
end

os.exit(lu.LuaUnit.run())
