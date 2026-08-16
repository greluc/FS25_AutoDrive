--[[
Clearing out of the crop after finishing with a harvester.

The task starts in STATE_WAITING and waits ten seconds before it drives anywhere. For that whole
window it issued no command to either driving module - no brake, no drive, and no
specialDrivingModule:update, which is what applies a stop. The task handing over can leave the
vehicle released and rolling: the chase ends with driveToPoint, and driveToPoint releases. So the
HUD said "waiting for room" while nothing at all was holding the vehicle, on every run of the task,
which for a chopper unloader is every time it finishes.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('AutoDriveTON')
require('AbstractTask')
require('ClearCropTask')

local commands

local function waitingTask()
    local o = setmetatable({}, { __index = ClearCropTask })
    o.vehicle = TestSetup.vehicle()
    o.state = ClearCropTask.STATE_WAITING
    o.waitTimer = AutoDriveTON:new()
    o.driveTimer = AutoDriveTON:new()
    o.stuckTimer = AutoDriveTON:new()
    o.wayPoints = {}
    ClearCropTask.setStateNames(o)

    commands = { stop = 0, update = 0, path = 0 }
    o.vehicle.ad.specialDrivingModule = {
        stopVehicle = function() commands.stop = commands.stop + 1 end,
        update = function() commands.update = commands.update + 1 end,
        releaseVehicle = function() end,
    }
    o.vehicle.ad.drivePathModule = {
        update = function() commands.path = commands.path + 1 end,
        setWayPoints = function() end,
        isTargetReached = function() return false end,
    }
    return o
end

TestClearCropWait = {}

function TestClearCropWait:setUp()
    TestSetup.reset()
end

--- Every frame of the wait has to reach the vehicle with something.
function TestClearCropWait:testTheWaitBrakesTheVehicle()
    local task = waitingTask()

    for _ = 1, 30 do
        task:update(16)
    end

    lu.assertEquals(task.state, ClearCropTask.STATE_WAITING, 'test setup: still inside the wait')
    lu.assertEquals(commands.stop, 30,
        'the previous task can hand over with the vehicle released and rolling, so the wait has to '
        .. 'hold it rather than assume somebody else did')
    lu.assertEquals(commands.update, 30, 'and apply that stop in the same frame')
end

--- And the wait still ends when it is meant to.
function TestClearCropWait:testItStillLeavesTheWaitAfterTheFullTime()
    local task = waitingTask()

    task:update(ClearCropTask.WAIT_TIME + 1)

    lu.assertEquals(task.state, ClearCropTask.STATE_CLEARING_FIRST)
end

os.exit(lu.LuaUnit.run())
