--[[
The speed ADSpecialDrivingModule:driveToPoint actually asks for.

It takes two speed arguments: maxFollowSpeed, the pace to hold once alongside the target, and
maxSpeed, the cap. It also clamps to the field speed limit. Then it adds a catch-up boost over the
last twenty metres, which is how an unloader closes on a moving harvester.

That boost used to be ASSIGNED over the computed speed rather than clamped against it, so inside
twenty metres both the caller's cap and the field limit were gone. Three of the four callers pass
the same number for both arguments - the bale nudge asks for 1 km/h, the reverse recovery and the
make-way manoeuvre for 8 - and were getting up to 44.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('AutoDriveTON')
require('SpecialDrivingModule')

------------------------------------------------------------------------------------------------------------------------
local commandedSpeed

local function moduleAt(x, z, fieldLimit)
    local m = ADSpecialDrivingModule:new(TestSetup.vehicle())
    local v = m.vehicle
    v.components = { { node = 'veh' } }
    MockEngine.nodePositions['veh'] = { x = x, y = 0, z = z }
    v.lastSpeedReal = 0
    v.ad.stateModule = { getFieldSpeedLimit = function() return fieldLimit or 50 end }
    v.ad.trailerModule = { handleTrailerReversing = function() end }
    v.ad.specialDrivingModule = m
    v.ad.collisionDetectionModule = { hasDetectedObstable = function() return false end }
    v.ad.sensors = { frontSensor = { pollInfo = function() return false end } }
    return m
end

--- driveInDirection is where the speed finally lands, so that is where it is read. Replaced for the
--- whole file rather than per test: restoring it at file scope would run before luaunit does.
AutoDrive.driveInDirection = function(_, _, _, _, _, _, _, _, _, _, speed)
    commandedSpeed = speed
end
AutoDrive.getDriveDirection = function() return 0, 1 end

local function speedTowards(module, targetX, targetZ, maxFollowSpeed, maxSpeed)
    commandedSpeed = nil
    module:driveToPoint(16, { x = targetX, y = 0, z = targetZ }, maxFollowSpeed, false, 0.5, maxSpeed)
    return commandedSpeed
end

------------------------------------------------------------------------------------------------------------------------
TestDriveToPointSpeed = {}

function TestDriveToPointSpeed:setUp()
    TestSetup.reset()
    MockEngine.nodePositions['veh'] = { x = 0, y = 0, z = 0 }
end

--- The regression, in the terms the make-way manoeuvre uses: a parked vehicle asked to shuffle
--- eighteen metres out of somebody's way, at eight km/h.
function TestDriveToPointSpeed:testTheCapHoldsInsideTheBoostEnvelope()
    local speed = speedTowards(moduleAt(0, 0), 0, 18, 8, 8)

    lu.assertTrue(speed <= 8,
        string.format('asked for at most 8 km/h eighteen metres out, got %.1f', speed))
end

--- A one km/h nudge is a nudge at every distance.
function TestDriveToPointSpeed:testASlowNudgeStaysSlow()
    for _, distance in ipairs({ 0.3, 3, 10, 19 }) do
        local speed = speedTowards(moduleAt(0, 0), 0, distance, 1, 1)
        lu.assertTrue(speed <= 1,
            string.format('at %.1f m a 1 km/h nudge commanded %.1f', distance, speed))
    end
end

--- The field limit is a limit too, and was being discarded by the same line.
function TestDriveToPointSpeed:testTheFieldLimitHolds()
    local speed = speedTowards(moduleAt(0, 0, 6), 0, 18, 8, 40)

    lu.assertTrue(speed <= 6, string.format('field limit is 6, commanded %.1f', speed))
end

--- The boost itself has to survive: it is how an unloader catches a harvester that is driving away
--- from it. With a cap above the boost, the boost is what governs.
function TestDriveToPointSpeed:testTheCatchUpBoostStillWorks()
    local near = speedTowards(moduleAt(0, 0), 0, 3, 8, 40)
    local far = speedTowards(moduleAt(0, 0), 0, 18, 8, 40)

    lu.assertTrue(far > near,
        string.format('further away has to be faster: %.1f at 18 m against %.1f at 3 m', far, near))
    lu.assertTrue(far > 8, 'the boost has to actually exceed the following speed')
end

--- Right on top of the target it settles to the following speed.
function TestDriveToPointSpeed:testItSettlesToTheFollowingSpeed()
    lu.assertAlmostEquals(speedTowards(moduleAt(0, 0), 0, 0.2, 8, 40), 8, 0.001)
end

--- Beyond the boost envelope the cap is all there is.
function TestDriveToPointSpeed:testBeyondTheEnvelopeTheCapGoverns()
    lu.assertAlmostEquals(speedTowards(moduleAt(0, 0), 0, 40, 8, 12), 12, 0.001)
end

os.exit(lu.LuaUnit.run())
