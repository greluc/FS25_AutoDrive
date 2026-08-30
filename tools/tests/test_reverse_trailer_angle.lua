lu = require('luaunit')
require('test-setup')
require('AutoDriveTON')
require('SpecialDrivingModule')

------------------------------------------------------------------------------------------------------------------------
--- How far the reverse controller may fold the rig
---
--- The controller clamps its TARGET hitch angle, and that clamp was a fixed forty degrees - a guess
--- about control stability, not a statement about the rig. How far a trailer can swing before its
--- front corner reaches the tractor depends on the drawbar, the width and where the hitch sits.
---
--- FS25 puts the attached implement's per-axis joint rotation limits on the implement:
--- jointRotLimit[2] is the Y axis, which is yaw, in radians. Only 70 of the 267 base game vehicles
--- with a trailer hitch declare one, so this is not a general answer - but where it exists it is a
--- real number instead of a guess, and it is taken as a ceiling so the change can only ever reduce
--- how far the trailer is asked to fold.
------------------------------------------------------------------------------------------------------------------------
TestMaxTrailerAngle = {}

function TestMaxTrailerAngle:setUp()
    TestSetup.reset()
end

local function moduleWithTrailer(yawLimitDegrees)
    local module = setmetatable({}, { __index = ADSpecialDrivingModule })
    module.vehicle = TestSetup.vehicle()
    if yawLimitDegrees ~= nil then
        module.vehicle.trailer = { jointRotLimit = { 0, math.rad(yawLimitDegrees), 0 } }
    end
    return module
end

--- Nothing attached, or nothing declared: the figure the controller has always used.
function TestMaxTrailerAngle:testItFallsBackWhenNothingIsDeclared()
    lu.assertEquals(moduleWithTrailer(nil):getMaxTrailerAngle(), ADSpecialDrivingModule.MAX_TRAILER_ANGLE)
end

function TestMaxTrailerAngle:testATrailerWithoutTheFieldFallsBack()
    local module = moduleWithTrailer(nil)
    module.vehicle.trailer = {}
    lu.assertEquals(module:getMaxTrailerAngle(), ADSpecialDrivingModule.MAX_TRAILER_ANGLE)
end

--- A rig that may only fold thirty degrees must not be aimed at forty.
function TestMaxTrailerAngle:testATighterDeclaredLimitIsUsed()
    lu.assertAlmostEquals(moduleWithTrailer(30):getMaxTrailerAngle(), 30, 0.001)
end

--- One that may fold eighty is still steered to forty: the clamp is also what keeps the controller
--- stable, so this is a ceiling and never a replacement.
function TestMaxTrailerAngle:testAWiderDeclaredLimitDoesNotRaiseIt()
    lu.assertEquals(moduleWithTrailer(80):getMaxTrailerAngle(), ADSpecialDrivingModule.MAX_TRAILER_ANGLE)
end

--- A locked or unset joint reads as a tiny number, and taking that as a hinge angle would stop the
--- vehicle steering at all.
function TestMaxTrailerAngle:testAnImplausiblySmallLimitIsIgnored()
    lu.assertEquals(moduleWithTrailer(0):getMaxTrailerAngle(), ADSpecialDrivingModule.MAX_TRAILER_ANGLE)
    lu.assertEquals(moduleWithTrailer(2):getMaxTrailerAngle(), ADSpecialDrivingModule.MAX_TRAILER_ANGLE)
end

--- The sign is not the point - a limit is a magnitude either way.
function TestMaxTrailerAngle:testItReadsTheMagnitude()
    lu.assertAlmostEquals(moduleWithTrailer(-25):getMaxTrailerAngle(), 25, 0.001)
end

--- And the controller has to actually use it.
---
--- The tests above drive getMaxTrailerAngle directly and say nothing about whether its answer ever
--- reaches the vehicle - the same gap that let a corner brake be computed and thrown away, and a
--- collision box be turned ninety degrees, with the whole suite green. Driving reverseToPoint itself
--- needs a running rig; this reads the source instead. It pins the wiring, not the behaviour, and it
--- is worth exactly that much: it catches the clamp being put back to a literal.
function TestMaxTrailerAngle:testTheControllerClampsToTheRigLimitNotToALiteral()
    local f = io.open('../../scripts/Modules/SpecialDrivingModule.lua', 'r')
    local src = f:read('*a')
    f:close()

    lu.assertNotNil(src:match('targetAngleToTrailer = math%.clamp'),
        'test setup: the clamp is what this is about')
    lu.assertNotNil(src:match('%-maxTrailerAngle, maxTrailerAngle'),
        'the target hitch angle has to be clamped to the rig limit')
    lu.assertNil(src:match('targetAngleToTrailer = math%.clamp%b()%s*$'),
        'and not to a literal')
    lu.assertNotNil(src:match('local maxTrailerAngle = self:getMaxTrailerAngle%(%)'),
        'which means asking for it')
end

os.exit(lu.LuaUnit.run())
