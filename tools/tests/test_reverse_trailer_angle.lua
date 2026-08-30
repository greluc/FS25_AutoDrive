lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
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


------------------------------------------------------------------------------------------------------------------------
--- The angle the rig's own shape allows
---
--- Only two numbers decide where a trailer's front corner reaches the back of the tractor: the free
--- gap between them when the rig is straight, and how wide the trailer is. The corner swings about
--- the coupling and arrives at the tractor's rear plane after atan(gap / halfWidth). This is what
--- replaces the fixed forty for the 197 of 267 base game vehicles that declare no joint limit.
---
--- Every approximation in it errs towards a SMALLER angle than the rig can really manage, so it can
--- make the controller gentler and never bolder.
------------------------------------------------------------------------------------------------------------------------
TestRigClearance = {}

function TestRigClearance:setUp()
    TestSetup.reset()
end

--- tractorBack: how far the tractor's hull reaches behind its root. trailerFront: the same forwards
--- for the trailer. spacing: how far apart the two roots are.
local function rig(spacing, tractorBack, trailerFront, trailerHalfWidth, hitchAngle)
    local module = setmetatable({}, { __index = ADSpecialDrivingModule })
    module.vehicle = TestSetup.vehicle()
    module.vehicle.components = { { node = 'tractor' } }
    module.vehicle.ad.adDimensions = {
        maxWidthLeft = 1.5, maxWidthRight = 1.5,
        maxLengthFront = 3, maxLengthBack = tractorBack,
    }
    MockEngine.nodePositions['tractor'] = { x = 0, y = 0, z = 0 }

    local trailer = { components = { { node = 'trailer' } } }
    trailer.ad = { adDimensions = {
        maxWidthLeft = trailerHalfWidth, maxWidthRight = trailerHalfWidth,
        maxLengthFront = trailerFront, maxLengthBack = 5,
    } }
    MockEngine.nodePositions['trailer'] = { x = 0, y = 0, z = -spacing }
    module.vehicle.trailer = trailer

    module.angleToTrailer = hitchAngle or 0
    return module
end

--- A gap equal to the half width is forty-five degrees, by construction.
function TestRigClearance:testAGapEqualToTheHalfWidthIsFortyFive()
    -- roots 8 m apart, hulls take 3 + 3.5, leaving 1.5 m of gap against a 1.5 m half width
    local module = rig(8, 3, 3.5, 1.5)
    lu.assertAlmostEquals(module:getRigClearanceAngle(), 45, 0.01)
end

--- A wide trailer close behind folds into the tractor early.
function TestRigClearance:testATightWideRigFoldsEarly()
    local module = rig(7, 3, 3.5, 1.5)   -- 0.5 m gap, 1.5 m half width
    lu.assertTrue(module:getRigClearanceAngle() < 20,
        'half a metre of room behind a three metre wide trailer is not forty degrees')
end

--- A long drawbar gives room.
function TestRigClearance:testALongDrawbarAllowsMore()
    local module = rig(11, 3, 3.5, 1.5)  -- 4.5 m gap
    lu.assertTrue(module:getRigClearanceAngle() > 60)
end

--- Overlapping hulls are not a negative gap, they are no room at all.
function TestRigClearance:testOverlappingHullsGiveZero()
    local module = rig(5, 3, 3.5, 1.5)
    lu.assertAlmostEquals(module:getRigClearanceAngle(), 0, 0.01)
end

--- Measured only while the rig is straight: taken mid fold it would read the already rotated
--- geometry and answer far too small.
function TestRigClearance:testItRefusesToMeasureAMidFoldRig()
    local module = rig(8, 3, 3.5, 1.5, 35)
    lu.assertNil(module:getRigClearanceAngle())
end

--- And once measured it stays measured - the gap does not change while driving.
function TestRigClearance:testItIsMeasuredOnce()
    local module = rig(8, 3, 3.5, 1.5)
    local first = module:getRigClearanceAngle()
    module.angleToTrailer = 35
    lu.assertAlmostEquals(module:getRigClearanceAngle(), first, 0.01)
end

function TestRigClearance:testNoTrailerNoAngle()
    local module = rig(8, 3, 3.5, 1.5)
    module.vehicle.trailer = nil
    lu.assertNil(module:getRigClearanceAngle())
end

--- It feeds the controller's ceiling like the declared limit does, and only downwards.
function TestRigClearance:testATightRigLowersTheControllerLimit()
    local module = rig(7, 3, 3.5, 1.5)
    lu.assertTrue(module:getMaxTrailerAngle() < ADSpecialDrivingModule.MAX_TRAILER_ANGLE)
end

function TestRigClearance:testARoomyRigDoesNotRaiseIt()
    local module = rig(11, 3, 3.5, 1.5)
    lu.assertEquals(module:getMaxTrailerAngle(), ADSpecialDrivingModule.MAX_TRAILER_ANGLE)
end

--- An absurdly small measurement must not lock the steering.
function TestRigClearance:testAnAbsurdMeasurementIsFloored()
    local module = rig(5, 3, 3.5, 1.5)   -- zero gap, so zero degrees
    lu.assertEquals(module:getMaxTrailerAngle(), ADSpecialDrivingModule.MAX_TRAILER_ANGLE)
end

os.exit(lu.LuaUnit.run())
