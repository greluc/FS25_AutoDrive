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


------------------------------------------------------------------------------------------------------------------------
--- Pulling forward out of a fold
---
--- Past a certain hitch angle reversing cannot undo the fold: every further metre backwards drives
--- the trailer further into the tractor, whatever the steering does. The only way out is the one
--- every driver uses - pull forward, which straightens a trailer on its own, and then resume.
---
--- The threshold is not a number from a textbook. Whether reversing is still recovering is
--- observable: the angle is past what the controller aims for AND it is not getting smaller, held
--- over a stretch rather than judged on one frame.
------------------------------------------------------------------------------------------------------------------------
TestFoldRecovery = {}

function TestFoldRecovery:setUp()
    TestSetup.reset()
end

local function folding()
    local module = setmetatable({}, { __index = ADSpecialDrivingModule })
    module.vehicle = TestSetup.vehicle()
    module.foldTimer = AutoDriveTON:new()
    module.straighteningTimer = AutoDriveTON:new()
    module.straightening = false
    return module
end

--- Inside the limit is not a fold at all, however long it lasts.
function TestFoldRecovery:testAnAngleInsideTheLimitIsNotAFold()
    lu.assertFalse(ADSpecialDrivingModule.isFoldGettingWorse(30, 29, 40))
end

--- Past the limit and still growing is the case this exists for.
function TestFoldRecovery:testPastTheLimitAndGrowingIsAFold()
    lu.assertTrue(ADSpecialDrivingModule.isFoldGettingWorse(50, 45, 40))
end

--- Past the limit but coming back means reversing IS recovering it - leave it alone.
function TestFoldRecovery:testPastTheLimitButRecoveringIsNotAFold()
    lu.assertFalse(ADSpecialDrivingModule.isFoldGettingWorse(45, 50, 40))
end

--- The sign is not the point; a rig folds both ways.
function TestFoldRecovery:testItFoldsBothWays()
    lu.assertTrue(ADSpecialDrivingModule.isFoldGettingWorse(-50, -45, 40))
    lu.assertFalse(ADSpecialDrivingModule.isFoldGettingWorse(-45, -50, 40))
end

local function run(module, angles, dt)
    local pulling = nil
    for _, a in ipairs(angles) do
        module.angleToTrailer = a
        pulling = module:updateFoldRecovery(dt or 100, 40)
    end
    return pulling
end

--- One bad frame is not a fold. It has to persist.
function TestFoldRecovery:testAMomentaryFoldDoesNotTriggerIt()
    local module = folding()
    local angles = {}
    for i = 1, 10 do angles[i] = 45 + i * 0.1 end
    lu.assertFalse(run(module, angles) == true, 'a second of it is not enough')
end

--- Held long enough without improving, the vehicle pulls forward.
function TestFoldRecovery:testASustainedFoldPullsForward()
    local module = folding()
    local angles = {}
    for i = 1, 50 do angles[i] = 45 + i * 0.1 end
    lu.assertTrue(run(module, angles), 'reversing is not recovering this - pull forward')
end

--- And it stops once the rig is comfortably straight again, not the instant it touches the limit.
function TestFoldRecovery:testItResumesOnceStraightened()
    local module = folding()
    local angles = {}
    for i = 1, 50 do angles[i] = 45 + i * 0.1 end
    run(module, angles)
    lu.assertTrue(module.straightening, 'test setup: it has to be pulling forward first')

    lu.assertTrue(run(module, { 39, 38 }), 'just inside the limit is not straight enough yet')
    lu.assertFalse(run(module, { 20 }), 'well inside it is')
end

--- A pull that never straightens the rig has to hand back rather than drive on for ever.
function TestFoldRecovery:testThePullForwardIsBounded()
    local module = folding()
    local angles = {}
    for i = 1, 50 do angles[i] = 45 + i * 0.1 end
    run(module, angles)

    local stillPulling = true
    for _ = 1, math.ceil(ADSpecialDrivingModule.FOLD_RECOVERY_MAX / 100) + 5 do
        module.angleToTrailer = 60
        stillPulling = module:updateFoldRecovery(100, 40)
    end

    lu.assertFalse(stillPulling, 'it cannot pull forward indefinitely')
end

--- And the controller has to actually pull forward, not merely be able to decide that it should.
--- The tests above drive updateFoldRecovery directly; this pins that its answer reaches the vehicle.
--- Wiring, not behaviour, and worth exactly that much.
function TestFoldRecovery:testTheControllerActsOnTheDecision()
    local f = io.open('../../scripts/Modules/SpecialDrivingModule.lua', 'r')
    local src = f:read('*a')
    f:close()

    lu.assertNotNil(src:match('if self:updateFoldRecovery%(dt, maxTrailerAngle%) then'),
        'the reverse controller has to ask whether the rig has folded past recovery')
    local branch = src:match('if self:updateFoldRecovery%(dt, maxTrailerAngle%) then(.-)end')
    lu.assertNotNil(branch)
    lu.assertNotNil(branch:match('self:driveForward%(dt%)'),
        'and answering yes means pulling forward')
end

--- The forward pull moves the rig in a direction nothing asked for, so it asks whether the way is
--- clear rather than arguing that it probably is.
function TestFoldRecovery:testItRefusesToPullIntoSomething()
    local module = folding()
    module.vehicle.ad.sensors = { frontSensor = { pollInfo = function() return true end } }

    lu.assertFalse(module:canPullForward(), 'something is in front - do not drive into it')
end

function TestFoldRecovery:testAClearWayForwardIsPullable()
    local module = folding()
    module.vehicle.ad.sensors = { frontSensor = { pollInfo = function() return false end } }

    lu.assertTrue(module:canPullForward())
end

--- A rig without that sensor is not a reason to refuse the recovery entirely.
function TestFoldRecovery:testAMissingSensorDoesNotBlockTheRecovery()
    local module = folding()
    module.vehicle.ad.sensors = nil
    lu.assertTrue(module:canPullForward())

    module.vehicle.ad.sensors = { }
    lu.assertTrue(module:canPullForward())
end

--- And the controller has to consult it before driving.
function TestFoldRecovery:testTheControllerChecksBeforePulling()
    local f = io.open('../../scripts/Modules/SpecialDrivingModule.lua', 'r')
    local src = f:read('*a')
    f:close()

    local decide = src:find('if self:updateFoldRecovery%(dt, maxTrailerAngle%) then')
    local ask = src:find('if self:canPullForward%(%) then')
    local hold = src:find('reverse: folded to %%.1f deg and the way forward is blocked')
    lu.assertNotNil(decide, 'the controller has to ask whether the rig has folded past recovery')
    lu.assertNotNil(ask, 'and then whether the way forward is clear')
    lu.assertNotNil(hold, 'and hold rather than reverse deeper when it is not')
    lu.assertTrue(ask > decide, 'the check belongs inside the recovery branch')
end

os.exit(lu.LuaUnit.run())
