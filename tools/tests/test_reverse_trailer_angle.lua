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

--- Overlapping hulls mean the model does not describe this rig, and the honest answer is none.
---
--- Measured in game: a real tractor and trailer produced "rig allows 0.0" on every frame of a
--- reverse. The subtraction assumes both reference points sit at the middle of their hulls, and a
--- trailer's sits at its axle with the drawbar reaching out in front - so the gap comes out
--- negative. Nought degrees looked like a measurement; it was a failure wearing a number.
function TestRigClearance:testOverlappingHullsGiveNoAnswer()
    local module = rig(5, 3, 3.5, 1.5)
    lu.assertNil(module:getRigClearanceAngle(),
        'a model that does not fit the rig has to say so, not answer zero')
end

--- And the controller then keeps its own figure rather than being driven to a standstill.
function TestRigClearance:testAnUnusableModelLeavesTheControllerLimit()
    local module = rig(5, 3, 3.5, 1.5)
    lu.assertEquals(module:getMaxTrailerAngle(), ADSpecialDrivingModule.MAX_TRAILER_ANGLE)
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
--- A measurement that is real but absurdly small must not lock the steering either.
function TestRigClearance:testAnAbsurdMeasurementIsFloored()
    local module = rig(6.6, 3, 3.5, 8)   -- 0.1 m of gap against an eight metre half width
    local angle = module:getRigClearanceAngle()
    lu.assertNotNil(angle)
    lu.assertTrue(angle < ADSpecialDrivingModule.MIN_PLAUSIBLE_TRAILER_ANGLE)
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

--- Returns the last answer, and separately whether it EVER pulled forward. The difference matters:
--- a pull that starts, runs its length and gives up ends on false, so asking only the last answer
--- would call that "never pulled" - which is how the first version of the solo test passed against
--- a version that did pull.
local function run(module, angles, dt)
    local pulling, everPulled = nil, false
    for _, a in ipairs(angles) do
        module.angleToTrailer = a
        pulling = module:updateFoldRecovery(dt or 100, 40)
        everPulled = everPulled or pulling == true
    end
    return pulling, everPulled
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

--- Both reverse controllers have to act on the decision, not just the one I built it in first.
---
--- driveReverse has two branches. The guided one steers the trailer and runs through reverseToPoint;
--- the blind one drives straight back with the joints locked. A rig of three or more units - a
--- dolly, a road train - takes the blind one, and that is the shape most able to jackknife. It was
--- reached by none of this until a log showed reverseToPoint never being entered at all: 731
--- reverses commanded, and checkWayPointReached, the first thing the guided path logs, absent from
--- the whole file.
---
--- Wiring, not behaviour. It catches the recovery being dropped from either path.
function TestFoldRecovery:testBothReversePathsActOnTheDecision()
    local f = io.open('../../scripts/Modules/SpecialDrivingModule.lua', 'r')
    local src = f:read('*a')
    f:close()

    local guided = select(2, src:gsub('if self:updateFoldRecovery%(dt, maxTrailerAngle%) then', ''))
    lu.assertEquals(guided, 1, 'the guided reverse has to ask whether the rig has folded')

    local blind = select(2, src:gsub('self:updateFoldRecovery%(dt, self:getMaxTrailerAngle%(towed, hitch%), hitch%)', ''))
    lu.assertEquals(blind, 1, 'and so does the blind one, on its own hitch angle')

    local checks = select(2, src:gsub('if self:canPullForward%(%) then', ''))
    lu.assertEquals(checks, 2, 'each of them checks the way forward before driving into it')

    local holds = select(2, src:gsub('the way forward is blocked', ''))
    lu.assertEquals(holds, 2, 'and each holds rather than reversing deeper when it is not')
end


------------------------------------------------------------------------------------------------------------------------
--- The hitch angle without a reverse node
---
--- getBasicStates works the angle out from the REVERSE NODE, which only exists on the guided
--- reverse. The blind path needs the same angle and getReverseNode is never called there, so it is
--- taken from the first towed unit directly - the hitch angle does not depend on which controller
--- happens to be steering.
------------------------------------------------------------------------------------------------------------------------
TestHitchAngle = {}

function TestHitchAngle:setUp()
    TestSetup.reset()
end

local function towedRig(tractorHeading, unitHeading)
    local module = setmetatable({}, { __index = ADSpecialDrivingModule })
    module.vehicle = TestSetup.vehicle()
    local unit = { name = 'towed' }
    AutoDrive.getAllTowedUnits = function() return { module.vehicle, unit }, 2 end
    AutoDrive.localDirectionToWorld = function(v)
        local a = math.rad(v == unit and unitHeading or tractorHeading)
        return math.sin(a), 0, math.cos(a)
    end
    return module, unit
end

function TestHitchAngle:testAStraightRigIsZero()
    local module = towedRig(0, 0)
    lu.assertAlmostEquals(module:getHitchAngle(module:firstTowedUnit()), 0, 0.01)
end

function TestHitchAngle:testAFoldedRigReadsTheFold()
    local module = towedRig(0, 35)
    lu.assertAlmostEquals(math.abs(module:getHitchAngle(module:firstTowedUnit())), 35, 0.01)
end

--- It folds both ways and the reading has to follow.
function TestHitchAngle:testItFoldsBothWays()
    local left = towedRig(0, 35):getHitchAngle(towedRig(0, 35):firstTowedUnit())
    local right = towedRig(0, -35):getHitchAngle(towedRig(0, -35):firstTowedUnit())
    lu.assertTrue(left * right < 0, 'the two sides cannot have the same sign')
end

--- Nothing towed means no hitch angle, and the caller must get nil rather than a zero it would
--- read as a perfectly straight rig.
function TestHitchAngle:testNothingTowedGivesNil()
    local module = towedRig(0, 0)
    AutoDrive.getAllTowedUnits = function() return { module.vehicle }, 1 end
    lu.assertNil(module:firstTowedUnit())
    lu.assertNil(module:getHitchAngle(nil))
end

os.exit(lu.LuaUnit.run())
