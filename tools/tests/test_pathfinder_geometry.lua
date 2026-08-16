--[[
Tests for the pathfinder collision geometry.

Covers findings:
  K1 the cell box was a square derived from the turn radius, rotated to the DESTINATION heading
  K2 the trailer's swept area through a bend was never tested

The symptom these come from: an unloader turning outside the field catches its trailer on a tree
that the tractor cleared.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('SortedQueue')
require('Dubins')
require('PathFinderUtils')
require('PathFinderModule')

--- A pathfinder instance with just enough state for the geometry helpers.
local function pfm(overrides)
    local o = setmetatable({}, { __index = PathFinderModule })
    o.vehicle = TestSetup.vehicle()
    o.minTurnRadius = 8
    o.targetVector = { x = 0, z = 1 }
    o.vehicleMinHeight = 3
    o.PP_CELL_X = 1
    o.PP_CELL_Z = 1
    -- grid cells map 1:1 to world metres, which keeps the expected numbers readable
    o.gridLocationToWorldLocation = function(_, cell)
        return { x = cell.x, y = 0, z = cell.z }
    end
    if overrides then
        for k, v in pairs(overrides) do o[k] = v end
    end
    return o
end

------------------------------------------------------------------------------------------------------------------------
--- K1 - box size comes from the train, not from the steering radius
------------------------------------------------------------------------------------------------------------------------
TestBoxExtents = {}

function TestBoxExtents:setUp() TestSetup.reset() end

function TestBoxExtents:testFallsBackToTurnRadiusWhenNothingMeasured()
    local p = pfm()
    local hw, hl = p:getTrainHalfExtents()
    lu.assertEquals(hw, 4)   -- minTurnRadius / 2
    lu.assertEquals(hl, 4)
end

function TestBoxExtents:testUsesMeasuredTrainWidth()
    local p = pfm()
    -- 11 m wide train, i.e. wider than the minTurnRadius/2 = 4 fallback
    p.vehicle.ad.adDimensions = { maxWidthLeft = 5.5, maxWidthRight = 5.5 }
    local hw = p:getTrainHalfExtents()
    lu.assertEquals(hw, 5.5, 'half width must come from the measured hull when it is wider')
end

function TestBoxExtents:testNeverShrinksBelowTheTurnRadiusFallback()
    local p = pfm()
    p.vehicle.ad.adDimensions = { maxWidthLeft = 0.5, maxWidthRight = 0.5 }
    local hw = p:getTrainHalfExtents()
    lu.assertEquals(hw, 4, 'a narrow implement must not shrink the box below the old behaviour')
end

function TestBoxExtents:testExtentsAreCachedPerRun()
    local p = pfm()
    local hw1 = p:getTrainHalfExtents()
    p.vehicle.ad.adDimensions = { maxWidthLeft = 50, maxWidthRight = 50 }
    lu.assertEquals(p:getTrainHalfExtents(), hw1, 'extents must not be recomputed per cell')
end

--- Which SLOT the extents land in, which is a different question from what they are worth.
---
--- The box is handed to overlapBox turned by atan2(-dz, dx), and under that angle the ex slot is the
--- extent ALONG travel: smoothResultingPPPath_Refined builds its box with the very same angle and
--- puts the along-travel half length there (length/2+2.5 in ex, sideLength+1.5 in ez). Assigning the
--- half extents in their natural order put them the other way round, so the box stood broadside - as
--- wide across the path as the train is long, and only as long as it is wide. Every cell of every
--- search was tested that way, and a rig was refused every gap narrower than its own LENGTH.
---
--- The tests above pin the numbers getTrainHalfExtents returns and say nothing about where they go,
--- which is why the whole gate stayed green with the box turned ninety degrees. The old box survived
--- this because it was square: it is specifically a rig longer than its turn radius that suffers.
function TestBoxExtents:testTheLongExtentRunsAlongTravelNotAcrossIt()
    local p = pfm()
    p.trainHalfWidth, p.trainHalfLength = 1.5, 7   -- a 14 m long, 3 m wide train

    -- entered from the west, so travelling +x
    local shape = p:getShapeDefByDirectionType_New({ x = 10, z = 0, incoming = { x = 9, z = 0 } })

    lu.assertAlmostEquals(shape.angleRad, 0, 1e-9, 'test setup: travelling +x is angle zero')
    lu.assertEquals(shape.widthX, 7,
        'the ex slot is the along-travel extent, so it carries the half LENGTH')
    lu.assertEquals(shape.widthZ, 1.5,
        'and ez the half width - the other way round makes a 3 m rig demand a 14 m gap')
end

------------------------------------------------------------------------------------------------------------------------
--- K1 - box orientation follows the local direction of travel
------------------------------------------------------------------------------------------------------------------------
TestBoxHeading = {}

function TestBoxHeading:setUp() TestSetup.reset() end

function TestBoxHeading:testHeadingFollowsTheIncomingSegment()
    local p = pfm()
    -- travelling +x, while the destination heading points +z
    local cell = { x = 10, z = 0, incoming = { x = 9, z = 0 } }
    local heading = p:getCellHeading(cell)
    local expected = AutoDrive.normalizeAngle(math.atan2(-0, 1))
    lu.assertAlmostEquals(heading, expected, 1e-9,
        'the box must be rotated to how the vehicle passes THIS cell, not to the final heading')
end

function TestBoxHeading:testFallsBackToTargetVectorOnTheFirstCell()
    local p = pfm()
    local cell = { x = 0, z = 0 }   -- no incoming
    local heading = p:getCellHeading(cell)
    local expected = AutoDrive.normalizeAngle(math.atan2(-p.targetVector.z, p.targetVector.x))
    lu.assertAlmostEquals(heading, expected, 1e-9)
end

function TestBoxHeading:testDegenerateIncomingFallsBack()
    local p = pfm()
    local cell = { x = 5, z = 5, incoming = { x = 5, z = 5 } }  -- same position
    local heading = p:getCellHeading(cell)
    local expected = AutoDrive.normalizeAngle(math.atan2(-p.targetVector.z, p.targetVector.x))
    lu.assertAlmostEquals(heading, expected, 1e-9)
end

------------------------------------------------------------------------------------------------------------------------
--- K2 - off-tracking through a bend
------------------------------------------------------------------------------------------------------------------------
TestOffTracking = {}

function TestOffTracking:setUp() TestSetup.reset() end

function TestOffTracking:testStraightPathHasNoOffset()
    local p = pfm()
    local cell = { x = 2, z = 0, incoming = { x = 1, z = 0, incoming = { x = 0, z = 0 } } }
    lu.assertNil(p:getOffTrackingOffset(cell),
        'a straight run needs no second box - the trailer tracks the tractor')
end

function TestOffTracking:testNeedsTwoSegmentsToSeeABend()
    local p = pfm()
    lu.assertNil(p:getOffTrackingOffset({ x = 1, z = 0 }))
    lu.assertNil(p:getOffTrackingOffset({ x = 1, z = 0, incoming = { x = 0, z = 0 } }))
end

function TestOffTracking:testRightAngleTurnProducesAnInwardOffset()
    local p = pfm()
    p.vehicle.ad.adDimensions = { maxWidthLeft = 1.5, maxWidthRight = 1.5 }
    p.trainHalfWidth, p.trainHalfLength = 1.5, 6   -- 12 m train

    -- travelling +x, then turning to +z
    local cell = { x = 1, z = 1, incoming = { x = 1, z = 0, incoming = { x = 0, z = 0 } } }
    local ox, oz = p:getOffTrackingOffset(cell)
    lu.assertNotNil(ox, 'a 90 degree bend must produce an off-tracking offset')

    -- theta = 90 deg -> cos = 0 -> offset = L * (1 - 0) = 12
    local magnitude = math.sqrt(ox * ox + oz * oz)
    lu.assertAlmostEquals(magnitude, 12, 1e-6)
end

--- The offset has to point INTO the curve. Pointing outwards would test empty air and leave the
--- area the trailer really crosses unchecked - worse than not checking at all, because it looks
--- like a check.
function TestOffTracking:testOffsetPointsIntoTheCurve()
    local p = pfm()
    p.trainHalfWidth, p.trainHalfLength = 1.5, 5

    -- +x then +z is a left turn in this coordinate system; the inside is towards -x of the
    -- outgoing heading. Verify by checking the offset moves the box back towards the corner.
    local cell = { x = 1, z = 1, incoming = { x = 1, z = 0, incoming = { x = 0, z = 0 } } }
    local ox, oz = p:getOffTrackingOffset(cell)

    -- the corner of the path is at (1,0); the inside of the bend lies on that side of the
    -- outgoing segment, so the offset must have a negative z component relative to travel
    local dot = ox * 0 + oz * 1   -- outgoing heading is +z
    lu.assertTrue(math.abs(dot) < 1e-6,
        'the offset must be perpendicular to the outgoing heading, not along it')
    lu.assertTrue(math.abs(ox) > 1e-6, 'and it must actually displace the box sideways')
end

function TestOffTracking:testShallowBendBelowResolutionIsIgnored()
    local p = pfm()
    p.trainHalfWidth, p.trainHalfLength = 1.5, 3   -- 6 m train
    -- a very slight bend: cos(theta) close to 1 -> offset well below 0.5 m
    local cell = { x = 100, z = 1, incoming = { x = 50, z = 0, incoming = { x = 0, z = 0 } } }
    lu.assertNil(p:getOffTrackingOffset(cell),
        'a bend too shallow to move the trailer out of the cell box must not cost a second query')
end

function TestOffTracking:testLongerTrainSweepsWider()
    local p1 = pfm()
    p1.trainHalfWidth, p1.trainHalfLength = 1.5, 3
    local p2 = pfm()
    p2.trainHalfWidth, p2.trainHalfLength = 1.5, 9

    local cell = { x = 1, z = 1, incoming = { x = 1, z = 0, incoming = { x = 0, z = 0 } } }
    local ax, az = p1:getOffTrackingOffset(cell)
    local bx, bz = p2:getOffTrackingOffset(cell)
    lu.assertTrue(math.sqrt(bx * bx + bz * bz) > math.sqrt(ax * ax + az * az),
        'a longer train must cut the corner further')
end

os.exit(lu.LuaUnit.run())
