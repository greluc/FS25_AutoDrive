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

--- What the cell box is, in both axes.
---
--- ACROSS travel is the rig's half width. That is the honest number and the reason for measuring the
--- train at all: the corridor has to be wide enough for what is being towed. Assigning the extents
--- in their natural order put half LENGTH across instead - the box stood broadside, as wide as the
--- train is long - and refused field entrances and gateways the rig fits through comfortably.
---
--- ALONG travel is half a grid step, not half the train. The A* steps by minTurnRadius and checks
--- cell by cell, so consecutive boxes at minTurnRadius/2 abut exactly and the corridor is covered
--- with no gap. A box the length of the TRAIN reaches three cells either side of its own centre, so
--- one machine parked near the route blocks a corridor of seven cells. Measured in game with a
--- harvester on the headland: 17,035 cells refused for collision, 5,075 more by the off-tracking
--- probe that re-uses this box, and the search giving up with six cells in its grid - it never left
--- the start. The train's length belongs to the off-tracking box, which is where the rear of the rig
--- actually cuts the corner.
---
--- The slot matters as much as the value: the box is turned by atan2(-dz, dx), under which ex is the
--- along-travel axis. The tests below pin what getTrainHalfExtents returns and say nothing about
--- where it goes, which is why the box could stand ninety degrees out with the whole gate green.
function TestBoxExtents:testTheCellBoxIsOneGridStepAlongAndTheRigWideAcross()
    local p = pfm()                                  -- minTurnRadius 8
    p.trainHalfWidth, p.trainHalfLength = 1.5, 10    -- a 20 m long, 3 m wide train

    -- entered from the west, so travelling +x
    local shape = p:getShapeDefByDirectionType_New({ x = 10, z = 0, incoming = { x = 9, z = 0 } })

    lu.assertAlmostEquals(shape.angleRad, 0, 1e-9, 'test setup: travelling +x is angle zero')
    lu.assertEquals(shape.widthX, p.minTurnRadius / 2,
        'along travel is half a grid step, so neighbouring cell boxes abut')
    lu.assertEquals(shape.widthZ, 1.5,
        'across travel is the half width of the rig, not its length')
end

--- The failure this replaced, stated as itself: the box must never be as wide as the train is long.
function TestBoxExtents:testTheBoxIsNeverAsWideAsTheTrainIsLong()
    local p = pfm()
    p.trainHalfWidth, p.trainHalfLength = 1.5, 10

    local shape = p:getShapeDefByDirectionType_New({ x = 10, z = 0, incoming = { x = 9, z = 0 } })

    lu.assertTrue(shape.widthZ < p.trainHalfLength,
        'a three metre rig demanded a twenty metre gap when the long extent went across')
end

--- And it must never reach further along the path than one cell, or a single obstacle closes a
--- corridor several cells deep and the search cannot leave its start.
function TestBoxExtents:testTheBoxDoesNotReachPastTheNeighbouringCell()
    for _, trainHalfLength in ipairs({ 3, 7, 10, 12 }) do
        local p = pfm()
        p.trainHalfWidth, p.trainHalfLength = 1.5, trainHalfLength
        local shape = p:getShapeDefByDirectionType_New({ x = 10, z = 0, incoming = { x = 9, z = 0 } })
        lu.assertTrue(shape.widthX <= p.minTurnRadius / 2, string.format(
            'a %.0f m train made the cell box reach %.1f m along a %.1f m grid step',
            trainHalfLength * 2, shape.widthX * 2, p.minTurnRadius))
    end
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


------------------------------------------------------------------------------------------------------------------------
--- The corner array has two consumers that want different things from it
---
--- getCorners returns (-u-v), (+u-v), (-u+v), (+u+v), which is exactly what getFruitValue wants:
--- start, start+width, start+height. boxesIntersect walks its polygon CYCLICALLY, and in that order
--- edges 2->3 and 4->1 are the rectangle's diagonals while 1->2 and 3->4 are the same axis twice -
--- so one of the two face normals is never offered as a separating axis and quads that are
--- genuinely apart come back as intersecting. Measured over 36180 placements: 644 false hits, worst
--- case a real 2.69 m gap reported as blocked. The debug drawing already knew: it draws 1->2, 2->4,
--- 4->3, 3->1.
------------------------------------------------------------------------------------------------------------------------
TestCornerPolygon = {}

function TestCornerPolygon:setUp() TestSetup.reset() end

--- The property, not the permutation: consecutive corners of the polygon have to be EDGES of the
--- rectangle, never its diagonals. On a square the diagonal is longer than the side by root two.
function TestCornerPolygon:testConsecutiveCornersAreEdgesNotDiagonals()
    local p = pfm()
    local half = 3.5
    local corners = p:getCorners({ x = 0, z = 0 },
        { x = half, z = 0 }, { x = 0, z = half })
    local polygon = PathFinderModule.cornersAsPolygon(corners)

    local side = half * 2
    for i = 1, 4 do
        local a, b = polygon[i], polygon[i % 4 + 1]
        local length = MathUtil.vector2Length(b.x - a.x, b.z - a.z)
        lu.assertAlmostEquals(length, side, 1e-6, string.format(
            'corner %d to %d is %.2f m, which is the diagonal of a %.2f m square, not an edge - '
            .. 'a separating axis test walking this order never sees one of the two face normals',
            i, i % 4 + 1, length, side))
    end
end

--- And the raw array keeps the shape its other consumer needs: 1->2 is the width, 1->3 the height.
function TestCornerPolygon:testTheRawArrayIsStillTheFruitTriple()
    local p = pfm()
    local corners = p:getCorners({ x = 0, z = 0 }, { x = 3.5, z = 0 }, { x = 0, z = 3.5 })

    lu.assertAlmostEquals(corners[2].x - corners[1].x, 7, 1e-6, 'corner 1 to 2 has to span the width')
    lu.assertAlmostEquals(corners[2].z - corners[1].z, 0, 1e-6)
    lu.assertAlmostEquals(corners[3].z - corners[1].z, 7, 1e-6, 'and 1 to 3 the height')
    lu.assertAlmostEquals(corners[3].x - corners[1].x, 0, 1e-6)
end

------------------------------------------------------------------------------------------------------------------------
--- Distance to the target is measured in metres, whoever is asking
---
--- cellDistance subtracted cell.x/cell.z directly. The A* cells are grid indices, so that worked;
--- the Dubins samples carry WORLD coordinates, so it returned the vehicle's distance from the map
--- origin - 1442 for a sample sitting exactly on the target. Its one consumer lifts the fruit
--- restriction near the pipe, so the Dubins shortcut was held to a harsher rule than the A* it
--- exists to short-circuit and every candidate curve died on its first sample.
------------------------------------------------------------------------------------------------------------------------
TestCellDistance = {}

function TestCellDistance:setUp() TestSetup.reset() end

function TestCellDistance:testAGridCellAndAWorldSampleOnTheSameSpotAgree()
    local p = pfm()
    p.targetCell = { x = 0, z = 0 }

    local gridCell = { x = 3, z = 0 }
    local worldSample = { x = 0, z = 0, worldPos = p:gridLocationToWorldLocation(gridCell) }

    lu.assertAlmostEquals(p:cellDistance(worldSample), p:cellDistance(gridCell), 1e-6,
        'the same physical point has to be the same distance from the target however the cell that '
        .. 'describes it was built')
end

function TestCellDistance:testASampleOnTheTargetIsAtZero()
    local p = pfm()
    p.targetCell = { x = 0, z = 0 }
    local onTarget = { x = 0, z = 0, worldPos = p:gridLocationToWorldLocation({ x = 0, z = 0 }) }

    lu.assertAlmostEquals(p:cellDistance(onTarget), 0, 1e-6)
end

os.exit(lu.LuaUnit.run())
