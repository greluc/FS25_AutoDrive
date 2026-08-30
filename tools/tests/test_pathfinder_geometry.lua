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
-- for MAX_TRAILER_ANGLE: the planner and the reversing code must agree on the hitch limit,
-- so the test reads the real constant rather than restating it
require('SpecialDrivingModule')

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
    lu.assertAlmostEquals(hw, 5.5 + PathFinderModule.TRAIN_SIDE_CLEARANCE, 0.001,
        'half width comes from the measured hull, plus room to drive in')
end

--- The reversal of an earlier call of mine, and the reason it was wrong.
---
--- This used to assert the opposite - that a narrow rig keeps the wider turn-radius box, "must not
--- shrink below the old behaviour". That made the measurement inert, because a tractor and trailer
--- measure about 1.5 m to a side and minTurnRadius/2 is four to seven, so the box was always the
--- turn radius and the rig was never consulted. A steering radius says nothing about how much room
--- a vehicle needs BESIDE itself, and demanding a fourteen metre corridor for a three metre rig
--- refuses headlands and field entrances it drives through comfortably. Measured in one session:
--- 5,178 cells refused for collision against 8 refused for fruit, and a search that had failed four
--- times succeeded once the player nudged the vehicle two metres.
function TestBoxExtents:testANarrowRigGetsACorridorItsOwnSize()
    local p = pfm()
    p.vehicle.ad.adDimensions = { maxWidthLeft = 0.5, maxWidthRight = 0.5 }
    local hw = p:getTrainHalfExtents()
    lu.assertAlmostEquals(hw, 0.5 + PathFinderModule.TRAIN_SIDE_CLEARANCE, 0.001,
        'the corridor is the rig plus clearance, not the steering radius')
    lu.assertTrue(hw < 4, 'and that is narrower than the fallback, which is the whole point')
end

--- The clearance is real room, not a rounding error: a corridor of exactly rig width would be
--- optimistic, because the driven line is smoothed between cell centres rather than tracking them.
function TestBoxExtents:testTheCorridorIsWiderThanTheRigItself()
    local p = pfm()
    p.vehicle.ad.adDimensions = { maxWidthLeft = 1.5, maxWidthRight = 1.5 }

    lu.assertTrue(p:getTrainHalfExtents() > 1.5)
end

function TestBoxExtents:testExtentsAreCachedPerRun()
    local p = pfm()
    local hw1 = p:getTrainHalfExtents()
    p.vehicle.ad.adDimensions = { maxWidthLeft = 50, maxWidthRight = 50 }
    lu.assertEquals(p:getTrainHalfExtents(), hw1, 'extents must not be recomputed per cell')
end

--- What the cell box is, in both axes.
---
--- ACROSS travel is the rig's half width plus a clearance. That is the honest number and the reason
--- for measuring the train at all: the corridor has to be wide enough for what is being towed, and
--- no wider. Assigning the extents
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


------------------------------------------------------------------------------------------------------------------------
--- What counts as an obstacle
---
--- Not ourselves, and not the vehicle we are driving TO. A path to a harvester's pipe ends alongside
--- the harvester, and an unloader asking for one is usually standing beside it already - so counting
--- it as an obstacle walls in both ends of the search at once.
---
--- Measured in game: the A* popped its start node, had every neighbour refused, and gave up on the
--- first iteration with an empty open list ("PFM:find exit end count 1", grid of six) while the
--- harvester it could not reach stood a hundred metres away. The vehicle was already handed in -
--- startPathPlanningToPipe takes the combine, startPathPlanningToVehicle the target - and both threw
--- it away after reading a position out of it, keeping only a boolean saying a chase was on.
------------------------------------------------------------------------------------------------------------------------
TestCollisionCallback = {}

function TestCollisionCallback:setUp() TestSetup.reset() end

local function hitsFor(nodeObject, targetVehicle)
    local p = pfm()
    p.collisionhits = 0
    p.targetVehicle = targetVehicle
    g_currentMission.getNodeObject = function(_, _) return nodeObject end
    g_currentMission.terrainRootNode = 999
    p:collisionTestCallback(1)
    return p.collisionhits
end

function TestCollisionCallback:testAStrangerIsAnObstacle()
    local other = {}
    other.rootVehicle = other
    lu.assertEquals(hitsFor(other, nil), 1)
end

function TestCollisionCallback:testWeAreNotOurOwnObstacle()
    local p = pfm()
    p.collisionhits = 0
    g_currentMission.getNodeObject = function(_, _) return { rootVehicle = p.vehicle } end
    g_currentMission.terrainRootNode = 999
    p:collisionTestCallback(1)
    lu.assertEquals(p.collisionhits, 0)
end

--- The one this exists for.
function TestCollisionCallback:testTheVehicleWeAreDrivingToIsNotAnObstacle()
    local combine = {}
    combine.rootVehicle = combine
    lu.assertEquals(hitsFor(combine, combine), 0,
        'a path to a harvester cannot be found if the harvester is a wall')
end

--- And it is only that one vehicle, not every vehicle on the field.
function TestCollisionCallback:testAnotherVehicleIsStillAnObstacleWhileChasing()
    local combine = {}
    combine.rootVehicle = combine
    local stranger = {}
    stranger.rootVehicle = stranger
    lu.assertEquals(hitsFor(stranger, combine), 1)
end

--- The terrain itself is not an obstacle, and neither is node zero.
function TestCollisionCallback:testTerrainAndNothingAreNotObstacles()
    local p = pfm()
    p.collisionhits = 0
    g_currentMission.terrainRootNode = 999
    g_currentMission.getNodeObject = function(_, _) return nil end
    p:collisionTestCallback(0)
    p:collisionTestCallback(999)
    lu.assertEquals(p.collisionhits, 0)
end

--- And the callback can only spare the target if the search was told which vehicle that is.
---
--- Both entry points already receive it and used to throw it away after reading a position out of
--- it. Driving startPathPlanningToPipe needs a combine, a pipe, a chase position and a grid, so this
--- reads the source instead: it pins the wiring, not the behaviour, and that is exactly the half
--- that was missing - the callback above was correct in isolation and had nothing to compare against.
function TestCollisionCallback:testBothEntryPointsRecordTheVehicleTheyAreDrivingTo()
    local f = io.open('../../scripts/Modules/PathFinderModule.lua', 'r')
    local src = f:read('*a')
    f:close()

    local function contains(needle, message)
        lu.assertNotNil(string.find(src, needle, 1, true), message)
    end

    contains('self.targetVehicle = combine',
        'startPathPlanningToPipe has to remember the combine')
    contains('self.targetVehicle = targetVehicle',
        'startPathPlanningToVehicle has to remember its target')
    contains('self.targetVehicle = nil',
        'and it has to be let go when the chase ends, or the next search spares a stranger')
end


------------------------------------------------------------------------------------------------------------------------
--- A vehicle STANDING in the way, as opposed to one whose route crosses it
---
--- The planner ran exactly one vehicle test, checkForVehiclePathInBox, and that walks other
--- vehicles' waypoint ROUTES - only for vehicles that are AutoDrive-active and on a pathfinder path.
--- A harvester parked on the headland is none of those, and its body was never compared against
--- anything, so it was not an obstacle to plan around at all.
---
--- Measured: in a 62 MB log the harvester blocked exactly ONE cell, while buildings blocked
--- hundreds. Reported by the player in the same words - the planned path "differs hardly at all
--- from the field course", arrives at the harvester again and runs through its left half and its
--- maize header.
------------------------------------------------------------------------------------------------------------------------
TestVehicleAsObstacle = {}

function TestVehicleAsObstacle:setUp()
    TestSetup.reset()
    self.savedCheck = AutoDrive.checkForVehiclesInBox
    self.seen = {}
    local seen = self.seen
    AutoDrive.checkForVehiclesInBox = function(box, excluded)
        seen.box = box
        seen.excluded = excluded
        return seen.answer == true
    end
end

function TestVehicleAsObstacle:tearDown()
    AutoDrive.checkForVehiclesInBox = self.savedCheck
end

local function cellWithCorners()
    local cell = { x = 0, z = 0, shapeDefinition = { y = 42 } }
    local corners = {
        { x = 0, z = 0 }, { x = 4, z = 0 }, { x = 0, z = 4 }, { x = 4, z = 4 },
    }
    return cell, corners
end

function TestVehicleAsObstacle:testAStandingVehicleBlocksTheCell()
    local p = pfm()
    self.seen.answer = true
    local cell, corners = cellWithCorners()

    lu.assertTrue(p:vehicleBlockingCell(cell, corners))
end

function TestVehicleAsObstacle:testAnEmptyCellIsNotBlocked()
    local p = pfm()
    self.seen.answer = false
    local cell, corners = cellWithCorners()

    lu.assertFalse(p:vehicleBlockingCell(cell, corners))
end

--- The corners are built in the plane and the check compares heights, so the ground height has to be
--- filled in. Without it the first vehicle within fifty metres makes it error on nil.
function TestVehicleAsObstacle:testTheBoxCarriesTheGroundHeight()
    local p = pfm()
    local cell, corners = cellWithCorners()

    p:vehicleBlockingCell(cell, corners)

    for _, corner in ipairs(self.seen.box) do
        lu.assertEquals(corner.y, 42)
    end
end

function TestVehicleAsObstacle:testOurOwnRigIsNotAnObstacleToItself()
    local p = pfm()
    local cell, corners = cellWithCorners()

    p:vehicleBlockingCell(cell, corners)

    lu.assertEquals(self.seen.excluded[1], p.vehicle)
end

--- Same rule as collisionTestCallback: a path to a harvester's pipe ends alongside the harvester, so
--- counting it as a wall walls in the destination.
function TestVehicleAsObstacle:testTheVehicleWeAreDrivingToIsSpared()
    local p = pfm()
    p.targetVehicle = TestSetup.vehicle({ id = 99 })
    local cell, corners = cellWithCorners()

    p:vehicleBlockingCell(cell, corners)

    lu.assertEquals(#self.seen.excluded, 2)
    lu.assertEquals(self.seen.excluded[2], p.targetVehicle)
end

function TestVehicleAsObstacle:testWithoutATargetOnlyOurselvesAreSpared()
    local p = pfm()
    local cell, corners = cellWithCorners()

    p:vehicleBlockingCell(cell, corners)

    lu.assertEquals(#self.seen.excluded, 1)
end


------------------------------------------------------------------------------------------------------------------------
--- The corner the RIG can hold, not the one the tractor could
---
--- getDriverRadius asks the engine for getAttachedImplementsMaxTurnRadius, which reads the
--- aiTurnRadiusLimitation a vehicle declares in its XML. Trailers almost never declare one, so it
--- answers -1 and the planned radius is the TRACTOR's. Reported from the game twice: the trailer
--- reaches the tractor in the bend, first while reversing and then driving forwards too, and the rig
--- stops following the course it was given.
------------------------------------------------------------------------------------------------------------------------
TestRigTurnRadius = {}

function TestRigTurnRadius:setUp()
    TestSetup.reset()
    self.savedLength = AutoDrive.getTractorTrainLength
end

function TestRigTurnRadius:tearDown()
    AutoDrive.getTractorTrainLength = self.savedLength
end

--- A tractor at radius R pulls its trailer at a hitch angle of about atan(L / R), so the tightest
--- radius it can hold is L / tan(max angle).
function TestRigTurnRadius:testATrailerNeedsAWiderCornerThanTheTractor()
    local p = pfm()
    p.vehicle.size = { length = 6, width = 3 }
    AutoDrive.getTractorTrainLength = function() return 16 end   -- 10 m of trailer

    local expected = 10 / math.tan(math.rad(ADSpecialDrivingModule.MAX_TRAILER_ANGLE))
    lu.assertAlmostEquals(p:getTrainTurnRadius(), expected, 0.001)
    lu.assertTrue(p:getTrainTurnRadius() > 8, 'and that is wider than the tractor alone')
end

function TestRigTurnRadius:testASoloTractorKeepsItsOwnRadius()
    local p = pfm()
    p.vehicle.size = { length = 6, width = 3 }
    AutoDrive.getTractorTrainLength = function() return 6 end

    lu.assertEquals(p:getTrainTurnRadius(), 0, 'nothing towed, nothing to widen for')
end

function TestRigTurnRadius:testAnUnmeasurableRigDoesNotWidenAnything()
    local p = pfm()
    p.vehicle.size = nil
    AutoDrive.getTractorTrainLength = function() return 16 end

    lu.assertEquals(p:getTrainTurnRadius(), 0)
end

--- A longer trailer needs a wider corner, monotonically. The rule is the point, not the constant.
function TestRigTurnRadius:testALongerTrailerNeedsAWiderCorner()
    local p = pfm()
    p.vehicle.size = { length = 6, width = 3 }
    AutoDrive.getTractorTrainLength = function() return 12 end
    local short = p:getTrainTurnRadius()
    AutoDrive.getTractorTrainLength = function() return 22 end

    lu.assertTrue(p:getTrainTurnRadius() > short)
end


--- And the planner has to actually USE it. reset() is where the search picks its cell size, and the
--- rig's requirement has to beat the tractor's radius there or none of the above reaches a path.
function TestRigTurnRadius:testResetPlansWithTheWiderOfTheTwo()
    local savedRadius = AutoDrive.getDriverRadius
    local savedSetting = AutoDrive.getSetting
    local savedMask = AutoDrive.collisionMaskTerrain
    AutoDrive.getDriverRadius = function() return 7 end
    AutoDrive.getSetting = function(name) if name == 'Pathfinder' then return 1 end return 0 end
    AutoDrive.collisionMaskTerrain = 0
    AutoDrive.getTractorTrainLength = function() return 16 end

    local p = setmetatable({}, { __index = PathFinderModule })
    p.vehicle = TestSetup.vehicle()
    p.vehicle.size = { length = 6, width = 3, height = 4 }
    p:reset()

    AutoDrive.getDriverRadius = savedRadius
    AutoDrive.getSetting = savedSetting
    AutoDrive.collisionMaskTerrain = savedMask

    lu.assertAlmostEquals(p.minTurnRadius, 10 / math.tan(math.rad(ADSpecialDrivingModule.MAX_TRAILER_ANGLE)), 0.001,
        'the rig needs a wider corner than the tractor, so the rig decides')
    lu.assertTrue(p.minTurnRadius > 7)
end

--- And the same for the other pathfinder, which plans tighter still.
function TestRigTurnRadius:testTheOldPathfinderAlsoRespectsTheRig()
    local savedRadius = AutoDrive.getDriverRadius
    local savedSetting = AutoDrive.getSetting
    local savedMask = AutoDrive.collisionMaskTerrain
    AutoDrive.getDriverRadius = function() return 7 end
    AutoDrive.getSetting = function(name) if name == 'Pathfinder' then return 0 end return 0 end
    AutoDrive.collisionMaskTerrain = 0
    AutoDrive.getTractorTrainLength = function() return 16 end

    local p = setmetatable({}, { __index = PathFinderModule })
    p.vehicle = TestSetup.vehicle()
    p.vehicle.size = { length = 6, width = 3, height = 4 }
    p:reset()

    AutoDrive.getDriverRadius = savedRadius
    AutoDrive.getSetting = savedSetting
    AutoDrive.collisionMaskTerrain = savedMask

    lu.assertTrue(p.minTurnRadius > 7 * 2 / 3,
        'two thirds of the tractor radius is tighter still, and the trailer does not care')
end


------------------------------------------------------------------------------------------------------------------------
--- The cell test has to ask. A rule nothing calls is not a rule.
------------------------------------------------------------------------------------------------------------------------
TestCellConsultsVehicles = {}

function TestCellConsultsVehicles:setUp()
    TestSetup.reset()
    self.saved = {
        onField = AutoDrive.checkIsOnField,
        inBox = AutoDrive.checkForVehiclesInBox,
        pathInBox = AutoDrive.checkForVehiclePathInBox,
        overlap = _G.overlapBox,
    }
    AutoDrive.checkIsOnField = function() return true end
    AutoDrive.checkForVehiclePathInBox = function() return false end
    _G.overlapBox = function() return 0 end
end

function TestCellConsultsVehicles:tearDown()
    AutoDrive.checkIsOnField = self.saved.onField
    AutoDrive.checkForVehiclesInBox = self.saved.inBox
    AutoDrive.checkForVehiclePathInBox = self.saved.pathInBox
    _G.overlapBox = self.saved.overlap
end

--- A pathfinder with every OTHER check in isDriveableAstar neutralised, so the only thing that can
--- refuse the cell is a vehicle standing in it.
local function driveable()
    local p = pfm()
    p.restrictToField = false
    p.avoidFruitSetting = false
    p.isSecondChasingVehicle = false
    p.vectorX = { x = 1, z = 0 }
    p.vectorZ = { x = 0, z = 1 }
    p.collisionhits = 0
    p.getShapeDefByDirectionType_New = function(_, cell)
        return { x = cell.x, y = 0, z = cell.z, angleRad = 0, widthX = 1, widthZ = 1, height = 3 }
    end
    p.checkSlopeAngle = function() return false, 0 end
    p.getOffTrackingOffset = function() return nil end
    p.checkForFruitInArea = function() end
    return p
end

function TestCellConsultsVehicles:testACellWithAVehicleStandingInItIsRefused()
    local p = driveable()
    AutoDrive.checkForVehiclesInBox = function() return true end
    local cell = { x = 2, z = 0, from_node = { x = 1, z = 0 } }

    p:isDriveableAstar(cell)

    lu.assertTrue(cell.isRestricted,
        'a machine parked on the route is something to plan around, not to arrive at')
    lu.assertTrue(cell.hasCollision)
end

function TestCellConsultsVehicles:testAnEmptyCellIsStillDriveable()
    local p = driveable()
    AutoDrive.checkForVehiclesInBox = function() return false end
    local cell = { x = 2, z = 0, from_node = { x = 1, z = 0 } }

    p:isDriveableAstar(cell)

    lu.assertFalse(cell.isRestricted)
end


os.exit(lu.LuaUnit.run())
