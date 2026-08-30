--[[
How big the rig physically is, as opposed to how much of it can be filled.

Two questions that look alike and are not. AutoDrive.getAllUnits walks the implement chain through
getTrailersOfImplement, which only accepts a unit that answers getFillUnits - right for fill levels,
and the definition of the physical train everywhere else. FS25 installs fillUnit through the
baseFillable vehicle type, and two base game families do not inherit it: livestockTrailer (Kingston
Belvedere, Fliegl Noah TTW 140, Wilson Silverstar) and dolly (Krampe Dolly 10 L). So hauling animals
measured the rig as a bare tractor, and a dolly rig lost the dolly out of its own middle.

That length is the pathfinder's collision clearance, the step-aside reach, and the count that decides
whether a rig may be steered while reversing rather than having its joints locked.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('ExternalInterface')
require('TrailerUtil')
require('TrailerModule')

--- A tractor with a chain of things hitched behind it. `fillable` is the whole point: it decides
--- whether a unit answers getFillUnits, which is exactly what separates a tipper from a livestock
--- trailer in the shipped game.
local function rig(tractorLength, towed)
    local tractor = { size = { length = tractorLength, width = 3 } }
    tractor.getRootVehicle = function() return tractor end
    local last = tractor
    for _, spec in ipairs(towed) do
        local unit = { size = { length = spec.length, width = 3 }, typeDesc = spec.typeDesc }
        unit.getRootVehicle = function() return tractor end
        if spec.fillable then
            unit.getFillUnits = function() return {} end
        end
        local attached = { { object = unit } }
        last.getAttachedImplements = function() return attached end
        last = unit
    end
    last.getAttachedImplements = function() return {} end
    return tractor
end

TestTrainUnits = {}

function TestTrainUnits:setUp()
    TestSetup.reset()
end

--- The control: a tipper has a fill unit, so both walks have always agreed about it.
function TestTrainUnits:testAFillableTrailerCountsInBothWalks()
    local r = rig(6, { { length = 8, fillable = true } })

    local _, fillable = AutoDrive.getAllUnits(r)
    local _, towed = AutoDrive.getAllTowedUnits(r)

    lu.assertEquals(fillable, 2)
    lu.assertEquals(towed, 2)
    lu.assertEquals(AutoDrive.getTractorTrainLength(r, true, false), 14)
end

--- A livestock trailer has no fill unit, and fifteen metres of it used to weigh nothing.
function TestTrainUnits:testALivestockTrailerStillHasALength()
    local r = rig(6, { { length = 15.3, fillable = false } })

    lu.assertEquals(select(2, AutoDrive.getAllUnits(r)), 1,
        'test setup: the fill-capable walk is meant to skip it, and still does')
    lu.assertEquals(select(2, AutoDrive.getAllTowedUnits(r)), 2)
    lu.assertAlmostEquals(AutoDrive.getTractorTrainLength(r, true, false), 21.3, 0.001)
end

--- And a unit in the MIDDLE of the chain that has no fill unit must not swallow what is behind it.
function TestTrainUnits:testADollyDoesNotHideTheSemitrailerBehindIt()
    local r = rig(6, {
        { length = 4.65, fillable = false },   -- the dolly
        { length = 10.5, fillable = true },    -- the semitrailer on top of it
    })

    lu.assertEquals(select(2, AutoDrive.getAllTowedUnits(r)), 3)
    lu.assertAlmostEquals(AutoDrive.getTractorTrainLength(r, true, false), 21.15, 0.001)
end

--- Front loader tools are not being towed along behind and stay out, as they do in the other walk.
function TestTrainUnits:testAFrontLoaderToolIsNotPartOfTheTrain()
    local r = rig(6, { { length = 3, fillable = false, typeDesc = 'typeDesc_frontloaderTool' } })

    lu.assertEquals(select(2, AutoDrive.getAllTowedUnits(r)), 1)
    lu.assertEquals(AutoDrive.getTractorTrainLength(r, true, false), 6)
end

--- Steering while reversing is safe for a rigid pair and not for a double articulated train. The
--- count came off the fill-capable list, so the rigs that most need their joints locked - the ones
--- with an unfillable dolly in the middle - were the ones that reported they could be steered.
function TestTrainUnits:testADoubleArticulatedRigIsNotSteeredWhileReversing()
    local module = { vehicle = rig(6, {
        { length = 4.65, fillable = false },
        { length = 10.5, fillable = true },
    }) }

    lu.assertFalse(ADTrailerModule.canBeHandledInReverse(module))
end

function TestTrainUnits:testATractorAndOneTrailerStillIs()
    local module = { vehicle = rig(6, { { length = 8, fillable = true } }) }

    lu.assertTrue(ADTrailerModule.canBeHandledInReverse(module))
end


------------------------------------------------------------------------------------------------------------------------
--- When a load counts as finished
---
--- AutoDrive.isUnloadFillLevelReached is the single predicate behind filledToUnload for every mode -
--- getAllFillLevels, getObjectFillLevels, getALObjectFillLevels and getIsFillUnitFull all end in it -
--- and nothing executed it. The one suite that required TrailerUtil replaced its callers with stubs
--- on the next line, so the require was decorative: inverting the comparison left the whole gate
--- green while making every hauler leave the silo a quarter full and never recognise a full one.
------------------------------------------------------------------------------------------------------------------------
TestUnloadFillLevel = {}

function TestUnloadFillLevel:setUp()
    TestSetup.reset()
    AutoDrive.testSettings['unloadFillLevel'] = 0.85
    self.object = { getRootVehicle = function(self) return self end }
end

--- fillFreeCapacity is what the rule reads, so these are stated the way the callers pass them.
local function reached(object, level, capacity)
    return AutoDrive.isUnloadFillLevelReached(object, level, capacity - level, capacity)
end

function TestUnloadFillLevel:testAQuarterFullIsNotEnough()
    lu.assertFalse(reached(self.object, 5000, 20000))
end

function TestUnloadFillLevel:testJustUnderTheSettingIsNotEnough()
    lu.assertFalse(reached(self.object, 16800, 20000), '84 % against a setting of 85 %')
end

function TestUnloadFillLevel:testTheSettingIsReached()
    lu.assertTrue(reached(self.object, 17000, 20000), '85 % against a setting of 85 %')
end

function TestUnloadFillLevel:testAFullTrailerIsReached()
    lu.assertTrue(reached(self.object, 20000, 20000))
end

--- Empty is never "full", however the capacity is stated.
function TestUnloadFillLevel:testAnEmptyTrailerIsNeverReached()
    lu.assertFalse(reached(self.object, 0, 20000))
    lu.assertFalse(reached(self.object, 0, 0))
end

--- The setting moves the line, which is the whole point of it being a setting.
function TestUnloadFillLevel:testTheSettingMovesTheLine()
    AutoDrive.testSettings['unloadFillLevel'] = 0.25
    lu.assertTrue(reached(self.object, 5000, 20000),
        'a quarter full is enough once the driver is told a quarter is enough')
end


------------------------------------------------------------------------------------------------------------------------
--- Towed means towed
---
--- The walk counted everything hanging off the vehicle and excluded only front loader and wheel
--- loader TOOLS, by typeDesc. Everything else at the front came along: a weight, a front tank, a
--- front mounted implement. Measured in game on a tractor carrying a front weight and pulling a
--- trailer: three units where there are two, an angle to "the first towed unit" of 0.0 degrees
--- because that unit is bolted to the nose and points exactly where the tractor does, and
--- canBeHandledInReverse counting 3 against its limit of 2 - which put the rig on the blind reverse
--- that steers nothing at all. The trailer folding into the tractor followed from there.
------------------------------------------------------------------------------------------------------------------------
TestTowedIsBehind = {}

function TestTowedIsBehind:setUp()
    TestSetup.reset()
end

--- A rig whose units have real positions, so the behind test can actually run. zOffset is metres
--- along the tractor's own axis: negative is behind it, positive in front.
local function positionedRig(towed)
    local tractor = { size = { length = 6, width = 3 }, components = { { node = 'root' } } }
    MockEngine.nodePositions['root'] = { x = 0, y = 0, z = 0 }
    tractor.getRootVehicle = function() return tractor end

    local attached = {}
    for i, spec in ipairs(towed) do
        local node = 'unit' .. i
        MockEngine.nodePositions[node] = { x = 0, y = 0, z = spec.zOffset }
        local unit = {
            size = { length = spec.length or 8, width = 3 },
            components = { { node = node } },
            getRootVehicle = function() return tractor end,
            getAttachedImplements = function() return {} end,
        }
        attached[#attached + 1] = { object = unit }
    end
    tractor.getAttachedImplements = function() return attached end
    return tractor
end

--- The reported rig: a weight on the nose and a trailer behind.
function TestTowedIsBehind:testAFrontWeightIsNotPartOfTheTrain()
    local r = positionedRig({
        { zOffset = 3, length = 1 },     -- the weight, in front
        { zOffset = -9, length = 10 },   -- the trailer, behind
    })

    lu.assertEquals(select(2, AutoDrive.getAllTowedUnits(r)), 2,
        'a weight bolted to the nose is not being towed')
    lu.assertEquals(AutoDrive.getTractorTrainLength(r, true, false), 16,
        'and its length is not part of the train')
end

--- Which is what decides whether the rig may be steered while reversing at all.
function TestTowedIsBehind:testAFrontWeightDoesNotForceTheBlindReverse()
    local module = { vehicle = positionedRig({
        { zOffset = 3, length = 1 },
        { zOffset = -9, length = 10 },
    }) }

    lu.assertTrue(ADTrailerModule.canBeHandledInReverse(module),
        'tractor and one trailer may be steered backwards, weight or no weight')
end

--- What is genuinely behind still counts, however much of it there is.
function TestTowedIsBehind:testEverythingBehindStillCounts()
    local r = positionedRig({
        { zOffset = -5, length = 4.65 },
        { zOffset = -14, length = 10.5 },
    })

    lu.assertEquals(select(2, AutoDrive.getAllTowedUnits(r)), 3)
end

--- And a unit that cannot be measured is counted, which is the safe direction for the length
--- callers and is what the walk did before there was a test at all.
function TestTowedIsBehind:testAnUnmeasurableUnitIsStillCounted()
    local tractor = { size = { length = 6, width = 3 }, components = { { node = 'root' } } }
    MockEngine.nodePositions['root'] = { x = 0, y = 0, z = 0 }
    tractor.getRootVehicle = function() return tractor end
    local ghost = { size = { length = 8, width = 3 }, getAttachedImplements = function() return {} end }
    tractor.getAttachedImplements = function() return { { object = ghost } } end

    lu.assertEquals(select(2, AutoDrive.getAllTowedUnits(tractor)), 2)
end

os.exit(lu.LuaUnit.run())
