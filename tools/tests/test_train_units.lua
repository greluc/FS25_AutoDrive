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

os.exit(lu.LuaUnit.run())
