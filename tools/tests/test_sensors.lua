--[[
Tests for scripts/Sensors/VirtualSensors.lua.

Covers findings:
  P4  handleSensors ran for every AutoDrive capable vehicle every frame, even with the driver off
  K3  every sensor was sized from vehicle.size, i.e. from the tractor, ignoring the trailer
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('VirtualSensors')
require('CollSensor')
require('CollSensorSplit')

local function sensorVehicle(overrides)
    local v = TestSetup.vehicle(overrides)
    v.ad.sensors = {}
    return v
end

--- Minimal sensor double: records whether it was updated and whether it was reset.
local function fakeSensor()
    return {
        updates = 0,
        triggered = true,
        updateSensor = function(self) self.updates = self.updates + 1 end,
        setTriggered = function(self, value) self.triggered = value end,
    }
end

------------------------------------------------------------------------------------------------------------------------
--- P4 - inactive vehicles must not pay for 20 sensors per frame
------------------------------------------------------------------------------------------------------------------------
TestSensorScheduling = {}

function TestSensorScheduling:setUp()
    TestSetup.reset()
    AutoDrive.isEditorModeEnabled = function() return false end
end

function TestSensorScheduling:testInactiveVehicleSkipsSensorUpdates()
    local v = sensorVehicle()
    v.ad.stateModule = { isActive = function() return false end }
    local s = fakeSensor()
    v.ad.sensors.front = s

    for _ = 1, 10 do
        ADSensor:handleSensors(v, 16)
    end
    lu.assertEquals(s.updates, 0, 'an inactive driver must not update its sensors')
end

function TestSensorScheduling:testActiveVehicleUpdatesSensors()
    local v = sensorVehicle()
    v.ad.stateModule = { isActive = function() return true end }
    local s = fakeSensor()
    v.ad.sensors.front = s

    ADSensor:handleSensors(v, 16)
    ADSensor:handleSensors(v, 16)
    lu.assertEquals(s.updates, 2)
end

--- Leaving a sensor triggered when the driver stops would keep a stale obstacle reported.
function TestSensorScheduling:testSensorsAreResetExactlyOnceWhenGoingIdle()
    local v = sensorVehicle()
    local active = true
    v.ad.stateModule = { isActive = function() return active end }
    local s = fakeSensor()
    v.ad.sensors.front = s

    ADSensor:handleSensors(v, 16)
    active = false
    ADSensor:handleSensors(v, 16)
    lu.assertFalse(s.triggered, 'sensors must be cleared when the driver goes idle')

    -- and the reset must not repeat on every subsequent frame
    s.triggered = 'untouched'
    for _ = 1, 5 do
        ADSensor:handleSensors(v, 16)
    end
    lu.assertEquals(s.triggered, 'untouched', 'the idle reset must happen once, not every frame')
end

function TestSensorScheduling:testEditorModeKeepsSensorsRunning()
    local v = sensorVehicle()
    v.ad.stateModule = { isActive = function() return false end }
    local s = fakeSensor()
    v.ad.sensors.front = s

    AutoDrive.isEditorModeEnabled = function() return true end
    ADSensor:handleSensors(v, 16)
    lu.assertEquals(s.updates, 1, 'the editor draws sensors, so they must keep updating')
end

function TestSensorScheduling:testSensorDebugChannelKeepsSensorsRunning()
    local v = sensorVehicle()
    v.ad.stateModule = { isActive = function() return false end }
    local s = fakeSensor()
    v.ad.sensors.front = s

    AutoDrive.currentDebugChannelMask = AutoDrive.DC_SENSORINFO
    ADSensor:handleSensors(v, 16)
    lu.assertEquals(s.updates, 1, 'the sensor debug channel draws sensors')
end

function TestSensorScheduling:testRecordingKeepsSensorsRunning()
    local v = sensorVehicle()
    v.ad.stateModule = { isActive = function() return false end }
    v.ad.recordingModule = { isRecording = true }
    local s = fakeSensor()
    v.ad.sensors.front = s

    ADSensor:handleSensors(v, 16)
    lu.assertEquals(s.updates, 1)
end

------------------------------------------------------------------------------------------------------------------------
--- K3 - sensors must be sized from the whole train
------------------------------------------------------------------------------------------------------------------------
TestSensorGeometry = {}

function TestSensorGeometry:setUp()
    TestSetup.reset()
end

function TestSensorGeometry:testFallsBackToVehicleDefinitionWhenNothingMeasured()
    local v = TestSetup.vehicle()
    v.size = { width = 3, length = 6, lengthOffset = 0 }
    local w, l = ADSensor.getTrainDimensions(v)
    lu.assertEquals(w, 3)
    lu.assertEquals(l, 6)
end

function TestSensorGeometry:testUsesMeasuredTrainWidthWhenWider()
    local v = TestSetup.vehicle()
    v.size = { width = 3, length = 6, lengthOffset = 0 }
    -- a 5.4 m wide trailer behind a 3 m tractor, and 12 m of train
    v.ad.adDimensions = { maxWidthLeft = 2.7, maxWidthRight = 2.7,
                          maxLengthFront = 3, maxLengthBack = 9 }
    local w, l = ADSensor.getTrainDimensions(v)
    lu.assertAlmostEquals(w, 5.4, 1e-9,
        'the sensor width must come from the measured train, not the tractor definition')
    lu.assertAlmostEquals(l, 12, 1e-9)
end

--- A narrow implement must never shrink the box below the tractor itself.
function TestSensorGeometry:testNeverShrinksBelowTheVehicleDefinition()
    local v = TestSetup.vehicle()
    v.size = { width = 3, length = 6, lengthOffset = 0 }
    v.ad.adDimensions = { maxWidthLeft = 0.5, maxWidthRight = 0.5,
                          maxLengthFront = 1, maxLengthBack = 1 }
    local w, l = ADSensor.getTrainDimensions(v)
    lu.assertEquals(w, 3)
    lu.assertEquals(l, 6)
end

function TestSensorGeometry:testPartialMeasurementIsIgnored()
    local v = TestSetup.vehicle()
    v.size = { width = 3, length = 6, lengthOffset = 0 }
    v.ad.adDimensions = { maxWidthLeft = 4 }  -- right side missing
    local w, l = ADSensor.getTrainDimensions(v)
    lu.assertEquals(w, 3, 'a half measured hull must not be trusted')
    lu.assertEquals(l, 6)
end

------------------------------------------------------------------------------------------------------------------------
--- A collision sensor has to answer with the scan it just ran
---
--- Both collision sensors latched the PREVIOUS scan's outcome at the top of onUpdate and only then
--- ran the new one. That is harmless when a sensor is updated every frame - and these are not. They
--- are only actually run once per ADSensor.EXECUTION_DELAY polls, so the deferral cost a full ten
--- frames on top of that throttle, measured at exactly ten in every phase of the throttle lattice.
--- At 60 fps that is a sixth of a second of continuing to drive at a closed gate, a fence or a tree
--- at a field entrance - obstacles for which this is the only thing looking.
---
--- overlapBox is synchronous, which the mod itself relies on elsewhere by reading its hit count on
--- the very next line, so there was nothing to wait for.
------------------------------------------------------------------------------------------------------------------------
TestSensorLatch = {}

local savedOverlapBox

function TestSensorLatch:setUp()
    TestSetup.reset()
    savedOverlapBox = overlapBox
end

function TestSensorLatch:tearDown()
    overlapBox = savedOverlapBox
end

--- The scan finds something, or does not, according to `hit`.
local function scanFinds(hit)
    overlapBox = function(_, _, _, _, _, _, _, _, _, _, target)
        if target ~= nil then target.newHit = hit end
    end
end

local function splitSensor()
    local s = setmetatable({}, { __index = ADCollSensorSplit })
    s.vehicle = TestSetup.vehicle()
    s.sensorParameters = { minDynamicLengthForVehicles = 5 }
    s.triggered = false
    s.setTriggered = function(self, value) self.triggered = value end
    s.getBoxShapes = function() return { { x = 0, y = 0, z = 0, rx = 0, ry = 0, size = { 1, 1, 1 } } } end
    s.onDrawDebug = function() end
    return s
end

local function plainSensor()
    local s = setmetatable({}, { __index = ADCollSensor })
    s.vehicle = TestSetup.vehicle()
    s.triggered = false
    s.setTriggered = function(self, value) self.triggered = value end
    s.getMask = function() return 0 end
    s.getBoxShape = function() return { x = 0, y = 0, z = 0, rx = 0, ry = 0, size = { 1, 1, 1 } } end
    s.onDrawDebug = function() end
    return s
end

function TestSensorLatch:testTheSplitSensorReportsWhatItJustFound()
    local s = splitSensor()
    scanFinds(true)

    s:onUpdate(16)

    lu.assertTrue(s.triggered,
        'the obstacle is in the box during this very scan - waiting for the next one costs ten frames')
end

function TestSensorLatch:testTheSplitSensorClearsInTheSameUpdate()
    local s = splitSensor()
    scanFinds(true)
    s:onUpdate(16)
    scanFinds(false)

    s:onUpdate(16)

    lu.assertFalse(s.triggered, 'and it must let go of an obstacle as promptly as it took it up')
end

function TestSensorLatch:testThePlainSensorReportsWhatItJustFound()
    local s = plainSensor()
    scanFinds(true)

    s:onUpdate(16)

    lu.assertTrue(s.triggered)
end

function TestSensorLatch:testThePlainSensorClearsInTheSameUpdate()
    local s = plainSensor()
    scanFinds(true)
    s:onUpdate(16)
    scanFinds(false)

    s:onUpdate(16)

    lu.assertFalse(s.triggered)
end


------------------------------------------------------------------------------------------------------------------------
--- Both front sensors read the same measurement
---
--- The long one asks ADSensor.getTrainDimensions. The split one asked
--- AutoDrive.getVehicleDimensions(vehicle, false), which overwrites adDimensions.width and .length
--- from vehicle.size before returning - so it could only ever hand back the vehicle definition, and
--- the measured hull never reached the box the driver is actually stopped on. It spanned exactly
--- vehicle.size.width in every configuration tried.
---
--- Nothing covered either call site: reverting getTrainDimensions to vehicle.size at its only
--- consumer left the whole gate green, because the geometry tests call the function directly and
--- the scheduling tests preset v.ad.sensors so the sensors are never built.
------------------------------------------------------------------------------------------------------------------------
TestSensorWidthSource = {}

function TestSensorWidthSource:setUp() TestSetup.reset() end

local function measuredVehicle()
    local v = TestSetup.vehicle()
    v.size = { width = 3, length = 6, lengthOffset = 0 }
    v.ad.adDimensions = { maxWidthLeft = 1.8, maxWidthRight = 1.8,
                          maxLengthFront = 3, maxLengthBack = 9 }
    v.lastSpeedReal = 0
    v.rotatedTime = 0
    return v
end

function TestSensorWidthSource:testTheLongSensorIsBuiltFromTheMeasurement()
    local v = measuredVehicle()
    local sensor = setmetatable({ vehicle = v }, { __index = ADSensor })
    sensor.getLocationByPosition = function() return { x = 0, y = 0, z = 3 } end

    sensor:loadBaseParameters()

    lu.assertAlmostEquals(sensor.width, 3.6 * ADSensor.WIDTH_FACTOR, 1e-9,
        'the measured hull is wider than the definition, so it is what the box is built from')
end

--- The split sensor divides its width into five boxes, so the covered span is what to compare.
local function splitSpan(vehicle)
    local sensor = setmetatable({ vehicle = vehicle }, { __index = ADCollSensorSplit })
    -- what loadBaseParameters would set; it is not called here because getLocationByPosition reaches
    -- into the vehicle's specs, and the box height reaches into the engine's density maps
    sensor.location = { x = 0, y = 0, z = 3 }
    sensor.position = ADSensor.POS_FRONT
    sensor.frontFactor = 1
    sensor.sideFactor = 1
    AutoDrive.checkIsOnField = function() return false end
    local boxes = sensor:getBoxShapes(2)
    lu.assertNotNil(boxes[1], 'test setup: it has to build a box at all')
    local span = 0
    for _, box in ipairs(boxes) do
        span = span + box.size[1] * 2
    end
    return span
end

function TestSensorWidthSource:testTheSplitSensorReadsTheSameMeasurement()
    local v = measuredVehicle()

    lu.assertAlmostEquals(ADSensor.getTrainDimensions(v), 3.6, 1e-9,
        'test setup: the measured hull is the wider of the two')
    lu.assertAlmostEquals(splitSpan(v), 3.6, 1e-6,
        'the box the driver is stopped on has to span the measured hull, not the vehicle definition')
end

--- And with nothing measured yet, it falls back to the definition rather than to nothing.
function TestSensorWidthSource:testTheSplitSensorFallsBackToTheVehicleDefinition()
    local v = measuredVehicle()
    v.ad.adDimensions = nil

    lu.assertAlmostEquals(splitSpan(v), 3, 1e-6)
end

os.exit(lu.LuaUnit.run())
