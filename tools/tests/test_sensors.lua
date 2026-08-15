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

os.exit(lu.LuaUnit.run())
