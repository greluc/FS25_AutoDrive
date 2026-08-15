--[[
Tests for the Courseplay interface, AutoDrive side.

Covers findings:
  C1 an AutoDrive unloader could not ask a Courseplay harvester to tolerate it nearby
  C2 Courseplay could not ask an AutoDrive unloader to back out of a reversing harvester
  C3 there was no version handshake, so a renamed function failed silently
  C4 the four harvester queries polled every frame instead of being pushed on change

The rule every one of these has to satisfy: a player running a Courseplay WITHOUT the counterpart
must see exactly the old behaviour. Mixed installations are the normal case, not an edge case.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('ExternalInterface')

TestInterfaceVersion = {}

function TestInterfaceVersion:testVersionIsPublished()
    -- read from the mod source by test-setup, so this fails if the constant is dropped
    lu.assertNotNil(AutoDrive.INTERFACE_VERSION,
        'AutoDrive.INTERFACE_VERSION must exist - Courseplay reads it to negotiate capabilities')
    lu.assertTrue(AutoDrive.INTERFACE_VERSION >= 1)
end

------------------------------------------------------------------------------------------------------------------------
--- C1 - proximity request
------------------------------------------------------------------------------------------------------------------------
TestProximityRequest = {}

function TestProximityRequest:setUp()
    TestSetup.reset()
end

function TestProximityRequest:testForwardsToTheHarvester()
    local asked = nil
    local harvester = TestSetup.vehicle()
    harvester.cpRequestToIgnoreProximity = function(_, v) asked = v; return true end

    local unloader = TestSetup.vehicle()
    lu.assertTrue(AutoDrive:requestCourseplayProximity(unloader, harvester))
    lu.assertEquals(asked, unloader)
end

function TestProximityRequest:testUsesTheRootVehicle()
    local root = TestSetup.vehicle()
    local asked = false
    root.cpRequestToIgnoreProximity = function() asked = true; return true end

    local attached = TestSetup.vehicle()
    attached.cpRequestToIgnoreProximity = nil
    attached.getRootVehicle = function() return root end

    AutoDrive:requestCourseplayProximity(TestSetup.vehicle(), attached)
    lu.assertTrue(asked, 'the request must go to the root vehicle, where the CP spec lives')
end

--- The mixed-installation case: no counterpart, no error, no pretending it worked.
function TestProximityRequest:testCourseplayWithoutTheInterfaceIsHarmless()
    local harvester = TestSetup.vehicle()   -- no cpRequestToIgnoreProximity
    lu.assertFalse(AutoDrive:requestCourseplayProximity(TestSetup.vehicle(), harvester))
end

function TestProximityRequest:testNilArgumentsAreHarmless()
    lu.assertFalse(AutoDrive:requestCourseplayProximity(nil, TestSetup.vehicle()))
    lu.assertFalse(AutoDrive:requestCourseplayProximity(TestSetup.vehicle(), nil))
end

------------------------------------------------------------------------------------------------------------------------
--- C2 - backup request coming the other way
------------------------------------------------------------------------------------------------------------------------
TestBackupRequest = {}

function TestBackupRequest:setUp()
    TestSetup.reset()
    g_time = 1000
end

function TestBackupRequest:testActiveUnloaderAcceptsTheRequest()
    local unloader = TestSetup.vehicle()
    unloader.ad.stateModule = { isActive = function() return true end }
    local released = false
    unloader.ad.specialDrivingModule = { releaseVehicle = function() released = true end }

    lu.assertTrue(AutoDrive:requestBackupForReversingCombine(unloader, TestSetup.vehicle()))
    lu.assertEquals(unloader.ad.reverseForCombineRequest, 1000)
    lu.assertTrue(released, 'the unloader must be released so it can move out of the way')
end

function TestBackupRequest:testInactiveUnloaderDeclines()
    local unloader = TestSetup.vehicle()
    unloader.ad.stateModule = { isActive = function() return false end }
    lu.assertFalse(AutoDrive:requestBackupForReversingCombine(unloader, TestSetup.vehicle()))
end

function TestBackupRequest:testForeignVehicleDeclines()
    lu.assertFalse(AutoDrive:requestBackupForReversingCombine({}, TestSetup.vehicle()))
    lu.assertFalse(AutoDrive:requestBackupForReversingCombine(nil, TestSetup.vehicle()))
end

------------------------------------------------------------------------------------------------------------------------
--- C4 - pushed state replaces polling, but polling stays for older Courseplay
------------------------------------------------------------------------------------------------------------------------
TestPushedState = {}

function TestPushedState:setUp()
    TestSetup.reset()
end

local function harvesterWithPushedState(state)
    local v = TestSetup.vehicle()
    AutoDrive.onCpHarvesterStateChanged(v, state)
    return v
end

function TestPushedState:testQueriesReadThePushedState()
    local v = harvesterWithPushedState({
        active = true, waitingForUnload = true, inPocket = false, maneuvering = false })
    lu.assertTrue(AutoDrive:getIsCPWaitingForUnload(v))
    lu.assertFalse(AutoDrive:getIsCPTurning(v))
    lu.assertFalse(AutoDrive:getIsCPCombineInPocket(v))
end

function TestPushedState:testManeuveringAndPocketAreDistinct()
    local v = harvesterWithPushedState({
        active = true, waitingForUnload = false, inPocket = true, maneuvering = true })
    lu.assertFalse(AutoDrive:getIsCPWaitingForUnload(v))
    lu.assertTrue(AutoDrive:getIsCPTurning(v))
    lu.assertTrue(AutoDrive:getIsCPCombineInPocket(v))
end

--- The closing event: the strategy only lives for one job, so an inactive push must clear
--- everything. Without this the last state would outlive the harvester's work.
function TestPushedState:testInactiveStateClearsEverything()
    local v = harvesterWithPushedState({
        active = true, waitingForUnload = true, inPocket = true, maneuvering = true })
    AutoDrive.onCpHarvesterStateChanged(v, {
        active = false, waitingForUnload = false, inPocket = false, maneuvering = false })
    lu.assertFalse(AutoDrive:getIsCPWaitingForUnload(v))
    lu.assertFalse(AutoDrive:getIsCPTurning(v))
    lu.assertFalse(AutoDrive:getIsCPCombineInPocket(v))
end

--- Mixed installation: nothing was ever pushed, so the queries must poll as before.
function TestPushedState:testFallsBackToPollingWhenNothingIsPushed()
    local v = TestSetup.vehicle()
    local polled = 0
    v.getIsCpActive = function() return true end
    v.getIsCpHarvesterWaitingForUnload = function() polled = polled + 1; return true end

    lu.assertNil(AutoDrive:getPushedCpState(v), 'nothing pushed yet')
    lu.assertTrue(AutoDrive:getIsCPWaitingForUnload(v))
    lu.assertEquals(polled, 1, 'without a pushed state the query must ask Courseplay directly')
end

function TestPushedState:testPushedStateIsReadFromTheRootVehicle()
    local root = harvesterWithPushedState({
        active = true, waitingForUnload = true, inPocket = false, maneuvering = false })
    local attached = TestSetup.vehicle()
    attached.ad = nil
    attached.getRootVehicle = function() return root end
    lu.assertTrue(AutoDrive:getIsCPWaitingForUnload(attached))
end

function TestPushedState:testEventOnVehicleWithoutAdDataIsHarmless()
    local v = { ad = nil }
    lu.assertNil(AutoDrive.onCpHarvesterStateChanged(v, { active = true }))
end

os.exit(lu.LuaUnit.run())
