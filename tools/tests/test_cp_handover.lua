--[[
Handing a vehicle to Courseplay, and taking one back.

Three separate silences, all on the seam between the two mods, all with the same shape: AutoDrive
did something, Courseplay declined or was never asked, and nobody found out until a player walked
over to a machine standing in a field.

  StartCP / RestartCP called and walked away. Courseplay answers false on every failure path - no
  course, wrong implement, not on a field - and says WHY as a second return value. Both were
  dropped. The vehicle stood there with the button still lit.

  The manual refuel and repair buttons stop Courseplay and never restart it. StopCP clears the
  start-helper flag and the errand overwrites the mode with MODE_DRIVETO, which is not in
  modesToStartFromCP - so even once the flag is restored, the NEXT onCpFull reports Wrong_Mode and
  disarms the button. Both have to be put back, and the mode is the one that is easy to miss
  because nothing fails at the time.

  Being more than 30 m from the first marker skipped the handover entirely, without a word.

And the reverse direction: our unloader now tells a Courseplay harvester it is still coming, so the
harvester does not turn into the next row with its pipe in the fruit while we are on our way.
]]

lu = require('luaunit')
require('test-setup')
require('ExternalInterface')

if AutoDrive.debugPrint == nil then
    function AutoDrive.debugPrint() end
end

--- Helper ids only have to be distinct.
ADStateModule = ADStateModule or {}
ADStateModule.HELPER_CP = ADStateModule.HELPER_CP or 1
ADStateModule.HELPER_AIVE = ADStateModule.HELPER_AIVE or 2

AutoDrive.MODE_DRIVETO = AutoDrive.MODE_DRIVETO or 1
AutoDrive.MODE_PICKUPANDDELIVER = AutoDrive.MODE_PICKUPANDDELIVER or 2

--- Records what the player was told, since "the player is told something" is the whole point of
--- most of this file.
local notifications = {}
AutoDriveMessageEvent = AutoDriveMessageEvent or {}
function AutoDriveMessageEvent.sendMessageOrNotification(vehicle, messageType, text, duration, ...)
    table.insert(notifications, { vehicle = vehicle, messageType = messageType,
                                  text = string.format(text, ...) })
end

ADMessagesManager = ADMessagesManager or {}
ADMessagesManager.messageTypes = ADMessagesManager.messageTypes or { INFO = 1, WARN = 2, ERROR = 3 }

g_i18n = g_i18n or { getText = function(_, key) return key end }

--- A vehicle AutoDrive is driving on Courseplay's behalf.
local function cpVehicle(overrides)
    local v = TestSetup.vehicle()
    local state = {
        helper = ADStateModule.HELPER_CP,
        startHelper = true,
        mode = AutoDrive.MODE_PICKUPANDDELIVER,
    }
    v.ad.stateModule = {
        getUsedHelper = function() return state.helper end,
        getStartHelper = function() return state.startHelper end,
        setStartHelper = function(_, value) state.startHelper = value end,
        getMode = function() return state.mode end,
        setMode = function(_, value) state.mode = value end,
        getName = function() return 'TestDriver' end,
    }
    v.ad.restartCP = false
    v.state = state
    for k, val in pairs(overrides or {}) do
        if k == 'stateOverrides' then
            for sk, sv in pairs(val) do state[sk] = sv end
        else
            v[k] = val
        end
    end
    return v
end

TestCpHandover = {}

function TestCpHandover:setUp()
    TestSetup.reset()
    notifications = {}
    g_time = 10000
end

------------------------------------------------------------------------------------------------------------------------
--- Courseplay refusing the handover
------------------------------------------------------------------------------------------------------------------------

function TestCpHandover:testAStartCourseplayAcceptsIsLeftAlone()
    local v = cpVehicle()
    v.startCpAtLastWp = function() return true end

    lu.assertTrue(AutoDrive:StartCP(v))
    lu.assertEquals(#notifications, 0, 'nothing went wrong, so the player is told nothing')
    lu.assertTrue(v.state.startHelper, 'the button stays lit while Courseplay is driving')
end

--- The case this whole file exists for.
function TestCpHandover:testARefusalIsReportedToThePlayer()
    local v = cpVehicle()
    v.ad.restartCP = true
    v.startCpAtLastWp = function() return false, 'Generate a course before starting the job.' end

    lu.assertFalse(AutoDrive:StartCP(v))
    lu.assertEquals(#notifications, 1, 'a refused handover has to reach the player')
    lu.assertNotNil(notifications[1].text:find('Generate a course', 1, true),
            "Courseplay's own reason has to be the one shown, not a generic one")
    lu.assertEquals(notifications[1].messageType, ADMessagesManager.messageTypes.ERROR)
end

--- The button has to go out. Leaving it lit means the HUD keeps claiming a helper is coming, and
--- getCanAdTakeControl keeps answering as though a handover were pending.
function TestCpHandover:testARefusalPutsTheButtonOut()
    local v = cpVehicle()
    v.ad.restartCP = true
    v.startCpAtLastWp = function() return false, 'nope' end

    AutoDrive:StartCP(v)
    lu.assertFalse(v.state.startHelper)
    lu.assertFalse(v.ad.restartCP, 'or we try the same doomed handover again at the next opportunity')
end

--- Courseplay may refuse without saying why - an older Courseplay, or a path that has no message.
--- The player still gets told something.
function TestCpHandover:testARefusalWithoutAReasonStillSaysSomething()
    local v = cpVehicle()
    v.startCpAtLastWp = function() return false end

    AutoDrive:StartCP(v)
    lu.assertEquals(#notifications, 1)
    lu.assertNotNil(notifications[1].text:find('AD_CP_could_not_start', 1, true))
end

--- No Courseplay at all, or one from before this interface. Not an error, and not a notification
--- either: there is nothing to report to a player who is not running Courseplay.
function TestCpHandover:testNoCourseplayIsNotAnError()
    local v = cpVehicle()
    lu.assertFalse(AutoDrive:StartCP(v))
    lu.assertEquals(#notifications, 0)
end

--- The old name, for a Courseplay that spells it the other way.
function TestCpHandover:testTheLegacySpellingStillWorks()
    local v = cpVehicle()
    v.startCpALastWp = function() return true end
    lu.assertTrue(AutoDrive:StartCP(v))
end

function TestCpHandover:testRestartReportsRefusalsTheSameWay()
    local v = cpVehicle()
    v.startCpAtLastWp = function() return false, 'not on a field' end
    lu.assertFalse(AutoDrive:RestartCP(v))
    lu.assertEquals(#notifications, 1)
    lu.assertNotNil(notifications[1].text:find('not on a field', 1, true))
end

function TestCpHandover:testNoVehicleIsHarmless()
    lu.assertFalse(AutoDrive:StartCP(nil))
    lu.assertFalse(AutoDrive:RestartCP(nil))
end

------------------------------------------------------------------------------------------------------------------------
--- Borrowing a Courseplay vehicle for an errand, and giving it back
------------------------------------------------------------------------------------------------------------------------

function TestCpHandover:testAnErrandRemembersCourseplayWasDriving()
    local v = cpVehicle()
    lu.assertTrue(AutoDrive:rememberCpBeforeErrand(v))
    lu.assertNotNil(v.ad.cpErrandReturn)
    lu.assertEquals(v.ad.cpErrandReturn.mode, AutoDrive.MODE_PICKUPANDDELIVER)
end

--- Both halves have to come back. The flag alone is not enough: MODE_DRIVETO is not in
--- modesToStartFromCP, so handleCPFieldWorker would report Wrong_Mode at the next onCpFull and
--- disarm the button - one errand later, with nothing connecting cause and effect.
function TestCpHandover:testTheErrandGivesBackBothTheFlagAndTheMode()
    local v = cpVehicle()
    AutoDrive:rememberCpBeforeErrand(v)

    -- what the errand does to it
    v.state.startHelper = false
    v.state.mode = AutoDrive.MODE_DRIVETO

    lu.assertTrue(AutoDrive:restoreCpAfterErrand(v))
    lu.assertTrue(v.state.startHelper, 'without the flag the handover gate never opens')
    lu.assertEquals(v.state.mode, AutoDrive.MODE_PICKUPANDDELIVER,
            'without the mode the next onCpFull reports Wrong_Mode and gives up for good')
    lu.assertTrue(v.ad.restartCP, 'and it should continue the course, not start it again')
end

--- A vehicle the player was driving with AutoDrive alone has nothing to give back, and must not be
--- pushed into Courseplay when its errand ends.
function TestCpHandover:testAVehicleCourseplayWasNotDrivingIsNotHandedToIt()
    local v = cpVehicle({ stateOverrides = { helper = ADStateModule.HELPER_AIVE } })
    lu.assertFalse(AutoDrive:rememberCpBeforeErrand(v))
    lu.assertNil(v.ad.cpErrandReturn)
    lu.assertFalse(AutoDrive:restoreCpAfterErrand(v))
    lu.assertFalse(v.ad.restartCP)
end

--- Courseplay assigned but switched off by the player: also nothing to resume.
function TestCpHandover:testAnAlreadyStoppedCourseplayIsNotRestarted()
    local v = cpVehicle({ stateOverrides = { startHelper = false } })
    lu.assertFalse(AutoDrive:rememberCpBeforeErrand(v))
    lu.assertFalse(AutoDrive:restoreCpAfterErrand(v))
end

--- Restoring twice must not resurrect a job. The promise is consumed.
function TestCpHandover:testTheHandbackHappensOnce()
    local v = cpVehicle()
    AutoDrive:rememberCpBeforeErrand(v)
    lu.assertTrue(AutoDrive:restoreCpAfterErrand(v))
    v.ad.restartCP = false
    lu.assertFalse(AutoDrive:restoreCpAfterErrand(v))
    lu.assertFalse(v.ad.restartCP)
end

------------------------------------------------------------------------------------------------------------------------
--- The wiring, read from the sources
------------------------------------------------------------------------------------------------------------------------

local function readSource(path)
    local file = assert(io.open('../../' .. path, 'r'), 'cannot read ' .. path)
    local text = file:read('*a')
    file:close()
    return text
end

--- Source with comment lines removed.
---
--- Every assertion below about what the code DOES has to read the code, not the prose around it.
--- Two of these tests passed and then failed on comments that mention the very function they were
--- looking for - one of them a comment explaining why the call is deliberately absent, which is
--- exactly the case the assertion existed to catch.
local function codeOnly(text)
    return (text:gsub('%-%-[^\n]*', ''))
end

--- The two temporary errands remember, and parking deliberately does not: a vehicle the player sent
--- to park is meant to stay parked.
function TestCpHandover:testOnlyTheTemporaryErrandsRememberCourseplay()
    local text = codeOnly(readSource('scripts/Manager/InputManager.lua'))
    for _, fn in ipairs({ 'input_refuelVehicle', 'input_repairVehicle' }) do
        local body = assert(text:match('function ADInputManager:' .. fn .. '.-\nend'), fn .. ' is gone')
        local remember = body:find('AutoDrive:rememberCpBeforeErrand(', 1, true)
        lu.assertNotNil(remember,
                fn .. ' takes the vehicle from Courseplay and never gives it back')
        local stop = body:find('AutoDrive:StopCP', 1, true)
        lu.assertTrue(remember < stop, fn .. ': StopCP clears the flag, so we have to read it first')
    end
    local park = assert(text:match('function ADInputManager:input_parkVehicle.-\nend'))
    lu.assertNil(park:find('AutoDrive:rememberCpBeforeErrand(', 1, true),
            'parking is not a temporary errand; a parked vehicle has to stay parked')
end

--- And both tasks give it back before stopAutoDrive, which is where the handover actually happens.
function TestCpHandover:testBothErrandTasksGiveTheVehicleBack()
    for _, task in ipairs({ 'RefuelTask', 'RepairTask' }) do
        local text = codeOnly(readSource('scripts/Tasks/' .. task .. '.lua'))
        local finished = assert(text:match('function ' .. task .. ':finished%(.-\nend'))
        local restore = finished:find('AutoDrive:restoreCpAfterErrand(', 1, true)
        local stop = finished:find('vehicle:stopAutoDrive()', 1, true)
        lu.assertNotNil(restore, task .. ' never hands the vehicle back to Courseplay')
        lu.assertNotNil(stop, task .. ':finished no longer stops AutoDrive')
        lu.assertTrue(restore < stop,
                task .. ': the handover happens inside stopAutoDrive, so restore before it')

        local abort = assert(text:match('function ' .. task .. ':abort%(.-\nend'))
        lu.assertNotNil(abort:find('cpErrandReturn', 1, true),
                task .. ': an aborted errand leaves a promise nobody will keep')
    end
end

--- Our unloader tells the harvester it is still coming, during the two states where it is.
function TestCpHandover:testTheUnloaderKeepsTheHarvesterWaiting()
    local text = codeOnly(readSource('scripts/Tasks/EmptyHarvesterTask.lua'))
    lu.assertNotNil(text:find('cpReconfirmUnloaderRendezvous', 1, true),
            'nothing tells a Courseplay harvester we are on our way, so it gives up after 30 s')
    local block = assert(text:match('STATE_PATHPLANNING or\n.-STATE_DRIVING then(.-)\n    end'))
    lu.assertNotNil(block:find('cpReconfirmUnloaderRendezvous', 1, true))
    lu.assertNotNil(block:find('self.combine.cpReconfirmUnloaderRendezvous ~= nil', 1, true),
            'a Courseplay without this function must not be a crash')
end

--- More than 30 m from the marker: still not a handover, but no longer a silent one.
function TestCpHandover:testBeingTooFarAwayIsReported()
    local text = codeOnly(readSource('scripts/Specialization.lua'))
    local body = assert(text:match('function AutoDrive.passToExternalMod_CP.-\nend'))
    lu.assertNotNil(body:find('distanceToStart >= 30', 1, true),
            'the too-far case is silent again')
    lu.assertNotNil(body:find('AD_CP_too_far_from_start', 1, true))
end

--- The isControlled hack has to be undone even when Courseplay throws. Leaving it false breaks the
--- player's own input on that vehicle for the rest of the session.
function TestCpHandover:testTheControlledFlagIsRestoredEvenOnAnError()
    local text = codeOnly(readSource('scripts/Specialization.lua'))
    local body = assert(text:match('function AutoDrive.passToExternalMod_CP.-\nend'))
    lu.assertNotNil(body:find('pcall', 1, true),
            'an error inside Courseplay skips the isControlled restore again')
    local call = body:find('pcall', 1, true)
    local restore = body:find('isControlled = isControlled', 1, true)
    lu.assertTrue(call < restore, 'the restore has to be after the guarded call, not inside it')
end

--- The version number is a label, never a gate. Deriving a capability from it would strand every
--- player who updates one mod and not the other.
function TestCpHandover:testTheInterfaceVersionIsDeclared()
    local text = readSource('scripts/AutoDrive.lua')
    lu.assertEquals(text:match('AutoDrive%.INTERFACE_VERSION = (%d+)'), '2',
            'the contract grew; the label has to say so')
    lu.assertNotNil(text:find('requestMakeWay', 1, true),
            'the contract comment no longer lists what version 2 added')
    lu.assertNotNil(text:find('cpReconfirmUnloaderRendezvous', 1, true))
end

------------------------------------------------------------------------------------------------------------------------
--- Events off the wire
------------------------------------------------------------------------------------------------------------------------

--- NetworkUtil.getObject answers nil for a vehicle that has been sold, or that this machine has not
--- finished creating. Both of these events dereferenced it without asking.
function TestCpHandover:testTheInputEventsSurviveAVehicleThatIsGone()
    for _, event in ipairs({ 'InputEvent', 'HudInputEvent' }) do
        local text = codeOnly(readSource('scripts/Events/' .. event .. '.lua'))
        local body = assert(text:match('function AutoDrive%w-Event:run%(connection%).-\nend'),
                event .. ': no run handler')
        lu.assertNotNil(body:find('self.vehicle == nil', 1, true),
                event .. ' still dereferences a vehicle that may have been sold')
    end
end

--- The farm id decides what the sender is allowed to do, and a client writes it into the stream
--- itself. Taking it from the connection instead is the difference between a permission check and a
--- suggestion.
--- Hiring a helper is what the game asks permission for. Gated only there, only where the
--- connection is known, and never on the stop.
function TestCpHandover:testOnlyHiringIsGatedByPermission()
    local text = codeOnly(readSource('scripts/Events/InputEvent.lua'))
    lu.assertNotNil(text:find("getHasPlayerPermission('hireAssistant', connection, farmId)", 1, true),
            'either the check is gone, or it no longer passes the connection - and without the ' ..
                    'connection getHasPlayerPermission answers true on the server no matter what')

    local names = assert(text:match('HIRING_INPUTS = {(.-)}'))
    for _, input in ipairs({ 'input_start_stop', 'input_parkVehicle',
                             'input_refuelVehicle', 'input_repairVehicle' }) do
        lu.assertNotNil(names:find(input, 1, true), input .. ' hires a helper and is not gated')
    end

    --- These names have to be the ones the dispatcher actually uses, or the gate matches nothing.
    local manager = readSource('scripts/Manager/InputManager.lua')
    -- [%w_]+, not %w+: %w stops at the underscore and input_start_stop would be read as input_start
    for input in names:gmatch('(input_[%w_]+)') do
        lu.assertNotNil(manager:find('"' .. input .. '"', 1, true),
                input .. ' is not an input AutoDrive knows, so the gate is dead')
    end

    --- And stopping stays ungated: the base game's own AIJobStopEvent has no check either.
    local stop = assert(codeOnly(readSource('scripts/Manager/InputManager.lua'))
            :match('function ADInputManager:input_start_stop.-\nend'))
    lu.assertNil(stop:find('getHasPlayerPermission', 1, true),
            'a player who may stop a helper by hand must be able to stop one through AutoDrive')
end

function TestCpHandover:testTheFarmIdComesFromTheConnectionNotTheWire()
    local text = codeOnly(readSource('scripts/Events/InputEvent.lua'))
    local body = assert(text:match('function AutoDriveInputEventEvent:run%(connection%).-\nend'))
    lu.assertNotNil(body:find('getUserByConnection', 1, true),
            'the farm id is still trusted as written by the client')
    lu.assertNil(body:find('onInputCall(self.vehicle, input, self.farmId', 1, true),
            'the wire value is still what gets used')
end

os.exit(lu.LuaUnit.run())
