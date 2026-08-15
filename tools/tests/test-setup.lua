--[[
Shared bootstrap for the AutoDrive unit tests.

Loads the engine mock and then the AutoDrive modules under test in dependency order. A new test
file is three lines of boilerplate:

    lu = require('luaunit')
    require('test-setup')
    ...
    os.exit(lu.LuaUnit.run())

Run everything with:  python tools/check.py
Run one file with:    lua tools/tests/test_<name>.lua      (from inside tools/tests)

Why so little is loaded here: AutoDrive's modules are coupled to the vehicle object and to each
other through globals, so loading all of them would mean mocking most of the game. Each test file
therefore requires only what it needs, and this file provides the parts every one of them uses.
]]

package.path = package.path .. ';./?.lua;../../scripts/?.lua;../../scripts/Utils/?.lua;'
    .. '../../scripts/Manager/?.lua;../../scripts/Modules/?.lua;../../scripts/Modes/?.lua;'
    .. '../../scripts/Tasks/?.lua;../../scripts/Sensors/?.lua'

require('mock-engine')

------------------------------------------------------------------------------------------------------------------------
--- AutoDrive namespace
------------------------------------------------------------------------------------------------------------------------
--- scripts/AutoDrive.lua does a great deal of load-time work that needs the full game. The tests
--- want the namespace and the constants, not that bootstrap, so the table is created here and the
--- pieces under test are pulled in individually by each test file.
AutoDrive = AutoDrive or {}
AutoDrive.directory = '../../'
AutoDrive.debug = {}

--- Constants are READ FROM THE MOD, not copied here.
---
--- scripts/AutoDrive.lua cannot simply be required: it does a lot of load-time work that needs the
--- full game. But its constant block is plain `AutoDrive.NAME = <number>` lines, so they are
--- extracted textually. That way a test can never assert against a stale copy of a value the mod
--- has since changed - which is exactly how a green test suite starts lying.
local function loadConstantsFromMod()
    local path = '../../scripts/AutoDrive.lua'
    local f = io.open(path, 'r')
    if not f then
        error('test-setup: cannot read ' .. path .. ' - run tests from tools/tests')
    end
    local src = f:read('*a')
    f:close()

    --- Line by line, not one gmatch over the whole file: a pattern that consumes the trailing
    --- newline cannot match the next line, because gmatch does not produce overlapping matches.
    --- That silently halved the extracted set the first time round.
    local count = 0
    for line in src:gmatch('[^\r\n]+') do
        local name, value = line:match('^%s*AutoDrive%.([%u][%u%d_]*)%s*=%s*(%-?[%d%.]+)%s*;?%s*$')
        if name ~= nil then
            local n = tonumber(value)
            if n ~= nil then
                AutoDrive[name] = n
                count = count + 1
            end
        end
    end
    if count < 20 then
        error(string.format('test-setup: only %d constants extracted from AutoDrive.lua - '
            .. 'the constant block moved or changed shape, fix loadConstantsFromMod()', count))
    end
    return count
end

AutoDrive.constantsLoaded = loadConstantsFromMod()

--- Runtime state that is not a constant.
AutoDrive.drawDistance = 50
AutoDrive.drawHeight = 0
AutoDrive.currentDebugChannelMask = 0

--- The debug layer, for suites that do not load UtilFuncs.lua.
---
--- Since the P3 work every debug call with computed arguments sits behind an explicit
--- `if AutoDrive.getDebugChannelIsSet(CHANNEL) then` at the call site, so any module can reach for
--- these two even when the suite under test never required UtilFuncs. Loading UtilFuncs replaces
--- both with the real implementations; these only fill the gap.
if AutoDrive.getDebugChannelIsSet == nil then
    function AutoDrive.getDebugChannelIsSet(channel)
        if channel == nil or AutoDrive.currentDebugChannelMask == nil then
            return false
        end
        return bit32.band(AutoDrive.currentDebugChannelMask, channel) > 0
    end
end
if AutoDrive.debugPrint == nil then
    function AutoDrive.debugPrint() end
end
if AutoDrive.debugMsg == nil then
    function AutoDrive.debugMsg() end
end

--- Courseplay interface calls, for suites that do not load ExternalInterface.lua.
--- Both are no-ops here; the real behaviour is covered by test_cp_interface.lua.
if AutoDrive.requestCourseplayProximity == nil then
    function AutoDrive:requestCourseplayProximity() return false end
end
if AutoDrive.requestBackupForReversingCombine == nil then
    function AutoDrive:requestBackupForReversingCombine() return false end
end
if AutoDrive.getIsCPTurning == nil then
    function AutoDrive:getIsCPTurning() return false end
end

--- Editor mode gates the pathfinder's debug drawing; off by default in tests.
if AutoDrive.isEditorModeEnabled == nil then
    function AutoDrive.isEditorModeEnabled() return false end
end

--- Settings are a whole subsystem; tests that need one install a stub value here.
AutoDrive.settings = AutoDrive.settings or {}
AutoDrive.testSettings = {}
function AutoDrive.getSetting(name)
    return AutoDrive.testSettings[name]
end
function AutoDrive.getSettingState(name)
    return AutoDrive.testSettings[name]
end

--- The class helper from register.lua, used by every AutoDrive class.
function ADInheritsFrom(baseClass)
    local new_class = {}
    local class_mt = { __index = new_class }
    function new_class:create()
        local newinst = {}
        setmetatable(newinst, class_mt)
        return newinst
    end
    if baseClass ~= nil then
        setmetatable(new_class, { __index = baseClass })
    end
    return new_class
end

------------------------------------------------------------------------------------------------------------------------
--- Shared helpers
------------------------------------------------------------------------------------------------------------------------
TestSetup = {}

--- A vehicle stub with the fields the modules under test read. Extra fields can be merged in.
function TestSetup.vehicle(overrides)
    local v = {
        id = 1,
        isServer = true,
        isClient = true,
        size = { width = 3, length = 6, lengthOffset = 0 },
        components = { { node = 'root' } },
        lastSpeedReal = 0,
        ad = {
            distances = { wayPoints = nil, closest = { wayPoint = -1, distance = 0 },
                          closestNotReverse = { wayPoint = -1, distance = 0 } },
            groups = {},
            settings = {},
        },
        getName = function() return 'TestVehicle' end,
        getRootVehicle = function(self) return self end,
        getAttachedImplements = function() return {} end,
    }
    if overrides then
        for k, val in pairs(overrides) do v[k] = val end
    end
    return v
end

--- Builds a waypoint the way ADGraphManager:createNode does.
function TestSetup.waypoint(id, x, z, out, incoming)
    return { id = id, x = x, y = 0, z = z, out = out or {}, incoming = incoming or {}, flags = 0 }
end

--- A straight line of n waypoints one metre apart, each connected to the next.
function TestSetup.lineNetwork(n, spacing)
    spacing = spacing or 1
    local wps = {}
    for i = 1, n do
        wps[i] = TestSetup.waypoint(i, i * spacing, 0,
            i < n and { i + 1 } or {},
            i > 1 and { i - 1 } or {})
    end
    return wps
end

function TestSetup.reset()
    MockEngine.reset()
    AutoDrive.testSettings = {}
    AutoDrive.currentDebugChannelMask = 0
end

return TestSetup
