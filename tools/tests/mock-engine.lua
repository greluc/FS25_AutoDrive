--[[
Mocks for the Giants engine globals that AutoDrive touches.

Farming Simulator 25 runs Luau and provides several hundred engine functions as globals. The
modules under test reach for a small, well-known subset of them, and only a handful of those
actually influence the behaviour we assert on. Everything else only has to exist and return
something harmless.

Two rules keep this file honest:

  1. Anything whose *return value* a test depends on is defined explicitly below, so the
     expectation is visible in the source rather than buried in a permissive fallback.
  2. Everything else is caught by MockEngine.autostub at the bottom, which hands out a
     no-op function on first access and records the name. A test can read
     MockEngine.touched to see which engine calls a code path made - useful when a fix is
     supposed to *stop* calling something.

Deliberately NOT permissive about tables: reading an undefined global table would silently
produce nil rather than an error, and that is exactly the class of bug we are fixing.
]]

MockEngine = {
    touched = {},
    -- Scene state the geometry helpers read. Tests overwrite these.
    terrainHeight = 0,
    overlapBoxHits = {},
    raycastHits = {},
}

------------------------------------------------------------------------------------------------------------------------
--- Runtime compatibility
------------------------------------------------------------------------------------------------------------------------
--- Luau's standard library follows Lua 5.1: math.atan2 and math.pow exist there and AutoDrive uses
--- both. Standard Lua dropped them in 5.3/5.4. Define them only when missing, so the same test
--- files run unchanged on 5.1 through 5.4 and under Luau. Do NOT mirror this into the mod.
if math.atan2 == nil then
    math.atan2 = function(y, x) return math.atan(y, x) end
end
if math.pow == nil then
    math.pow = function(base, exp) return base ^ exp end
end

--- bit32 was removed in Lua 5.3. The engine provides it, AutoDrive uses it for the debug channel
--- mask. Only the operations AutoDrive actually calls are implemented.
if bit32 == nil then
    bit32 = {}
    local function norm(x) return x % 0x100000000 end
    function bit32.band(a, b)
        a, b = norm(a), norm(b)
        local res, bitval = 0, 1
        for _ = 1, 32 do
            if (a % 2 == 1) and (b % 2 == 1) then res = res + bitval end
            a, b, bitval = math.floor(a / 2), math.floor(b / 2), bitval * 2
        end
        return res
    end
    function bit32.bor(a, b)
        a, b = norm(a), norm(b)
        local res, bitval = 0, 1
        for _ = 1, 32 do
            if (a % 2 == 1) or (b % 2 == 1) then res = res + bitval end
            a, b, bitval = math.floor(a / 2), math.floor(b / 2), bitval * 2
        end
        return res
    end
    function bit32.bxor(a, b)
        a, b = norm(a), norm(b)
        local res, bitval = 0, 1
        for _ = 1, 32 do
            if (a % 2) ~= (b % 2) then res = res + bitval end
            a, b, bitval = math.floor(a / 2), math.floor(b / 2), bitval * 2
        end
        return res
    end
    function bit32.lshift(a, n) return norm(norm(a) * (2 ^ n)) end
    function bit32.rshift(a, n) return math.floor(norm(a) / (2 ^ n)) end
end

--- FS25 exposes table.unpack as the global unpack as well.
if unpack == nil then unpack = table.unpack end

------------------------------------------------------------------------------------------------------------------------
--- Mission and mod context
------------------------------------------------------------------------------------------------------------------------
g_updateLoopIndex = 0
g_time = 0
g_currentModName = 'FS25_AutoDrive'
g_currentModDirectory = './'
g_server = {}
g_client = {}
g_dedicatedServer = nil

g_currentMission = {
    mock = true,
    terrainRootNode = 1,
    mapWidth = 2048,
    mapHeight = 2048,
    missionInfo = { mapId = 'MockMap', map = { title = 'MockMap' } },
    missionDynamicInfo = { isMultiplayer = false },
    nodeToObject = {},
    vehicleSystem = { vehicles = {} },
    getNodeObject = function(_, id) return g_currentMission.nodeToObject[id] end,
}

function getUserProfileAppPath() return './' end
function fileExists() return false end
function createFolder() end
function getfenv() return _G end

------------------------------------------------------------------------------------------------------------------------
--- Logging
------------------------------------------------------------------------------------------------------------------------
--- Captured rather than printed: several fixes are specifically about a message that should no
--- longer be emitted, and a test needs to assert on that.
Logging = {
    lines = {},
    _record = function(level, fmt, ...)
        local ok, msg = pcall(string.format, fmt, ...)
        table.insert(Logging.lines, { level = level, text = ok and msg or tostring(fmt) })
    end,
}
function Logging.info(fmt, ...) Logging._record('info', fmt, ...) end
function Logging.warning(fmt, ...) Logging._record('warning', fmt, ...) end
function Logging.error(fmt, ...) Logging._record('error', fmt, ...) end
function Logging.devError(fmt, ...) Logging._record('devError', fmt, ...) end
function Logging.devWarning(fmt, ...) Logging._record('devWarning', fmt, ...) end
function Logging.reset() Logging.lines = {} end
function Logging.countMatching(pattern)
    local n = 0
    for _, l in ipairs(Logging.lines) do
        if l.text:find(pattern, 1, true) then n = n + 1 end
    end
    return n
end

------------------------------------------------------------------------------------------------------------------------
--- Math helpers the engine provides
------------------------------------------------------------------------------------------------------------------------
MathUtil = {}
function MathUtil.vector2Length(x, z) return math.sqrt(x * x + z * z) end
function MathUtil.vector3Length(x, y, z) return math.sqrt(x * x + y * y + z * z) end
function MathUtil.vector2Normalize(x, z)
    local l = math.sqrt(x * x + z * z)
    if l < 1e-9 then return 0, 0 end
    return x / l, z / l
end
function MathUtil.getPointPointDistance(x1, z1, x2, z2)
    return math.sqrt((x2 - x1) ^ 2 + (z2 - z1) ^ 2)
end
function MathUtil.clamp(v, lo, hi) return math.min(math.max(v, lo), hi) end
function MathUtil.round(v) return math.floor(v + 0.5) end
function MathUtil.getYRotationFromDirection(x, z) return math.atan2(x, z) end
function MathUtil.getDirectionFromYRotation(y) return math.sin(y), math.cos(y) end
function MathUtil.numberToSetBitsStr(n) return tostring(n) end
function MathUtil.hasRectangleLineIntersection2D() return false end

Utils = {}
function Utils.getNoNil(v, default) if v == nil then return default end return v end
function Utils.renderTextAtWorldPosition() end
function Utils.splitString(sep, s)
    local out = {}
    for token in string.gmatch(s or '', '([^' .. sep .. ']+)') do table.insert(out, token) end
    return out
end

------------------------------------------------------------------------------------------------------------------------
--- Scene / transform functions
------------------------------------------------------------------------------------------------------------------------
--- The scene is a flat plane at MockEngine.terrainHeight with no objects, unless a test says
--- otherwise. Node positions live in MockEngine.nodePositions keyed by node id.
MockEngine.nodePositions = {}

function getWorldTranslation(node)
    local p = MockEngine.nodePositions[node]
    if p then return p.x, p.y, p.z end
    return 0, 0, 0
end

function setTranslation(node, x, y, z)
    MockEngine.nodePositions[node] = { x = x, y = y, z = z }
end

--- Every node is axis aligned and faces +Z, so a local coordinate is the world one minus the node
--- origin. Rotation is deliberately not modelled: a test that needs a turned vehicle would be
--- asserting against this mock's trigonometry rather than against AutoDrive, and the geometry that
--- matters here - in front of / behind / how far - survives the simplification intact.
function localToWorld(node, x, y, z)
    local p = MockEngine.nodePositions[node] or { x = 0, y = 0, z = 0 }
    return p.x + x, p.y + y, p.z + z
end

function worldToLocal(node, x, y, z)
    local p = MockEngine.nodePositions[node] or { x = 0, y = 0, z = 0 }
    return x - p.x, y - p.y, z - p.z
end

function localToLocal(fromNode, toNode, x, y, z)
    return worldToLocal(toNode, localToWorld(fromNode, x, y, z))
end

function localDirectionToWorld(_, x, y, z) return x, y, z end
function worldDirectionToLocal(_, x, y, z) return x, y, z end
function getTerrainNormalAtWorldPos() return 0, 1, 0 end

function getTerrainHeightAtWorldPos() return MockEngine.terrainHeight end
function createTransformGroup(name) return 'node:' .. tostring(name) end
function link() end
function delete() end
function getParent() return nil end
function getName(id) return tostring(id) end
function getChildAt() return nil end
function getNumOfChildren() return 0 end
function setVisibility() end
function getVisibility() return false end
function getCollisionFilterGroup() return 0 end
function getRigidBodyType() return 'NONE' end

--- overlapBox invokes the callback once per entry in MockEngine.overlapBoxHits and returns the
--- number of hits, which is how the engine behaves.
function overlapBox(_, _, _, _, _, _, _, _, _, callbackName, target, ...)
    local n = 0
    for _, id in ipairs(MockEngine.overlapBoxHits) do
        n = n + 1
        if target ~= nil and type(target) == 'table' and type(target[callbackName]) == 'function' then
            target[callbackName](target, id)
        elseif type(_G[callbackName]) == 'function' then
            _G[callbackName](id)
        end
    end
    return n
end

function raycastClosest() return nil end
function raycastAll() return nil end

------------------------------------------------------------------------------------------------------------------------
--- Collision flags, as documented in shared/collisionMaskFlags.xml of the game
------------------------------------------------------------------------------------------------------------------------
CollisionFlag = {
    DEFAULT = 2 ^ 0,
    STATIC_OBJECT = 2 ^ 1,
    AI_BLOCKING = 2 ^ 5,
    TERRAIN = 2 ^ 8,
    TERRAIN_DELTA = 2 ^ 9,
    TREE = 2 ^ 11,
    BUILDING = 2 ^ 12,
    ROAD = 2 ^ 13,
    VEHICLE = 2 ^ 16,
    DYNAMIC_OBJECT = 2 ^ 18,
    TRAFFIC_VEHICLE = 2 ^ 19,
    TRAFFIC_VEHICLE_BLOCKING = 2 ^ 24,
    TRIGGER = 2 ^ 29,
    FILLABLE = 2 ^ 30,
    WATER = 2 ^ 31,
}

------------------------------------------------------------------------------------------------------------------------
--- Specialization / vehicle plumbing
------------------------------------------------------------------------------------------------------------------------
SpecializationUtil = {
    hasSpecialization = function() return false end,
    registerFunction = function() end,
    registerEventListener = function() end,
    registerOverwrittenFunction = function() end,
    registerEvent = function() end,
    raiseEvent = function() end,
}

XMLValueType = setmetatable({}, { __index = function(t, k) rawset(t, k, k) return k end })
Drivable = { CRUISECONTROL_STATE_OFF = 0, CRUISECONTROL_STATE_ACTIVE = 1 }
AIJobFieldWork = { getPricePerMs = function() return 0.0005 end }
FillType = { UNKNOWN = 0, SEEDS = 1, FERTILIZER = 2, LIQUIDFERTILIZER = 3, WHEAT = 4 }

g_fillTypeManager = {
    getFillTypes = function() return {} end,
    getFillTypeByIndex = function(_, i) return { title = 'FillType' .. tostring(i), name = 'FT' .. tostring(i) } end,
    getFillTypeIndexByName = function() return 1 end,
}
g_fruitTypeManager = {
    getFruitTypeIndexByFillTypeIndex = function() return nil end,
    getFruitTypeByIndex = function(_, i) return { fillType = { name = 'f', title = 'F' } } end,
}
g_helperManager = { getRandomHelper = function() return { index = 1, name = 'Helper' } end, numHelpers = 4 }
g_i18n = { getText = function(_, k) return k end, hasText = function() return false end }
g_gui = { loadProfiles = function() end, loadGui = function() return {} end }
g_overlayManager = { addTextureConfigFile = function() end }
g_inputBinding = {}
g_consoleCommands = { registerConsoleCommand = function() end }
function addConsoleCommand() end

DebugUtil = { drawOverlapBox = function() end, drawDebugLine = function() end }

--- UtilFuncs.lua defines methods on NetworkNode at load time (the network traffic debug hooks).
NetworkNode = { PACKET_EVENT = 1 }

--- Event plumbing touched at load time by the Events/ classes.
Event = { new = function() return {} end }
function streamWriteUIntN() end
function streamReadUIntN() return 0 end
function streamWriteBool() end
function streamReadBool() return false end
function streamWriteString() end
function streamReadString() return '' end
function streamWriteInt32() end
function streamReadInt32() return 0 end
function streamWriteUInt8() end
function streamReadUInt8() return 0 end
function streamWriteUInt16() end
function streamReadUInt16() return 0 end
function streamWriteFloat32() end
function streamReadFloat32() return 0 end
NetworkUtil = { getObject = function() return nil end, getObjectId = function() return 0 end }
AIVehicleUtil = {
    getAttachedImplementsMaxTurnRadius = function() return -1 end,
    getMaxToolRadius = function() return 0 end,
    driveInDirection = function() end,
}

------------------------------------------------------------------------------------------------------------------------
--- Autostub: everything not named above
------------------------------------------------------------------------------------------------------------------------
--- Records the name and returns a no-op function. Only functions are auto-created; an undefined
--- global read in a value position still yields nil, which is what we want to catch.
function MockEngine.autostub()
    setmetatable(_G, {
        __index = function(_, key)
            if type(key) ~= 'string' then return nil end
            MockEngine.touched[key] = (MockEngine.touched[key] or 0) + 1
            return nil
        end,
    })
end

function MockEngine.reset()
    MockEngine.touched = {}
    MockEngine.overlapBoxHits = {}
    MockEngine.nodePositions = {}
    MockEngine.terrainHeight = 0
    g_currentMission.nodeToObject = {}
    g_currentMission.vehicleSystem.vehicles = {}
    g_updateLoopIndex = 0
    Logging.reset()
end

return MockEngine
