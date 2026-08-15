--[[
Tests for the pathfinder fallback display.

The bug: the HUD read vehicle.ad.pathFinderModule.fallBackMode, a field that is assigned nowhere
in the mod, so the overlay always showed "Fallback: nil". The real state is three separate flags,
and they accumulate as the search relaxes one restriction after another.

Found in the game, not by the audit - HudIcon.lua was in no agent's scope.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('SortedQueue')
require('Dubins')
require('PathFinderUtils')
require('PathFinderModule')

local function pfm()
    local o = setmetatable({}, { __index = PathFinderModule })
    o.fallBackMode1 = false
    o.fallBackMode2 = false
    o.fallBackMode3 = false
    return o
end

TestFallbackDisplay = {}

function TestFallbackDisplay:setUp() TestSetup.reset() end

function TestFallbackDisplay:testTheDeadFieldIsGone()
    local src = io.open('../../scripts/Hud/HudIcon.lua', 'r'):read('*a')
    lu.assertNil(src:find('pathFinderModule.fallBackMode', 1, true),
        'the HUD must not read the non-existent fallBackMode field again')
end

function TestFallbackDisplay:testNothingActive()
    lu.assertEquals(pfm():getFallBackModeText(), 'none')
end

function TestFallbackDisplay:testNeverReturnsNil()
    -- the whole point: the display must always get a string
    local p = pfm()
    lu.assertIsString(p:getFallBackModeText())
    p.fallBackMode2 = true
    lu.assertIsString(p:getFallBackModeText())
end

function TestFallbackDisplay:testEachFlagIsNamed()
    local p = pfm()
    p.fallBackMode1 = true
    lu.assertStrContains(p:getFallBackModeText(), 'no field')

    p = pfm(); p.fallBackMode2 = true
    lu.assertStrContains(p:getFallBackModeText(), 'no border')

    p = pfm(); p.fallBackMode3 = true
    lu.assertStrContains(p:getFallBackModeText(), 'no fruit')
end

--- The flags accumulate - the pathfinder drops one restriction after another - so the display has
--- to show all of them, not just the newest.
function TestFallbackDisplay:testAccumulatedFallbacksAreAllShown()
    local p = pfm()
    p.fallBackMode1 = true
    p.fallBackMode2 = true
    p.fallBackMode3 = true
    local text = p:getFallBackModeText()
    lu.assertStrContains(text, 'no field')
    lu.assertStrContains(text, 'no border')
    lu.assertStrContains(text, 'no fruit')
end

--- A fresh module must report "none", so a stale reading cannot look like an active fallback.
function TestFallbackDisplay:testResetClearsTheDisplay()
    local p = pfm()
    p.fallBackMode1, p.fallBackMode3 = true, true
    lu.assertNotEquals(p:getFallBackModeText(), 'none')
    p.fallBackMode1, p.fallBackMode3 = false, false
    lu.assertEquals(p:getFallBackModeText(), 'none')
end

os.exit(lu.LuaUnit.run())
