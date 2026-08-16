--[[
Joining bunker silo segments into one silo.

addBunkerSilo walked the known silos looking for a neighbour, merged with the first match and broke.
A segment that touches TWO existing entries - the middle one of three, placed last - therefore joined
only one of them, and the other stayed a separate silo forever: loadAllTriggers re-walks the
placeables in the same placement order on every rebuild, so it never healed.

getMaxBunkerSiloLength then reports the longest fragment rather than the silo, and that number
decides when a driver starts looking for the bunker tip point. A sixty metre silo reported as forty
has the driver begin its silo behaviour twenty metres late.
]]

lu = require('luaunit')
require('test-setup')
require('UtilFuncs')
require('TriggerManager')

--- One segment, laid along +Z from z0 to z1 and eight metres wide. v is filled in by
--- addBunkerSiloAreaV, exactly as it is for a real placeable.
local function segment(z0, z1)
    return {
        bunkerSiloArea = {
            sx = 0, sy = 0, sz = z0,
            wx = 8, wy = 0, wz = z0,
            hx = 0, hy = 0, hz = z1,
        },
    }
end

local function siloLengths()
    local out = {}
    for _, silo in ipairs(ADTriggerManager.bunkerSilosResult) do
        local area = silo.bunkerSiloArea
        out[#out + 1] = MathUtil.vector2Length(area.hx - area.sx, area.hz - area.sz)
    end
    table.sort(out)
    return out
end

TestBunkerSiloJoin = {}

function TestBunkerSiloJoin:setUp()
    TestSetup.reset()
    ADTriggerManager.bunkerSilos = {}
    ADTriggerManager.bunkerSilosResult = {}
    ADTriggerManager.maxBunkerSiloLength = 0
end

local function register(order)
    for _, seg in ipairs(order) do
        ADTriggerManager.addBunkerSilo(seg)
    end
end

--- The control: laid down in order, front to back, the three segments have always become one silo.
function TestBunkerSiloJoin:testSegmentsInOrderBecomeOneSilo()
    local a, b, c = segment(0, 20), segment(20, 40), segment(40, 60)
    register({ a, b, c })

    lu.assertEquals(siloLengths(), { 60 })
    lu.assertEquals(ADTriggerManager.getMaxBunkerSiloLength(), 60)
end

function TestBunkerSiloJoin:testSegmentsInReverseOrderBecomeOneSilo()
    local a, b, c = segment(0, 20), segment(20, 40), segment(40, 60)
    register({ c, b, a })

    lu.assertEquals(siloLengths(), { 60 })
end

--- And the case that fell apart: the middle segment placed last, which is what happens when a player
--- puts down both ends first and fills the gap, or extends a silo on both sides.
function TestBunkerSiloJoin:testTheMiddleSegmentPlacedLastJoinsBothSides()
    local a, b, c = segment(0, 20), segment(20, 40), segment(40, 60)
    register({ a, c, b })

    lu.assertEquals(siloLengths(), { 60 },
        'the middle segment touches both ends, so it has to join both - joining one leaves the '
        .. 'other as a silo of its own for the rest of the save')
    lu.assertEquals(ADTriggerManager.getMaxBunkerSiloLength(), 60,
        'and the reported length decides when a driver starts looking for the tip point')
end

function TestBunkerSiloJoin:testTheOtherMiddleLastOrderToo()
    local a, b, c = segment(0, 20), segment(20, 40), segment(40, 60)
    register({ c, a, b })

    lu.assertEquals(siloLengths(), { 60 })
end

--- Two silos that genuinely do not touch stay two.
function TestBunkerSiloJoin:testSegmentsThatDoNotTouchStaySeparate()
    register({ segment(0, 20), segment(100, 120) })

    lu.assertEquals(siloLengths(), { 20, 20 })
end

os.exit(lu.LuaUnit.run())
