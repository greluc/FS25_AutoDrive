--[[
Tests for the initial multiplayer sync, the settings/message events, the config writer and the
road network overlay on the in game map.

Covers findings:
  A5  AutoDriveSync.streamReadGraph consumed three of the four items the writer emits per map
      marker when the marker pointed at a missing waypoint, so a single dangling marker shifted
      the read cursor by one bit for the whole rest of the full graph sync
  A25 AutoDriveUpdateSettingsEvent:readStream skipped the entire vehicle settings block when the
      vehicle was unknown on this client, which misaligned everything behind it
  A26 AutoDrive.saveToXML never released the xml handle it created
  A51 AutoDriveMessageEvent:run called sendNotification without the vehicle argument
  A28 AutoDrive.drawNetworkOnMap projected the whole road network into screen space on every
      single frame the map was open

How the stream is tested: the real bug is about the NUMBER OF BITS each branch consumes, so a
type tagged queue would not reproduce it. The BitStream below is a genuine bit buffer - a writer
that emits one bit too many is only visible as a wrong value further down the stream, exactly as
it is on the wire - and every test asserts that the reader ends up on the same bit offset the
writer stopped at.

scripts/AutoDrive.lua cannot be required (its load time work needs the whole game), so
drawNetworkOnMap is lifted out of the file textually and run against fakes. That still exercises
the real code rather than a copy of it.
]]

lu = require('luaunit')
require('test-setup')

require('UtilFuncs')

------------------------------------------------------------------------------------------------------------------------
--- Engine plumbing the modules under test touch at load time
------------------------------------------------------------------------------------------------------------------------
--- Giants' Class/InitObjectClass/InitEventClass. Only the inheritance is modelled; the tests call
--- the static stream functions and the instance methods directly, never through the metatable.
if Class == nil then
    function Class(newClass, baseClass)
        if baseClass ~= nil then
            setmetatable(newClass, { __index = baseClass })
        end
        newClass.superClass = function() return baseClass end
        return { __index = newClass }
    end
end
if Object == nil then Object = {} end
if InitObjectClass == nil then function InitObjectClass() end end
if InitEventClass == nil then function InitEventClass() end end

------------------------------------------------------------------------------------------------------------------------
--- A bit accurate stream
------------------------------------------------------------------------------------------------------------------------
local BitStream = {}
BitStream.__index = BitStream

function BitStream.new()
    return setmetatable({ bits = {}, readPos = 1, overruns = 0 }, BitStream)
end

function BitStream:writeBits(value, numBits)
    value = math.floor(value)
    for _ = 1, numBits do
        self.bits[#self.bits + 1] = value % 2
        value = math.floor(value / 2)
    end
end

function BitStream:readBits(numBits)
    local value, mult = 0, 1
    for _ = 1, numBits do
        local bit = self.bits[self.readPos]
        if bit == nil then
            self.overruns = self.overruns + 1
            bit = 0
        end
        self.readPos = self.readPos + 1
        value = value + bit * mult
        mult = mult * 2
    end
    return value
end

function BitStream:writtenBits() return #self.bits end
function BitStream:readBitsCount() return self.readPos - 1 end

function streamWriteUIntN(streamId, value, numBits) streamId:writeBits(value, numBits) end
function streamReadUIntN(streamId, numBits) return streamId:readBits(numBits) end
function streamWriteBool(streamId, value) streamId:writeBits(value and 1 or 0, 1) end
function streamReadBool(streamId) return streamId:readBits(1) == 1 end
function streamWriteInt32(streamId, value) streamId:writeBits(value % 4294967296, 32) end
function streamReadInt32(streamId)
    local v = streamId:readBits(32)
    if v >= 2147483648 then v = v - 4294967296 end
    return v
end
function streamWriteUInt16(streamId, value) streamId:writeBits(value, 16) end
function streamReadUInt16(streamId) return streamId:readBits(16) end

function streamWriteString(streamId, value)
    value = tostring(value)
    streamId:writeBits(#value, 16)
    for i = 1, #value do streamId:writeBits(value:byte(i), 8) end
end
function streamReadString(streamId)
    local len = streamId:readBits(16)
    local chars = {}
    for i = 1, len do chars[i] = string.char(streamId:readBits(8) % 256) end
    return table.concat(chars)
end

function streamGetWriteOffset(streamId) return streamId:writtenBits() end
function streamGetReadOffset(streamId) return streamId:readBitsCount() end
function netGetTime() return 0 end

--- Fixed point with two decimals in 24 bits, which covers every map coordinate exactly for the
--- whole number positions the tests use.
local POS_BITS, POS_OFFSET, POS_SCALE = 24, 8388608, 100
NetworkUtil.writeCompressedWorldPosition = function(streamId, value, _)
    streamId:writeBits(math.floor(value * POS_SCALE + 0.5) + POS_OFFSET, POS_BITS)
end
NetworkUtil.readCompressedWorldPosition = function(streamId, _)
    return (streamId:readBits(POS_BITS) - POS_OFFSET) / POS_SCALE
end

--- test-setup does not know about the event classes; they live one directory deeper.
package.path = package.path .. ';../../scripts/Events/?.lua'

require('Sync')
require('XML')
require('UpdateSettingsEvent')
require('MessageEvent')

------------------------------------------------------------------------------------------------------------------------
--- A5 - the full graph sync on client join
------------------------------------------------------------------------------------------------------------------------
TestStreamGraph = {}

function TestStreamGraph:setUp()
    TestSetup.reset()
    g_currentMission.vehicleXZPosCompressionParams = { mock = true }
    g_currentMission.vehicleYPosCompressionParams = { mock = true }
end

--- Five waypoints means MWPC_SEND_NUM_BITS is 3, so marker ids 0 to 7 survive the wire. Id 7 has
--- no waypoint behind it and is therefore the dangling marker the server would send from a config
--- file whose marker section outlived the waypoint it pointed at.
local function graph(danglingId)
    local wayPoints = TestSetup.lineNetwork(5, 10)
    local mapMarkers = {
        { id = 2, markerIndex = 1, name = 'Silo', group = 'All', isADDebug = false },
        { id = danglingId or 4, markerIndex = 2, name = 'Kaputt', group = 'All', isADDebug = true },
        { id = 5, markerIndex = 3, name = 'Hof', group = 'Feld', isADDebug = false },
    }
    local groups = { All = 1, Feld = 2, Wald = 3 }
    return wayPoints, mapMarkers, groups
end

local function roundTrip(wayPoints, mapMarkers, groups)
    local stream = BitStream.new()
    AutoDriveSync.streamWriteGraph(stream, wayPoints, mapMarkers, groups)
    local written = stream:writtenBits()
    local readWayPoints, readMapMarkers, readGroups = AutoDriveSync.streamReadGraph(stream)
    return {
        wayPoints = readWayPoints,
        mapMarkers = readMapMarkers,
        groups = readGroups,
        written = written,
        read = stream:readBitsCount(),
        overruns = stream.overruns,
    }
end

function TestStreamGraph:testIntactGraphRoundTrips()
    local result = roundTrip(graph())
    lu.assertEquals(result.read, result.written, 'reader and writer must agree on the bit count')
    lu.assertEquals(result.overruns, 0)
    lu.assertEquals(#result.wayPoints, 5)
    lu.assertEquals(result.wayPoints[3].x, 30)
    lu.assertEquals(result.wayPoints[3].out, { 4 })
    lu.assertEquals(result.mapMarkers[2].name, 'Kaputt')
    lu.assertEquals(result.mapMarkers[3].name, 'Hof')
    lu.assertEquals(result.mapMarkers[3].group, 'Feld')
    lu.assertEquals(result.groups, { All = 1, Feld = 2, Wald = 3 })
end

--- The regression itself: the writer emits id, name, group and the isADDebug flag per marker, the
--- error branch used to consume only the first three. One bit of drift is enough to turn every
--- following marker into garbage and the 10 bit group count into an arbitrary value up to 1023.
function TestStreamGraph:testDanglingMarkerDoesNotShiftTheRestOfTheStream()
    local result = roundTrip(graph(7))

    lu.assertEquals(Logging.countMatching('Error receiving marker'), 1,
        'the dangling marker must still be reported')
    lu.assertEquals(result.read, result.written,
        'the error branch has to consume exactly as many bits as the writer emitted for a marker, '
        .. 'otherwise everything behind it decodes shifted by one bit')
    lu.assertEquals(result.overruns, 0, 'nothing may be read past the end of the stream')

    -- The marker written AFTER the dangling one is the one that proves the cursor is still aligned.
    -- Note the reader leaves a hole where the dangling marker was; compacting that index is a
    -- separate concern and deliberately not changed here.
    lu.assertEquals(result.mapMarkers[3].id, 5)
    lu.assertEquals(result.mapMarkers[3].name, 'Hof')
    lu.assertEquals(result.mapMarkers[3].group, 'Feld')
    lu.assertEquals(result.mapMarkers[3].isADDebug, false)

    -- ... and the whole group section behind the markers.
    lu.assertEquals(result.groups, { All = 1, Feld = 2, Wald = 3 })
end

--- Same graph, one marker valid in one run and dangling in the other: both runs have to leave the
--- reader on the very same offset, which is the property the fix is really about.
function TestStreamGraph:testBothMarkerBranchesConsumeTheSameNumberOfBits()
    local intact = roundTrip(graph(4))
    local dangling = roundTrip(graph(7))
    lu.assertEquals(dangling.read - dangling.written, 0)
    lu.assertEquals(intact.read - intact.written, 0)
    lu.assertEquals(dangling.written, intact.written,
        'both markers carry the same name and group, so they must occupy the same bits')
end

function TestStreamGraph:testEmptyGraphRoundTrips()
    local result = roundTrip({}, {}, {})
    lu.assertEquals(result.read, result.written)
    lu.assertEquals(result.overruns, 0)
    lu.assertEquals(#result.wayPoints, 0)
    lu.assertEquals(#result.mapMarkers, 0)
end

------------------------------------------------------------------------------------------------------------------------
--- A25 - the settings event when the vehicle is unknown on this client
------------------------------------------------------------------------------------------------------------------------
TestUpdateSettingsEvent = {}

local savedServer

local function setting(current, isVehicleSpecific, userDefault)
    return { current = current, new = current, isVehicleSpecific = isVehicleSpecific,
             isUserSpecific = false, userDefault = userDefault }
end

function TestUpdateSettingsEvent:setUp()
    TestSetup.reset()
    savedServer = g_server
    -- readStream re-broadcasts on the server; this test is the receiving client.
    g_server = nil
    AutoDrive.gui = { ADSettings = { forceLoadGUISettings = function() end } }

    AutoDrive.settings = {
        mapMarkerLimit = setting(11, false),
        lookAheadTurning = setting(22, false),
        followDistance = setting(33, true, 44),
        parkInField = setting(55, true, 66),
    }
    AutoDrive.testSettings = { followDistance = 77, parkInField = 88 }
    NetworkUtil.getObjectId = function() return 4711 end
    NetworkUtil.getObject = function() return nil end     -- the vehicle is unknown here
end

function TestUpdateSettingsEvent:tearDown()
    g_server = savedServer
    NetworkUtil.getObjectId = function() return 0 end
    NetworkUtil.getObject = function() return nil end
end

local function settingsRoundTrip(vehicle)
    local stream = BitStream.new()
    local writer = { vehicle = vehicle }
    AutoDriveUpdateSettingsEvent.writeStream(writer, stream, nil)
    local written = stream:writtenBits()

    -- The values have to be wrong before the read for a successful read to mean anything.
    for _, s in pairs(AutoDrive.settings) do
        s.current, s.new = 0, 0
        if s.userDefault ~= nil then s.userDefault = 0 end
    end

    AutoDriveUpdateSettingsEvent.readStream({}, stream, nil)
    return written, stream:readBitsCount(), stream.overruns
end

--- Without the fix the vehicle block is skipped whole, so the userDefault section behind it is
--- read out of the middle of the vehicle settings and every value in it is nonsense.
function TestUpdateSettingsEvent:testUnknownVehicleStillLeavesTheRestOfTheEventAligned()
    local written, read, overruns = settingsRoundTrip(TestSetup.vehicle())

    lu.assertEquals(read, written, 'the vehicle block has to be read and discarded, not skipped')
    lu.assertEquals(overruns, 0)
    lu.assertEquals(AutoDrive.settings.followDistance.userDefault, 44,
        'the userDefault block sits behind the vehicle block and must still decode')
    lu.assertEquals(AutoDrive.settings.parkInField.userDefault, 66)
    lu.assertEquals(AutoDrive.settings.mapMarkerLimit.current, 11)
    lu.assertEquals(AutoDrive.settings.lookAheadTurning.current, 22)
end

--- An event without a vehicle at all must be unaffected by the change.
function TestUpdateSettingsEvent:testEventWithoutVehicleRoundTrips()
    local written, read, overruns = settingsRoundTrip(nil)
    lu.assertEquals(read, written)
    lu.assertEquals(overruns, 0)
    lu.assertEquals(AutoDrive.settings.mapMarkerLimit.current, 11)
    lu.assertEquals(AutoDrive.settings.followDistance.userDefault, 44)
end

--- The vehicle IS known: its settings must actually be applied, so the discard path did not
--- swallow the normal case.
function TestUpdateSettingsEvent:testKnownVehicleGetsItsSettingsApplied()
    local vehicle = TestSetup.vehicle()
    vehicle.ad.settings = { followDistance = setting(0, true), parkInField = setting(0, true) }
    NetworkUtil.getObject = function() return vehicle end

    local written, read = settingsRoundTrip(vehicle)

    lu.assertEquals(read, written)
    lu.assertEquals(vehicle.ad.settings.followDistance.current, 77)
    lu.assertEquals(vehicle.ad.settings.followDistance.new, 77)
    lu.assertEquals(vehicle.ad.settings.parkInField.current, 88)
end

------------------------------------------------------------------------------------------------------------------------
--- A51 - the notification the server broadcasts on behalf of a client
------------------------------------------------------------------------------------------------------------------------
TestMessageEvent = {}

local savedSendNotification

function TestMessageEvent:setUp()
    TestSetup.reset()
    savedSendNotification = AutoDriveMessageEvent.sendNotification
end

function TestMessageEvent:tearDown()
    AutoDriveMessageEvent.sendNotification = savedSendNotification
end

function TestMessageEvent:testForwardedNotificationKeepsItsArgumentsInOrder()
    local seen
    AutoDriveMessageEvent.sendNotification = function(...) seen = { ... } end

    local vehicle = TestSetup.vehicle()
    local event = { vehicle = vehicle, isNotification = true, messageType = 3,
                    text = 'ad_driver_%s_at_%s', duration = 5000, args = { 'Fritz', 'Hof' } }
    AutoDriveMessageEvent.run(event, { getIsServer = function() return false end })

    lu.assertNotNil(seen, 'the server has to broadcast a notification coming from a client')
    lu.assertEquals(seen[1], vehicle,
        'sendNotification takes the vehicle first; leaving it out shifts messageType into it and '
        .. 'every following parameter with it')
    lu.assertEquals(seen[2], 3)
    lu.assertEquals(seen[3], 'ad_driver_%s_at_%s')
    lu.assertEquals(seen[4], 5000)
    lu.assertEquals(seen[5], 'Fritz')
    lu.assertEquals(seen[6], 'Hof')
end

function TestMessageEvent:testPlainMessageFromAClientIsNotBroadcast()
    local calls = 0
    AutoDriveMessageEvent.sendNotification = function() calls = calls + 1 end

    local event = { vehicle = TestSetup.vehicle(), isNotification = false, messageType = 1,
                    text = 'x', duration = 1, args = {} }
    AutoDriveMessageEvent.run(event, { getIsServer = function() return false end })

    lu.assertEquals(calls, 0)
end

------------------------------------------------------------------------------------------------------------------------
--- A26 - the xml handle of the config writer
------------------------------------------------------------------------------------------------------------------------
TestSaveToXML = {}

local xmlLog

function TestSaveToXML:setUp()
    TestSetup.reset()
    xmlLog = { created = {}, saved = {}, deleted = {} }

    createXMLFile = function(name)
        local handle = 'xml:' .. name .. ':' .. tostring(#xmlLog.created + 1)
        table.insert(xmlLog.created, handle)
        return handle
    end
    saveXMLFile = function(handle) table.insert(xmlLog.saved, handle) end
    delete = function(handle) table.insert(xmlLog.deleted, handle) end
    setXMLString = function() end
    setXMLInt = function() end
    setXMLFloat = function() end
    setXMLBool = function() end

    g_currentMission.missionInfo.savegameDirectory = './savegame1'
    AutoDrive.version = '3.0.0.0'
    AutoDrive.loadedMap = 'MockMap'
    AutoDrive.settings = { mapMarkerLimit = setting(1, false) }
    AutoDrive.experimentalFeatures = {}

    ADGraphManager = {
        wayPoints = TestSetup.lineNetwork(3, 10),
        mapMarkers = { { id = 2, name = 'Silo', group = 'All' } },
        groups = { All = 1 },
    }
    function ADGraphManager:getWayPoints() return self.wayPoints end
    function ADGraphManager:getWayPointsCount() return #self.wayPoints end
    function ADGraphManager:getMapMarkers() return self.mapMarkers end
    function ADGraphManager:getMapMarkerById(id) return self.mapMarkers[id] end
    function ADGraphManager:getGroups() return self.groups end
    function ADGraphManager:deleteColorSelectionWayPoints() end
end

function TestSaveToXML:tearDown()
    delete = function() end
end

function TestSaveToXML:testTheCreatedHandleIsReleasedAgain()
    AutoDrive.saveToXML()

    lu.assertEquals(#xmlLog.created, 1)
    lu.assertEquals(#xmlLog.saved, 1)
    lu.assertEquals(xmlLog.deleted, xmlLog.created,
        'the handle holds a full copy of the road network; one is leaked per savegame save '
        .. 'as long as it is not deleted')
end

function TestSaveToXML:testEveryCallReleasesItsOwnHandle()
    AutoDrive.saveToXML()
    AutoDrive.saveToXML()
    AutoDrive.saveToXML()
    lu.assertEquals(#xmlLog.created, 3)
    lu.assertEquals(#xmlLog.deleted, 3)
    lu.assertEquals(xmlLog.deleted, xmlLog.created)
end

--- A file that cannot be created must not be written to or deleted either.
function TestSaveToXML:testUncreatableFileIsReportedAndNothingIsSaved()
    createXMLFile = function() return nil end
    AutoDrive.saveToXML()
    lu.assertEquals(#xmlLog.saved, 0)
    lu.assertEquals(#xmlLog.deleted, 0)
    lu.assertEquals(Logging.countMatching('could not create'), 1)
end

------------------------------------------------------------------------------------------------------------------------
--- A28 - the road network overlay on the in game map
------------------------------------------------------------------------------------------------------------------------
--- scripts/AutoDrive.lua does load time work that needs the full game, so the function is lifted
--- out of the file by text and compiled on its own. The terminating "end" of a top level function
--- is the first line in the file that is exactly "end".
local function extractFunction(path, header)
    local file = io.open(path, 'r')
    if file == nil then
        error('test_sync_events: cannot read ' .. path .. ' - run tests from tools/tests')
    end
    local collected, capturing = {}, false
    for line in file:lines() do
        line = line:gsub('\r$', '')
        if not capturing and line:sub(1, #header) == header then capturing = true end
        if capturing then
            table.insert(collected, line)
            if line == 'end' then break end
        end
    end
    file:close()
    if #collected == 0 or collected[#collected] ~= 'end' then
        error('test_sync_events: could not extract "' .. header .. '" from ' .. path)
    end
    local chunk, err = load(table.concat(collected, '\n'), header)
    if chunk == nil then
        error('test_sync_events: ' .. tostring(err))
    end
    chunk()
end

extractFunction('../../scripts/AutoDrive.lua', 'function AutoDrive.drawNetworkOnMap()')

TestNetworkOnMap = {}

local view, projections, renders

--- Four connections at distinct scaled positions. The projection is a plain affine mapping, which
--- is what the map does as well: pan moves the offset, zoom changes the scale.
local function networkCache(n)
    local cache = {}
    for i = 1, n do
        table.insert(cache, {
            startX = 0.3 + i * 0.01, startY = 0.3, endX = 0.3 + i * 0.01, endY = 0.4,
            rotation = 0.5, r = 0, g = 1, b = 0, a = 1,
        })
    end
    return cache
end

function TestNetworkOnMap:setUp()
    TestSetup.reset()
    view = { offsetX = 0, offsetY = 0, scale = 1, hidden = {} }
    projections, renders = 0, 0

    g_screenHeight = 1080
    g_screenAspectRatio = 16 / 9

    AutoDrive.courseOverlayId = 'overlay'
    AutoDrive.aiNetworkOnMapCache = networkCache(4)
    AutoDrive.aiNetworkOnMapScreenCache = nil
    AutoDrive.aiNetworkOnMapScreenFrame = 0

    AutoDrive.isEditorModeEnabled = function() return true end
    AutoDrive.createNetworkOnMapCache = function() end
    AutoDrive.getScreenPosFromScaledWorldPos = function(x, y)
        projections = projections + 1
        local visible = not view.hidden[string.format('%.4f/%.4f', x, y)]
        return view.offsetX + x * view.scale, view.offsetY + y * view.scale, visible
    end

    setOverlayColor = function() end
    setOverlayRotation = function() end
    renderOverlay = function() renders = renders + 1 end
end

--- Two of the projections per frame are the probe points that detect a moved map view.
local PROBES = 2

function TestNetworkOnMap:testFirstFrameProjectsTheWholeNetwork()
    AutoDrive.drawNetworkOnMap()
    lu.assertEquals(projections, PROBES + 2 * 4)
    lu.assertEquals(renders, 4)
end

function TestNetworkOnMap:testUnchangedViewDoesNotProjectAgain()
    lu.assertTrue(AutoDrive.PERF_FRAMES_HIGH > 2,
        'this test needs at least one frame between the periodic refreshes')
    AutoDrive.drawNetworkOnMap()

    projections, renders = 0, 0
    AutoDrive.drawNetworkOnMap()

    lu.assertEquals(projections, PROBES,
        'with the map standing still the screen positions cannot have changed, so only the two '
        .. 'probe points have to be looked up')
    lu.assertEquals(renders, 4, 'every line must still be drawn')
end

function TestNetworkOnMap:testPanningTheMapReprojects()
    AutoDrive.drawNetworkOnMap()

    projections, renders = 0, 0
    view.offsetX = 0.05
    AutoDrive.drawNetworkOnMap()

    lu.assertEquals(projections, PROBES + 2 * 4, 'a moved view has to be picked up immediately')
    lu.assertEquals(renders, 4)
end

function TestNetworkOnMap:testZoomingTheMapReprojects()
    AutoDrive.drawNetworkOnMap()

    projections, renders = 0, 0
    view.scale = 2
    AutoDrive.drawNetworkOnMap()

    lu.assertEquals(projections, PROBES + 2 * 4)
    lu.assertEquals(renders, 4)
end

--- The safety net: even if the map were to hold the probe points in place, the projection is
--- refreshed again after at most PERF_FRAMES_HIGH frames.
function TestNetworkOnMap:testProjectionIsRefreshedPeriodically()
    AutoDrive.drawNetworkOnMap()

    AutoDrive.aiNetworkOnMapScreenFrame = AutoDrive.PERF_FRAMES_HIGH - 1
    projections, renders = 0, 0
    AutoDrive.drawNetworkOnMap()

    lu.assertEquals(projections, PROBES + 2 * 4)
    lu.assertEquals(renders, 4)
end

--- A rebuilt world cache must never be drawn from the old screen positions.
function TestNetworkOnMap:testRebuiltNetworkCacheInvalidatesTheScreenPositions()
    AutoDrive.drawNetworkOnMap()

    projections, renders = 0, 0
    AutoDrive.aiNetworkOnMapCache = networkCache(2)
    AutoDrive.drawNetworkOnMap()

    lu.assertEquals(projections, PROBES + 2 * 2)
    lu.assertEquals(renders, 2)
end

--- Lines outside the visible part of the map are dropped, as before.
function TestNetworkOnMap:testInvisibleConnectionsAreNotDrawn()
    view.hidden[string.format('%.4f/%.4f', 0.31, 0.3)] = true
    AutoDrive.drawNetworkOnMap()
    lu.assertEquals(renders, 3)
end

function TestNetworkOnMap:testNothingIsDrawnOutsideTheEditorModes()
    AutoDrive.isEditorModeEnabled = function() return false end
    AutoDrive.drawNetworkOnMap()
    lu.assertEquals(projections, 0)
    lu.assertEquals(renders, 0)
end

os.exit(lu.LuaUnit.run())
