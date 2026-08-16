--[[
Tests for the HUD/GUI layer - scripts/Gui.lua, scripts/Hud.lua, scripts/Hud/PullDownList.lua and
scripts/Hud/HudSettingsButton.lua.

Covers findings:
  B6  AutoDrive:loadGUI reported all 15 loads as failed on every game start
  A23 ADHudSettingsButton:act never told the caller it consumed the click
  A24 dragging the HUD re-derived the pull-down list contents on every mouse-move event
  A47 getListElementByIndex looked a group name up by scanning all groups, per group visited
  A48 assigning to the control variable of a numeric for loop dropped the first list row

These four modules only define functions at load time, so unlike most of AutoDrive they can be
required and called directly. What they call into (the graph manager, the settings, the text
renderer) is stubbed here rather than in mock-engine.lua, because the stubs are specific to what
these tests assert on.
]]

lu = require('luaunit')
require('test-setup')

--- scripts/Hud is not in the search path test-setup sets up.
package.path = package.path .. ';../../scripts/Hud/?.lua'

require('UtilFuncs')            -- ADTable, the debug channel layer
require('GenericHudElement')    -- ADGenericHudElement, the real ADInheritsFrom
require('PullDownList')
require('HudSettingsButton')
require('Hud')
require('Gui')

------------------------------------------------------------------------------------------------------------------------
--- Shared stubs
------------------------------------------------------------------------------------------------------------------------
--- The text renderer. Everything drawn is collected in UiTest.rendered so a test can assert on
--- which rows of a list actually reached the screen.
UiTest = { rendered = {} }

RenderText = { ALIGN_LEFT = 0, ALIGN_CENTER = 1, ALIGN_RIGHT = 2 }

function setTextAlignment() end
function setTextBold() end
function setTextColor() end
function drawFilledRect() end
function renderOverlay() end
function getTextHeight() return 0.02 end
function getTextLineLength(_, text) return #text end
function utf8Substr(text, from, length) return string.sub(text, from + 1, from + length) end
function renderText(_, _, _, text) table.insert(UiTest.rendered, text) end

g_gameSettings = { getValue = function() return 1 end }

--- The graph manager, reduced to the group/marker API the pull-down lists use. getGroupsCalls is
--- what proves A47: the reverse lookup must be built once, not once per group visited.
ADGraphManager = {
    groups = {},
    mapMarkers = {},
    getGroupsCalls = 0,
    markerScans = 0,
}
function ADGraphManager:getGroups()
    self.getGroupsCalls = self.getGroupsCalls + 1
    return self.groups
end
function ADGraphManager:getGroupByName(groupName)
    return self.groups[groupName]
end
function ADGraphManager:getMapMarkers()
    self.markerScans = self.markerScans + 1
    return self.mapMarkers
end
function ADGraphManager:getMapMarkerById(id)
    return self.mapMarkers[id]
end

AutoDrive.Hud = { gapWidth = 0.005, buttonWidth = 0.03, buttonHeight = 0.03, listItemHeight = 0.02 }
AutoDrive.currentColors = {
    ad_color_hudTextDefault = { 1, 1, 1, 1 },
    ad_color_hudTextHover = { 1, 1, 0, 1 },
    ad_color_hudTextSpecial = { 0, 1, 0, 1 },
    ad_color_hudTextHoverSpecial = { 0, 1, 1, 1 },
}
function AutoDrive.isEditorModeEnabled() return false end

--- A vehicle as the HUD sees it.
local function hudVehicle(groups)
    local v = TestSetup.vehicle()
    v.ad.groups = groups or {}
    v.ad.destinationFilterText = ''
    v.ad.stateModule = {
        getMode = function() return AutoDrive.MODE_DRIVETO end,
        isActive = function() return false end,
        getFirstMarker = function() return nil end,
        getFirstMarkerId = function() return nil end,
        getSecondMarker = function() return nil end,
        getSecondMarkerId = function() return nil end,
        getFillType = function() return 0 end,
        getSelectedFillTypes = function() return {} end,
    }
    return v
end

--- A destination list with the given groups, without going through ADPullDownList:new - the
--- constructor needs overlays, and these tests are about the lookup and the draw loop.
local function destinationList(options, fakeGroupIDs)
    local list = ADPullDownList:create()
    list:init(0.5, 0.5, 0.3, 0.02)
    list.type = ADPullDownList.TYPE_TARGET
    list.state = ADPullDownList.STATE_EXPANDED
    list.isVisible = true
    list.selected = 1
    list.hovered = 1
    list.direction = ADPullDownList.EXPANDED_UP
    list.options = options
    list.fakeGroupIDs = fakeGroupIDs
    list.size = { width = 0.3, height = 0.02 }
    list.expandedSize = { width = 0.3, height = 0.3 }
    list.expandedBottom = 0.2
    list.rightIconPos = { x = 0.75, y = 0.5 }
    list.rightIconPos2 = { x = 0.72, y = 0.5 }
    list.rightIconPos3 = { x = 0.69, y = 0.5 }
    list.rightIconPos4 = { x = 0.66, y = 0.5 }
    list.iconSize = { width = 0.015, height = 0.015 }
    list.rowSize = { width = 0.015, height = 0.01 }
    list.ovTop = { render = function() end }
    list.ovStretch = { render = function() end }
    list.ovBottom = { render = function() end }
    return list
end

local function resetUiStubs()
    UiTest.rendered = {}
    ADGraphManager.groups = { All = 1 }
    ADGraphManager.mapMarkers = {}
    ADGraphManager.getGroupsCalls = 0
    ADGraphManager.markerScans = 0
    ADPullDownList.ovPlus = { overlayId = 1 }
    ADPullDownList.ovMinus = { overlayId = 2 }
    ADPullDownList.ovFilter = { overlayId = 3 }
    ADPullDownList.ovCollapseAll = { overlayId = 4 }
    AutoDrive.testSettings['useFolders'] = false
    AutoDrive.testSettings['guiScale'] = 0
end

------------------------------------------------------------------------------------------------------------------------
--- B6 - the 15 "loadGUI failed" lines in every player's log
------------------------------------------------------------------------------------------------------------------------
--- What actually happens in the game: Gui:loadGui does return the created GuiElement, but
--- Utils.prependedFunction/appendedFunction call the original without passing its result on. Any
--- mod that wraps Gui.loadGui - FS25_Financing ships exactly that wrapper - therefore turns every
--- load in the game into a nil return, which AutoDrive read as failure.
TestLoadGui = {}

local GUI_NAMES = {
    'ADEnterDriverNameGui', 'ADEnterTargetNameGui', 'ADEnterGroupNameGui',
    'ADEnterDestinationFilterGui', 'ADRoutesManagerGui', 'ADNotificationsHistoryGui',
    'ADColorSettingsGui', 'ADScanConfirmationGui', 'ADSettings',
}
local FRAME_NAMES = {
    'autoDriveGlobalSettings', 'autoDriveUserSettings', 'autoDriveVehicleSettings',
    'autoDriveCombineUnloadSettings', 'autoDriveEnvironmentSettings', 'autoDriveDebugSettings',
}

--- @param swallowReturn boolean mimic a mod having wrapped Gui.loadGui
--- @param doNotRegister string|nil name of a gui that genuinely fails to load
local function fakeGui(swallowReturn, doNotRegister)
    local gui = { guis = {}, frames = {}, loaded = {} }
    function gui:loadProfiles() end
    function gui:loadGui(xmlFilename, name, controller, isFrame)
        table.insert(self.loaded, name)
        if name == doNotRegister then
            return nil    -- the engine logs its own error and deletes the controller
        end
        local element = { name = name, xmlFilename = xmlFilename, controller = controller }
        if isFrame then
            self.frames[name] = element
        else
            self.guis[name] = element
        end
        if swallowReturn then
            return nil
        end
        return element
    end
    return gui
end

function TestLoadGui:setUp()
    TestSetup.reset()
    resetUiStubs()
    self.originalGui = g_gui
    g_autoDriveUIConfigPath = 'gui/textures.xml'
    for _, name in ipairs({ 'ADEnterDriverNameGui', 'ADEnterTargetNameGui', 'ADEnterGroupNameGui',
                            'ADEnterDestinationFilterGui', 'ADRoutesManagerGui',
                            'ADNotificationsHistoryGui', 'ADColorSettingsGui',
                            'ADScanConfirmationGui', 'ADSettingsPage', 'ADDebugSettingsPage',
                            'ADSettings' }) do
        _G[name] = { new = function() return {} end }
    end
end

function TestLoadGui:tearDown()
    g_gui = self.originalGui
end

--- The regression itself: a wrapped loadGui must not produce a single line in the log.
function TestLoadGui:testWrappedLoadGuiIsNotReportedAsFailure()
    g_gui = fakeGui(true)
    AutoDrive.currentDebugChannelMask = AutoDrive.DC_ALL
    AutoDrive:loadGUI()
    lu.assertEquals(Logging.countMatching('loadGUI failed'), 0,
        'a load that registered its gui must not be reported as failed, whatever it returned')
end

--- The mod must still load every one of its guis - the fix is about the check, not the loading.
function TestLoadGui:testAllGuisAreLoadedAndRegistered()
    g_gui = fakeGui(true)
    AutoDrive:loadGUI()
    lu.assertEquals(#g_gui.loaded, #GUI_NAMES + #FRAME_NAMES)
    for _, name in ipairs(GUI_NAMES) do
        lu.assertNotNil(g_gui.guis[name], name .. ' must be registered as a gui')
    end
    for _, name in ipairs(FRAME_NAMES) do
        lu.assertNotNil(g_gui.frames[name], name .. ' must be registered as a frame')
    end
end

--- A gui that really does not come up is still reported, and says which one it was.
function TestLoadGui:testGenuineFailureIsReportedOnTheDebugChannel()
    g_gui = fakeGui(false, 'ADRoutesManagerGui')
    AutoDrive.currentDebugChannelMask = AutoDrive.DC_DEVINFO
    AutoDrive:loadGUI()
    lu.assertEquals(Logging.countMatching('loadGUI failed'), 1)
    lu.assertEquals(Logging.countMatching('ADRoutesManagerGui'), 1)
end

--- A frame is registered in g_gui.frames, not in g_gui.guis - checking the wrong table would
--- report all six settings pages as broken.
function TestLoadGui:testFramesAreNotReportedAsFailure()
    g_gui = fakeGui(true)
    AutoDrive.currentDebugChannelMask = AutoDrive.DC_ALL
    AutoDrive:loadGUI()
    for _, name in ipairs(FRAME_NAMES) do
        lu.assertEquals(Logging.countMatching(name), 0, name .. ' loaded fine and must not be logged')
    end
end

--- And with the debug channels off - the state every player is in - nothing is logged at all.
function TestLoadGui:testNothingIsLoggedWithDebugChannelsOff()
    g_gui = fakeGui(false, 'ADSettings')
    AutoDrive.currentDebugChannelMask = AutoDrive.DC_NONE
    AutoDrive:loadGUI()
    lu.assertEquals(Logging.countMatching('loadGUI'), 0,
        'the message must go through a debug channel, not unconditionally into the log')
end

------------------------------------------------------------------------------------------------------------------------
--- A47 - the group name lookup
------------------------------------------------------------------------------------------------------------------------
TestGroupNameLookup = {}

function TestGroupNameLookup:setUp()
    TestSetup.reset()
    resetUiStubs()
    ADGraphManager.groups = { All = 1, Field = 2, Yard = 3, Shop = 4 }
    AutoDrive.testSettings['useFolders'] = true
end

local function fourGroupList()
    return destinationList(
        {
            { { displayName = 'a', returnValue = 11 } },
            { { displayName = 'b', returnValue = 12 } },
            { { displayName = 'c', returnValue = 13 } },
            { { displayName = 'd', returnValue = 14 } },
        },
        { 1, 2, 3, 4 })
end

--- The lookup must still produce the right names.
function TestGroupNameLookup:testFolderNamesAreResolved()
    local list = fourGroupList()
    local vehicle = hudVehicle({ All = false, Field = false, Yard = false, Shop = false })
    lu.assertEquals(list:getListElementByIndex(vehicle, 1).displayName, 'All')
    lu.assertEquals(list:getListElementByIndex(vehicle, 1).returnValue, 'All')
    lu.assertTrue(list:getListElementByIndex(vehicle, 1).isFolder)
    lu.assertEquals(list:getListElementByIndex(vehicle, 2).displayName, 'Field')
    lu.assertEquals(list:getListElementByIndex(vehicle, 4).displayName, 'Shop')
    lu.assertNil(list:getListElementByIndex(vehicle, 5))
end

--- ... while reading the group table once per walk instead of once per group visited.
function TestGroupNameLookup:testGroupTableIsReadOncePerLookup()
    local list = fourGroupList()
    local vehicle = hudVehicle({ All = false, Field = false, Yard = false, Shop = false })
    ADGraphManager.getGroupsCalls = 0
    list:getListElementByIndex(vehicle, 4)
    lu.assertEquals(ADGraphManager.getGroupsCalls, 1,
        'the id -> name lookup must be built once, not per group visited')
end

--- Opened folders show their entries, and the entries are found by index as before.
function TestGroupNameLookup:testEntriesOfOpenedFoldersAreListed()
    local list = fourGroupList()
    local vehicle = hudVehicle({ All = true, Field = false, Yard = false, Shop = false })
    lu.assertEquals(list:getListElementByIndex(vehicle, 1).displayName, 'All')
    local entry = list:getListElementByIndex(vehicle, 2)
    lu.assertEquals(entry.displayName, 'a')
    lu.assertFalse(entry.isFolder)
    lu.assertEquals(list:getListElementByIndex(vehicle, 3).displayName, 'Field')
end

function TestGroupNameLookup:testGetGroupNamesByIDInvertsTheGroupTable()
    local list = fourGroupList()
    local byId = list:getGroupNamesByID()
    lu.assertEquals(byId[1], 'All')
    lu.assertEquals(byId[4], 'Shop')
    lu.assertNil(byId[99])
end

------------------------------------------------------------------------------------------------------------------------
--- A48 - the first row of the list
------------------------------------------------------------------------------------------------------------------------
TestListDrawing = {}

function TestListDrawing:setUp()
    TestSetup.reset()
    resetUiStubs()
    AutoDrive.testSettings['useFolders'] = false
end

local function threeEntryList()
    return destinationList(
        { { { displayName = 'first', returnValue = 1 },
            { displayName = 'second', returnValue = 2 },
            { displayName = 'third', returnValue = 3 } } },
        { 1 })
end

function TestListDrawing:testEveryEntryIsDrawnOnce()
    local list = threeEntryList()
    list:onDraw(hudVehicle(), 1)
    lu.assertEquals(UiTest.rendered, { 'first', 'second', 'third' })
end

--- The defect: when the view had scrolled past the end of the list - a folder was collapsed, an
--- entry removed - the code reset the scroll position and tried to redo the row by assigning to
--- the loop control variable, which does nothing in Lua. The first entry was not drawn.
function TestListDrawing:testFirstRowIsNotLostAfterTheViewSnapsBack()
    local list = threeEntryList()
    list.selected = 7    -- points past the end of the list
    list.hovered = 7
    list:onDraw(hudVehicle(), 1)
    lu.assertEquals(list.selected, 1, 'the view must snap back to the top')
    lu.assertEquals(list.hovered, 1)
    lu.assertEquals(UiTest.rendered[1], 'first',
        'the first entry must be drawn in the same frame the view snapped back')
    lu.assertEquals(UiTest.rendered, { 'first', 'second', 'third' })
end

--- An empty list draws nothing and, above all, terminates.
function TestListDrawing:testEmptyListDrawsNothing()
    local list = destinationList({ {} }, { 1 })
    list:onDraw(hudVehicle(), 1)
    lu.assertEquals(#UiTest.rendered, 0)
end

--- A view scrolled part way down still starts at the row it is scrolled to.
function TestListDrawing:testScrolledViewStartsAtTheSelectedEntry()
    local list = threeEntryList()
    list.selected = 2
    list.hovered = 2
    list:onDraw(hudVehicle(), 1)
    lu.assertEquals(UiTest.rendered, { 'second', 'third' })
    lu.assertEquals(list.selected, 2, 'a view that still has entries must not snap back')
end

------------------------------------------------------------------------------------------------------------------------
--- A24 - dragging the HUD
------------------------------------------------------------------------------------------------------------------------
TestHudDragging = {}

function TestHudDragging:setUp()
    TestSetup.reset()
    resetUiStubs()
    self.hud = setmetatable({}, { __index = AutoDriveHud })
    self.hud.posX = 0.5
    self.hud.posY = 0.5
    self.hud.rebuilds = {}
    self.hud.createHudAt = function(hud, x, y, reuseListData)
        table.insert(hud.rebuilds, { x = x, y = y, reuseListData = reuseListData })
        hud.posX = x
        hud.posY = y
    end
    ADUserDataManager = { sendToServer = function() end }
end

function TestHudDragging:testDragRebuildsWithTheListDataItAlreadyHas()
    self.hud:startMovingHud(0.5, 0.5)
    self.hud:moveHud(0.55, 0.6)
    lu.assertEquals(#self.hud.rebuilds, 1)
    lu.assertTrue(self.hud.rebuilds[1].reuseListData,
        'a drag only moves the HUD and must not re-derive the pull-down list contents')
end

--- moveHud runs for every mouse event while the header is held, not only for actual movement.
function TestHudDragging:testStandingStillDoesNotRebuild()
    self.hud:startMovingHud(0.5, 0.5)
    self.hud:moveHud(0.5, 0.5)
    self.hud:moveHud(0.5, 0.5)
    lu.assertEquals(#self.hud.rebuilds, 0)
end

function TestHudDragging:testDragFollowsTheMouse()
    self.hud:startMovingHud(0.5, 0.5)
    self.hud:moveHud(0.55, 0.5)
    self.hud:moveHud(0.60, 0.5)
    lu.assertEquals(#self.hud.rebuilds, 2, 'the HUD must keep following the mouse while dragged')
    lu.assertAlmostEquals(self.hud.rebuilds[2].x, 0.60, 1e-9)
end

--- When the drag is over the lists are derived once, in full.
function TestHudDragging:testDragEndDerivesTheListsAgain()
    self.hud:startMovingHud(0.5, 0.5)
    self.hud:moveHud(0.55, 0.5)
    local duringDrag = #self.hud.rebuilds
    self.hud:stopMovingHud()
    lu.assertFalse(self.hud.isMoving)
    lu.assertEquals(#self.hud.rebuilds, duringDrag + 1, 'the drag must end with one full rebuild')
    local last = self.hud.rebuilds[#self.hud.rebuilds]
    lu.assertNil(last.reuseListData, 'the rebuild after the drag must derive the lists again')
end

function TestHudDragging:testMoveHudDoesNothingWhenNotDragging()
    self.hud.isMoving = false
    self.hud.lastMousePosX = 0.5
    self.hud.lastMousePosY = 0.5
    self.hud:moveHud(0.9, 0.9)
    lu.assertEquals(#self.hud.rebuilds, 0)
end

--- The other half of A24, in the pull-down list itself: a list that takes over from the one it
--- replaces must not scan the map markers again.
TestListReuse = {}

function TestListReuse:setUp()
    TestSetup.reset()
    resetUiStubs()
    ADGraphManager.mapMarkers = {
        [1] = { name = 'Barn', group = 'All' },
        [2] = { name = 'Field', group = 'All' },
    }
    g_overlayManager = {
        addTextureConfigFile = function() end,
        createOverlay = function()
            return { render = function() end, setSliceId = function() end, overlayId = 1 }
        end,
    }
end

--- How AutoDriveHud:createHudAt picks the lists out of its element table when the HUD is dragged.
--- It must not use isa(): that walks up to ADGenericHudElement, which is a plain table without a
--- superClass method, so asking any other HUD element whether it is a pull-down list throws.
function TestListReuse:testPullDownListsAreTellableFromOtherHudElements()
    lu.assertIs(ADPullDownList:create():class(), ADPullDownList)
    lu.assertNotIs(ADHudSettingsButton:create():class(), ADPullDownList)
    lu.assertErrorMsgContains('superClass', function()
        ADHudSettingsButton:create():isa(ADPullDownList)
    end)
end

function TestListReuse:testNewListDerivesItsContents()
    local list = ADPullDownList:new(0.5, 0.5, 0.3, 0.02, ADPullDownList.TYPE_TARGET, 1)
    lu.assertEquals(ADGraphManager.markerScans, 1)
    lu.assertEquals(#list.options[1], 2)
end

function TestListReuse:testReusingListSkipsTheMarkerScan()
    local previous = ADPullDownList:new(0.5, 0.5, 0.3, 0.02, ADPullDownList.TYPE_TARGET, 1)
    ADGraphManager.markerScans = 0
    local reused = ADPullDownList:new(0.4, 0.4, 0.3, 0.02, ADPullDownList.TYPE_TARGET, 1, previous)
    lu.assertEquals(ADGraphManager.markerScans, 0,
        'a HUD drag must not re-scan every map marker')
    lu.assertIs(reused.options, previous.options)
    lu.assertIs(reused.fakeGroupIDs, previous.fakeGroupIDs)
    lu.assertAlmostEquals(reused.position.x, 0.4, 1e-9, 'the reused list still moves with the HUD')
end

------------------------------------------------------------------------------------------------------------------------
--- A23 - the settings button must claim the click it handled
------------------------------------------------------------------------------------------------------------------------
TestSettingsButton = {}

function TestSettingsButton:setUp()
    TestSetup.reset()
    resetUiStubs()
    self.applied = {}
    AutoDrive.settings = { rotateTargets = { values = { 1, 2, 3 } } }
    AutoDrive.getSettingState = function(name) return AutoDrive.testSettings[name] or 1 end
    AutoDrive.setSettingState = function(name, state) table.insert(self.applied, { name, state }) end
    AutoDriveUpdateSettingsEvent = { sendEvent = function() end }

    self.button = ADHudSettingsButton:create()
    self.button:init(0.5, 0.5, 0.03, 0.03)
    self.button.setting = 'rotateTargets'
    self.button.toolTip = 'gui_ad_rotateTargets'
    self.button.state = 1
    self.button.isVisible = true
end

--- The defect: the click was handled here and then handed on to the editor waypoint handling in
--- AutoDriveHud:mouseEvent, because act always said "not handled".
function TestSettingsButton:testHandledClickIsReportedAsHandled()
    local handled = self.button:act(hudVehicle(), 0.5, 0.5, false, true, 1)
    lu.assertTrue(handled, 'a click that changed the setting must be reported as consumed')
    lu.assertEquals(self.applied, { { 'rotateTargets', 2 } })
end

--- The press belonging to that click must not leak either, and must not play a second click sound.
function TestSettingsButton:testButtonPressIsConsumedSilently()
    local handled, silent = self.button:act(hudVehicle(), 0.5, 0.5, true, false, 1)
    lu.assertTrue(handled)
    lu.assertTrue(silent)
    lu.assertEquals(#self.applied, 0, 'the press must not already change the setting')
end

--- Hovering is not a click: it has to stay unhandled or the rest of the HUD never sees the mouse.
function TestSettingsButton:testHoverIsNotConsumed()
    lu.assertFalse(self.button:act(hudVehicle(), 0.5, 0.5, false, false, 0))
end

function TestSettingsButton:testInvisibleButtonConsumesNothing()
    self.button.isVisible = false
    lu.assertFalse(self.button:act(hudVehicle(), 0.5, 0.5, false, true, 1))
    lu.assertEquals(#self.applied, 0)
end

--- Handling the click is still what it always was: advance the setting, wrapping round.
function TestSettingsButton:testSettingWrapsAroundAtTheLastValue()
    AutoDrive.testSettings['rotateTargets'] = 3
    self.button:act(hudVehicle(), 0.5, 0.5, false, true, 1)
    lu.assertEquals(self.applied, { { 'rotateTargets', 1 } })
end

function TestSettingsButton:testToolTipIsStillSet()
    local vehicle = hudVehicle()
    self.button:act(vehicle, 0.5, 0.5, false, true, 1)
    lu.assertEquals(vehicle.ad.sToolTip, 'gui_ad_rotateTargets')
    lu.assertTrue(vehicle.ad.toolTipIsSetting)
end


------------------------------------------------------------------------------------------------------------------------
--- Wrapping the HUD header
---
--- The header is a handful of fields joined with " - ": the mod name, the version, the mode, the
--- drive time, the tooltip, the task. It wraps to fit the panel. It used to wrap by splitting on
--- every "-" in the whole string and then chopping one character off the front of each continued
--- line, to drop the leading space the separator leaves. A hyphen INSIDE a field broke both halves
--- at once - the line was cut mid-word, and the fragment has no leading space to drop, so the chop
--- ate its first letter. Seen in game: a line reading "dition".
------------------------------------------------------------------------------------------------------------------------
TestHeaderWrap = {}

function TestHeaderWrap:setUp()
    TestSetup.reset()
    require('HudIcon')
    -- one unit per character, so a maxLength is a character count and the numbers below are readable
    getTextWidth = function(_, text) return #text end
end

local function wrap(text, maxLength)
    return ADHudIcon.splitTextByLength(ADHudIcon, text, 1, maxLength)
end

--- Nothing may be lost, whatever the wrapping does. This is the property the bug violated.
function TestHeaderWrap:testNoCharacterIsEverDropped()
    local text = 'AutoDrive - 3.0.1.2-krt-special-edition - Drescher folgen - Hof 4 - Silo'
    for maxLength = 8, 90 do
        local joined = table.concat(wrap(text, maxLength), ' - ')
        lu.assertEquals(joined, text, string.format('at a width of %d the text came back changed', maxLength))
    end
end

--- The reported case, stated as itself.
function TestHeaderWrap:testAHyphenatedVersionKeepsItsWord()
    local lines = wrap('AutoDrive - 3.0.1.2-krt-special-edition - Drescher abfahren', 30)
    for _, line in ipairs(lines) do
        lu.assertNil(line:match('^dition'), 'the wrap ate the start of a word: ' .. line)
    end
    lu.assertStrContains(table.concat(lines, ' - '), '3.0.1.2-krt-special-edition')
end

--- A vehicle name with a hyphen in it is ordinary - "832 Vario Gen5 - 60km/h" is a stock one - and
--- must not become a wrap point that swallows a character either.
function TestHeaderWrap:testAHyphenInsideAFieldIsNotAWrapPoint()
    local lines = wrap('AutoDrive - 3.0.1.2 - 832 Vario Gen5 - 60km/h', 1000)
    lu.assertEquals(#lines, 1, 'it all fits, so it must stay on one line')
    lu.assertEquals(lines[1], 'AutoDrive - 3.0.1.2 - 832 Vario Gen5 - 60km/h')
end

--- And it still wraps when it has to.
function TestHeaderWrap:testItStillWrapsWhenTooLong()
    local lines = wrap('AutoDrive - 3.0.1.2 - Drescher folgen - Hof 4', 20)
    lu.assertTrue(#lines > 1, 'a long header has to be broken up')
end

function TestHeaderWrap:testASingleFieldSurvives()
    lu.assertEquals(wrap('AutoDrive', 100), { 'AutoDrive' })
end

os.exit(lu.LuaUnit.run())
