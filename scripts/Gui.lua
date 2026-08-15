--- Loads one gui-config and reports whether it is usable afterwards.
---
--- The return value of Gui:loadGui is NOT a usable success signal: Utils.prependedFunction and
--- Utils.appendedFunction call the original without passing its result on, so as soon as any mod
--- wraps Gui.loadGui - FS25_Financing does exactly that - every load in the game returns nil even
--- though it succeeded. Checking the registration the engine performs on success instead keeps
--- this honest no matter who else hooked the function.
local function loadADGui(xmlFilename, name, controller, isFrame)
	g_gui:loadGui(xmlFilename, name, controller, isFrame)

	local registry = g_gui.guis
	if isFrame then
		registry = g_gui.frames
	end
	local guiElement = registry[name]
	if guiElement == nil then
		AutoDrive.debugPrint(nil, AutoDrive.DC_DEVINFO, "AutoDrive:loadGUI failed to load %s", name)
	end
	return guiElement
end

function AutoDrive:loadGUI()
	g_overlayManager:addTextureConfigFile(g_autoDriveUIConfigPath, "ad_gui")
	g_gui:loadProfiles(AutoDrive.directory .. "gui/guiProfiles.xml")

	AutoDrive.gui = {}
	AutoDrive.gui.ADEnterDriverNameGui = ADEnterDriverNameGui.new()
	AutoDrive.gui.ADEnterTargetNameGui = ADEnterTargetNameGui.new()
	AutoDrive.gui.ADEnterGroupNameGui = ADEnterGroupNameGui.new()
	AutoDrive.gui.ADEnterDestinationFilterGui = ADEnterDestinationFilterGui.new()
	AutoDrive.gui.ADRoutesManagerGui = ADRoutesManagerGui.new()
	AutoDrive.gui.ADNotificationsHistoryGui = ADNotificationsHistoryGui.new()
	AutoDrive.gui.ADColorSettingsGui = ADColorSettingsGui.new()
	AutoDrive.gui.ADScanConfirmationGui = ADScanConfirmationGui.new()

	loadADGui(AutoDrive.directory .. "gui/enterDriverNameGUI.xml", "ADEnterDriverNameGui", AutoDrive.gui.ADEnterDriverNameGui)

	loadADGui(AutoDrive.directory .. "gui/enterTargetNameGUI.xml", "ADEnterTargetNameGui", AutoDrive.gui.ADEnterTargetNameGui)

	loadADGui(AutoDrive.directory .. "gui/enterGroupNameGUI.xml", "ADEnterGroupNameGui", AutoDrive.gui.ADEnterGroupNameGui)

	loadADGui(AutoDrive.directory .. "gui/enterDestinationFilterGUI.xml", "ADEnterDestinationFilterGui", AutoDrive.gui.ADEnterDestinationFilterGui)

	loadADGui(AutoDrive.directory .. "gui/routesManagerGUI.xml", "ADRoutesManagerGui", AutoDrive.gui.ADRoutesManagerGui)

	loadADGui(AutoDrive.directory .. "gui/notificationsHistoryGUI.xml", "ADNotificationsHistoryGui", AutoDrive.gui.ADNotificationsHistoryGui)

	loadADGui(AutoDrive.directory .. "gui/colorSettingsGUI.xml", "ADColorSettingsGui", AutoDrive.gui.ADColorSettingsGui)

	loadADGui(AutoDrive.directory .. "gui/scanConfirmationGUI.xml", "ADScanConfirmationGui", AutoDrive.gui.ADScanConfirmationGui)

	AutoDrive.gui.ADGlobalSettingsPage = ADSettingsPage.new()
	AutoDrive.gui.ADUserSettingsPage = ADSettingsPage.new()
	AutoDrive.gui.ADVehicleSettingsPage = ADSettingsPage.new()
	AutoDrive.gui.ADCombineUnloadSettingsPage = ADSettingsPage.new()
	AutoDrive.gui.ADEnvironmentSettingsPage = ADSettingsPage.new()
	AutoDrive.gui.ADDebugSettingsPage = ADDebugSettingsPage.new()

	AutoDrive.gui.ADSettings = ADSettings.new()

	loadADGui(AutoDrive.directory .. "gui/globalSettingsPage.xml", "autoDriveGlobalSettings", AutoDrive.gui.ADGlobalSettingsPage, true)

	loadADGui(AutoDrive.directory .. "gui/userSettingsPage.xml", "autoDriveUserSettings", AutoDrive.gui.ADUserSettingsPage, true)

	loadADGui(AutoDrive.directory .. "gui/vehicleSettingsPage.xml", "autoDriveVehicleSettings", AutoDrive.gui.ADVehicleSettingsPage, true)

	loadADGui(AutoDrive.directory .. "gui/combineUnloadSettingsPage.xml", "autoDriveCombineUnloadSettings", AutoDrive.gui.ADCombineUnloadSettingsPage, true)

	loadADGui(AutoDrive.directory .. "gui/environmentSettingsPage.xml", "autoDriveEnvironmentSettings", AutoDrive.gui.ADEnvironmentSettingsPage, true)

	loadADGui(AutoDrive.directory .. "gui/debugSettingsPage.xml", "autoDriveDebugSettings", AutoDrive.gui.ADDebugSettingsPage, true)

	loadADGui(AutoDrive.directory .. "gui/settings.xml", "ADSettings", AutoDrive.gui.ADSettings)
end

function AutoDrive.onOpenSettings()
	if AutoDrive.gui.ADSettings.isOpen then
		AutoDrive.gui.ADSettings:onClickBack()
	elseif not g_gui:getIsGuiVisible() then
		g_gui:showGui("ADSettings")
	end
end

function AutoDrive.onOpenEnterDriverName()
	if not AutoDrive.gui.ADEnterDriverNameGui.isOpen then
		g_gui:showDialog("ADEnterDriverNameGui")
	end
end

function AutoDrive.onOpenEnterTargetName()
	if not AutoDrive.gui.ADEnterTargetNameGui.isOpen then
		g_gui:showDialog("ADEnterTargetNameGui")
	end
end

function AutoDrive.onOpenEnterGroupName()
	if not AutoDrive.gui.ADEnterGroupNameGui.isOpen then
		g_gui:showDialog("ADEnterGroupNameGui")
	end
end

function AutoDrive.onOpenEnterDestinationFilter()
	if not AutoDrive.gui.ADEnterDestinationFilterGui.isOpen then
		g_gui:showDialog("ADEnterDestinationFilterGui")
	end
end

function AutoDrive.onOpenRoutesManager()
	if not AutoDrive.gui.ADRoutesManagerGui.isOpen then
		g_gui:showDialog("ADRoutesManagerGui")
	end
end

function AutoDrive.onOpenNotificationsHistory()
	if not AutoDrive.gui.ADNotificationsHistoryGui.isOpen then
		g_gui:showDialog("ADNotificationsHistoryGui")
	end
end

function AutoDrive.onOpenColorSettings()
	if not AutoDrive.gui.ADColorSettingsGui.isOpen then
		g_gui:showDialog("ADColorSettingsGui")
	end
end

function AutoDrive.onOpenScanConfirmation()
	if not AutoDrive.gui.ADScanConfirmationGui.isOpen then
		g_gui:showDialog("ADScanConfirmationGui")
	end
end

function AutoDrive.showYesNoDialog(title, text, callback, target, ...)
	local dlg = g_gui:showDialog("YesNoDialog")
	dlg.target:setTitle(title)
	dlg.target.dialogTextElement:setText(text)
	dlg.target:setCallback(callback, target, ...)
end


ADGuiDebugMixin = {}

function ADGuiDebugMixin.new()
    return setmetatable({}, {__index = ADGuiDebugMixin})
end

function ADGuiDebugMixin:addTo(guiElement)
    guiElement.debugMsg = ADGuiDebugMixin.debugMsg
end

function ADGuiDebugMixin:debugMsg(...)
    if self.debug == true then
        AutoDrive.debugMsg(nil, ...)
    end
end
