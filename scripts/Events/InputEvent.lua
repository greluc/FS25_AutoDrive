AutoDriveInputEventEvent = {}
AutoDriveInputEventEvent_mt = Class(AutoDriveInputEventEvent, Event)

InitEventClass(AutoDriveInputEventEvent, "AutoDriveInputEventEvent")

function AutoDriveInputEventEvent.emptyNew()
    local self = Event.new(AutoDriveInputEventEvent_mt)
    return self
end

function AutoDriveInputEventEvent.new(vehicle, inputId, farmId)
    local self = AutoDriveInputEventEvent.emptyNew()
    self.vehicle = vehicle
    self.inputId = inputId
    self.farmId = farmId
    return self
end

function AutoDriveInputEventEvent:writeStream(streamId, connection)
    NetworkUtil.writeNodeObjectId(streamId, NetworkUtil.getObjectId(self.vehicle))
    streamWriteUInt8(streamId, self.inputId)
    streamWriteUInt8(streamId, self.farmId)
end

function AutoDriveInputEventEvent:readStream(streamId, connection)
    self.vehicle = NetworkUtil.getObject(NetworkUtil.readNodeObjectId(streamId))
    self.inputId = streamReadUInt8(streamId)
    self.farmId = streamReadUInt8(streamId)
    self:run(connection)
end

function AutoDriveInputEventEvent:run(connection)
    if g_server ~= nil then
        --- nil when the vehicle was sold, or not yet created on this machine. See HudInputEvent for
        --- the same guard and the same reason.
        if self.vehicle == nil then
            return
        end
        local input = ADInputManager.idsToInputs[self.inputId]
        if input == nil then
            -- an id we do not know: an older or newer AutoDrive on the other end of the connection
            return
        end
        --- The farm id is taken from the connection, not from the wire. A client writes it itself,
        --- so trusting it lets one player act as another's farm - and the id is only ever used to
        --- decide what this player may do.
        local farmId = self.farmId
        local user = connection ~= nil and g_currentMission.userManager ~= nil and
                g_currentMission.userManager:getUserByConnection(connection)
        if user ~= nil then
            local farm = g_farmManager:getFarmByUserId(user:getId())
            if farm ~= nil then
                farmId = farm.farmId
            end
        end
        --- Only the inputs that hire a helper are gated, and only here, where the connection is
        --- known. getHasPlayerPermission short-circuits to true whenever it is called on the server
        --- with no connection (FSBaseMission:getHasPlayerPermission), so the same check written
        --- inside ADInputManager would have been a no-op that looked like a check.
        ---
        --- Stopping is deliberately NOT gated. The game's own AIJobStopEvent has no permission check
        --- either, so gating it would make AutoDrive stricter than the game it runs in: a player
        --- allowed to stop a helper by hand could not stop one through us.
        if AutoDriveInputEventEvent.HIRING_INPUTS[input] and connection ~= nil and
                g_currentMission ~= nil and g_currentMission.getHasPlayerPermission ~= nil and
                not g_currentMission:getHasPlayerPermission('hireAssistant', connection, farmId) then
            return
        end
        ADInputManager:onInputCall(self.vehicle, input, farmId, false)
    end
end

--- The inputs that end up at g_helperManager:useHelper, directly or through input_start_stop.
AutoDriveInputEventEvent.HIRING_INPUTS = {
    input_start_stop = true,
    input_parkVehicle = true,
    input_refuelVehicle = true,
    input_repairVehicle = true,
}

function AutoDriveInputEventEvent.sendEvent(vehicle, inputId, farmId)
    if g_client ~= nil then
        -- Client have to send to server
        g_client:getServerConnection():sendEvent(AutoDriveInputEventEvent.new(vehicle, inputId, farmId))
    end
end
