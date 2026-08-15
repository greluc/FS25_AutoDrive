ADMultipleTargetsManager = {}

function ADMultipleTargetsManager:load()
    self.groups = {}
    for groupName, _ in pairs(ADGraphManager:getGroups()) do
        self.groups[groupName] = {}
        self.groups[groupName].lastTarget = nil
        self.groups[groupName].lastVehicle = nil
    end
    self.pickups = {}
    for groupName, _ in pairs(ADGraphManager:getGroups()) do
        self.pickups[groupName] = {}
        self.pickups[groupName].lastTarget = nil
        self.pickups[groupName].lastVehicle = nil
    end
end

function ADMultipleTargetsManager:getNextTarget(driver, forcedSkip)
    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
        AutoDrive.debugPrint(driver, AutoDrive.DC_PATHINFO, "ADMultipleTargetsManager:getNextTarget driver.ad.stateModule:getSecondMarkerName() %s", tostring(driver.ad.stateModule:getSecondMarkerName()))
    end
    local target = driver.ad.stateModule:getSecondMarker().markerIndex
    local groupName = driver.ad.stateModule:getSecondMarker().group

    if self.groups[groupName] == nil then
        AutoDrive.debugPrint(driver, AutoDrive.DC_PATHINFO, "ADMultipleTargetsManager:getNextTarget self.groups[groupName] == nil")
        self.groups[groupName] = {}
        self.groups[groupName].lastTarget = nil
        self.groups[groupName].lastVehicle = nil
    end

    if self.groups[groupName].lastVehicle == nil or (driver ~= self.groups[groupName].lastVehicle or forcedSkip) then
        if groupName ~= "All" then
            if self.groups[groupName] ~= nil then
                if self.groups[groupName].lastTarget == nil then
                    self.groups[groupName].lastTarget = driver.ad.stateModule:getSecondMarker().markerIndex
                    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
                        AutoDrive.debugPrint(driver, AutoDrive.DC_PATHINFO, "ADMultipleTargetsManager:getNextTarget lastTarget == nil lastTarget %s", tostring(driver.ad.stateModule:getSecondMarkerName()))
                    end
                else
                    local nextMarkerInGroup = nil
                    local markerSeen = false
                    local firstMarkerInGroup = nil
                    for _, marker in ipairs(ADGraphManager:getMapMarkersInGroup(groupName)) do
                        if marker.group == groupName then
                            if firstMarkerInGroup == nil then
                                firstMarkerInGroup = marker.markerIndex
                            end

                            if markerSeen and nextMarkerInGroup == nil then
                                nextMarkerInGroup = marker.markerIndex
                            end

                            if marker.markerIndex == self.groups[groupName].lastTarget then
                                markerSeen = true
                            end
                        end
                    end

                    if nextMarkerInGroup ~= nil then
                        target = nextMarkerInGroup
                        if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
                            AutoDrive.debugPrint(driver, AutoDrive.DC_PATHINFO, "ADMultipleTargetsManager:getNextTarget nextMarkerInGroup ~= nil target %s", tostring(ADGraphManager:getMapMarkerById(target).name))
                        end
                    elseif firstMarkerInGroup ~= nil then
                        target = firstMarkerInGroup
                        if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
                            AutoDrive.debugPrint(driver, AutoDrive.DC_PATHINFO, "ADMultipleTargetsManager:getNextTarget firstMarkerInGroup ~= nil target %s", tostring(ADGraphManager:getMapMarkerById(target).name))
                        end
                    end
                    self.groups[groupName].lastTarget = target
                    self.groups[groupName].lastVehicle = driver
                end
            end
        end
    end

    if self.groups[groupName].lastVehicle == nil then
        self.groups[groupName].lastVehicle = driver
        self.groups[groupName].lastTarget = target
    end
    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
        AutoDrive.debugPrint(driver, AutoDrive.DC_PATHINFO, "ADMultipleTargetsManager:getNextTarget end target %s", tostring(ADGraphManager:getMapMarkerById(target).name))
    end

    return target
end

function ADMultipleTargetsManager:getNextPickup(driver, forcedSkip)
    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
        AutoDrive.debugPrint(driver, AutoDrive.DC_PATHINFO, "ADMultipleTargetsManager:getNextPickup driver.ad.stateModule:getFirstMarkerName() %s", tostring(driver.ad.stateModule:getFirstMarkerName()))
    end
    local target = driver.ad.stateModule:getFirstMarker().markerIndex
    local groupName = driver.ad.stateModule:getFirstMarker().group
    if self.pickups[groupName] == nil then
        AutoDrive.debugPrint(driver, AutoDrive.DC_PATHINFO, "ADMultipleTargetsManager:getNextPickup self.pickups[groupName] == nil")
        self.pickups[groupName] = {}
        self.pickups[groupName].lastTarget = nil
        self.pickups[groupName].lastVehicle = nil
    end

    if self.pickups[groupName].lastVehicle == nil or (driver ~= self.pickups[groupName].lastVehicle or forcedSkip) then
        if groupName ~= "All" then
            if self.pickups[groupName] ~= nil then
                if self.pickups[groupName].lastTarget == nil then
                    self.pickups[groupName].lastTarget = driver.ad.stateModule:getFirstMarker().markerIndex
                    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
                        AutoDrive.debugPrint(driver, AutoDrive.DC_PATHINFO, "ADMultipleTargetsManager:getNextPickup lastTarget == nil lastTarget %s", tostring(driver.ad.stateModule:getFirstMarkerName()))
                    end
                else
                    local nextMarkerInGroup = nil
                    local markerSeen = false
                    local firstMarkerInGroup = nil
                    for _, marker in ipairs(ADGraphManager:getMapMarkersInGroup(groupName)) do
                        if marker.group == groupName then
                            if firstMarkerInGroup == nil then
                                firstMarkerInGroup = marker.markerIndex
                            end

                            if markerSeen and nextMarkerInGroup == nil then
                                nextMarkerInGroup = marker.markerIndex
                            end

                            if marker.markerIndex == self.pickups[groupName].lastTarget then
                                markerSeen = true
                            end
                        end
                    end

                    if nextMarkerInGroup ~= nil then
                        target = nextMarkerInGroup
                        if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
                            AutoDrive.debugPrint(driver, AutoDrive.DC_PATHINFO, "ADMultipleTargetsManager:getNextPickup nextMarkerInGroup ~= nil target %s", tostring(ADGraphManager:getMapMarkerById(target).name))
                        end
                    elseif firstMarkerInGroup ~= nil then
                        target = firstMarkerInGroup
                        if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
                            AutoDrive.debugPrint(driver, AutoDrive.DC_PATHINFO, "ADMultipleTargetsManager:getNextPickup firstMarkerInGroup ~= nil target %s", tostring(ADGraphManager:getMapMarkerById(target).name))
                        end
                    end
                    self.pickups[groupName].lastTarget = target
                    self.pickups[groupName].lastVehicle = driver
                end
            end
        end
    end

    if self.pickups[groupName].lastVehicle == nil then
        self.pickups[groupName].lastVehicle = driver
        self.pickups[groupName].lastTarget = target
    end
        if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
            AutoDrive.debugPrint(driver, AutoDrive.DC_PATHINFO, "ADMultipleTargetsManager:getNextPickup end target %s", tostring(ADGraphManager:getMapMarkerById(target).name))
        end

    return target
end

