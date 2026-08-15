function AutoDrive:checkForVehicleCollision(vehicle,boundingBox, excludedVehicles)
    if excludedVehicles == nil then
        excludedVehicles = {}
    end
    table.insert(excludedVehicles, vehicle)
    return AutoDrive.checkForVehiclesInBox(boundingBox, excludedVehicles)
end

function AutoDrive.checkForVehiclesInBox(boundingBox, excludedVehicles)
    for _, otherVehicle in pairs(AutoDrive.getAllVehicles()) do
        if otherVehicle ~= nil and otherVehicle.components ~= nil and otherVehicle.size.width ~= nil and otherVehicle.size.length ~= nil and otherVehicle.rootNode ~= nil then
            local x, y, z = getWorldTranslation(otherVehicle.components[1].node)
            local distance = MathUtil.vector2Length(boundingBox[1].x - x, boundingBox[1].z - z)
            -- cull by distance before anything else: this runs over every vehicle in the mission and
            -- checkIsConnected below walks the whole implement chain of each excluded vehicle
            if distance < 50 then
                local isExcluded = false
                if (otherVehicle.spec_conveyorBelt and otherVehicle.spec_motorized and otherVehicle.getIsMotorStarted and otherVehicle:getIsMotorStarted()) -- ignore operating conveyor belts
                    or (otherVehicle.trainSystem ~= nil) -- ignore train vehicles
                then
                    isExcluded = true
                elseif excludedVehicles ~= nil then
                    for _, excludedVehicle in pairs(excludedVehicles) do
                        if excludedVehicle == otherVehicle or AutoDrive:checkIsConnected(excludedVehicle, otherVehicle) then
                            isExcluded = true
                            break
                        end
                    end
                end
                if not isExcluded then
                    if AutoDrive.boxesIntersect(boundingBox, AutoDrive.getBoundingBoxForVehicle(otherVehicle)) == true then
                        if math.abs(boundingBox[1].y - y) < 10 then
                            return true
                        end
                    end
                end
            end
        end
    end

    return false
end

function AutoDrive.checkForVehiclePathInBox(boundingBox, minTurnRadius, searchingVehicle, currentVec)
    for _, otherVehicle in pairs(AutoDrive.getAllVehicles()) do
        if otherVehicle ~= nil and otherVehicle ~= searchingVehicle and otherVehicle.components ~= nil and otherVehicle.size.width ~= nil and otherVehicle.size.length ~= nil and otherVehicle.rootNode ~= nil then
            if minTurnRadius ~= nil and otherVehicle.ad ~= nil and otherVehicle.ad.drivePathModule ~= nil and otherVehicle.ad.stateModule:isActive() then
                local otherWPs, otherCurrentWp = otherVehicle.ad.drivePathModule:getWayPoints()
                local lastWp = nil
                -- check for other pathfinder steered vehicles and avoid any intersection with their routes
                if otherWPs ~= nil and otherWPs[otherCurrentWp] ~= nil and otherWPs[otherCurrentWp].isPathFinderPoint then
                    -- How much of the other vehicle's route to leave unprotected at its end.
                    --
                    -- This used to be a blanket "index < #otherWPs - 5", which exempted the last
                    -- five way points of every route - that is, its destination. With two unloaders
                    -- converging on the same harvester the destinations are exactly where the
                    -- routes overlap, so the reservation had a hole precisely where it was needed.
                    --
                    -- The exemption exists for a real reason though: when both vehicles are headed
                    -- for the same place, protecting that place would deadlock them - neither could
                    -- ever approach. So it is applied only then. When the targets are far apart,
                    -- the other vehicle's destination is protected like the rest of its route, and
                    -- ordering two unloaders onto the same harvester is left to the queue in
                    -- CombineUnloaderMode, which is the mechanism meant for it.
                    local tailExemption = 0
                    local ownWPs, _ = searchingVehicle.ad ~= nil and searchingVehicle.ad.drivePathModule ~= nil
                        and searchingVehicle.ad.drivePathModule:getWayPoints() or nil
                    if ownWPs ~= nil and #ownWPs > 0 and #otherWPs > 0 then
                        local ownTarget, otherTarget = ownWPs[#ownWPs], otherWPs[#otherWPs]
                        if ownTarget ~= nil and otherTarget ~= nil then
                            local dx, dz = ownTarget.x - otherTarget.x, ownTarget.z - otherTarget.z
                            if (dx * dx + dz * dz) < (AutoDrive.SHARED_TARGET_RANGE * AutoDrive.SHARED_TARGET_RANGE) then
                                tailExemption = 5
                            end
                        end
                    end

                    for index, wp in pairs(otherWPs) do
                        if lastWp ~= nil and wp.id == nil and index >= otherCurrentWp and wp.isPathFinderPoint and index > 2 and index < (#otherWPs - tailExemption) then
                            local widthOfColBox = minTurnRadius
                            local sideLength = widthOfColBox / 1.66

                            local vectorX = lastWp.x - wp.x
                            local vectorZ = lastWp.z - wp.z
                            local angleRad = math.atan2(-vectorZ, vectorX)
                            angleRad = AutoDrive.normalizeAngle(angleRad)
                            local length = math.sqrt(math.pow(vectorX, 2) + math.pow(vectorZ, 2)) + widthOfColBox

                            local leftAngle = AutoDrive.normalizeAngle(angleRad + math.rad(-90))
                            local rightAngle = AutoDrive.normalizeAngle(angleRad + math.rad(90))

                            local cornerX = wp.x - math.cos(leftAngle) * sideLength
                            local cornerZ = wp.z + math.sin(leftAngle) * sideLength

                            local corner2X = lastWp.x - math.cos(leftAngle) * sideLength
                            local corner2Z = lastWp.z + math.sin(leftAngle) * sideLength

                            local corner3X = lastWp.x - math.cos(rightAngle) * sideLength
                            local corner3Z = lastWp.z + math.sin(rightAngle) * sideLength

                            local corner4X = wp.x - math.cos(rightAngle) * sideLength
                            local corner4Z = wp.z + math.sin(rightAngle) * sideLength
                            local cellBox = AutoDrive.boundingBoxFromCorners(cornerX, cornerZ, corner2X, corner2Z, corner3X, corner3Z, corner4X, corner4Z)

                            -- the angle test only narrows the hit down to routes running roughly
                            -- parallel to us. Without a direction vector there is nothing to compare
                            -- against, so the test is skipped instead of counting as a mismatch -
                            -- otherwise the whole check could never report a hit for that caller.
                            local anglesSimilar = true
                            if currentVec ~= nil then
                                local dirVec = { x=vectorX, z = vectorZ}
                                local angleBetween = AutoDrive.angleBetween(dirVec, currentVec)
                                anglesSimilar = math.abs(angleBetween) < 20 or math.abs(angleBetween) > 160
                            end

                            if AutoDrive.boxesIntersect(boundingBox, cellBox) == true and anglesSimilar then
                                return true
                            end
                        end
                        lastWp = wp
                    end
                end
            end
        end
    end

    return false
end

function AutoDrive.getBoundingBoxForVehicleAtPosition(vehicle, position, force)
    local x, y, z = position.x, position.y, position.z
    local rx, _, rz = AutoDrive.localDirectionToWorld(vehicle, 0, 0, 1)
    local width, length = AutoDrive.getVehicleDimensions(vehicle, force)
    local lengthOffset = vehicle.size.lengthOffset
    local vehicleVector = {x = rx, z = rz}
    local ortho = {x = -vehicleVector.z, z = vehicleVector.x}

    local maxWidthLeft = (width / 2)
    local maxWidthRight = (width / 2)
    local maxLengthFront = (length / 2)
    local maxLengthBack = (length / 2)
    if vehicle and vehicle.ad and vehicle.ad.adDimensions and vehicle.ad.adDimensions.maxWidthLeft then
        maxWidthLeft = vehicle.ad.adDimensions.maxWidthLeft
        maxWidthRight = vehicle.ad.adDimensions.maxWidthRight
        maxLengthFront = vehicle.ad.adDimensions.maxLengthFront
        maxLengthBack = vehicle.ad.adDimensions.maxLengthBack
        lengthOffset = 0
    end

    local boundingBox = {}
    boundingBox[1] = {
        x = x + (maxWidthRight) * ortho.x - ((maxLengthBack) - lengthOffset) * vehicleVector.x,
        y = y + 2,
        z = z + (maxWidthRight) * ortho.z - ((maxLengthBack) - lengthOffset) * vehicleVector.z
    }
    boundingBox[2] = {
        x = x - (maxWidthLeft) * ortho.x - ((maxLengthBack) - lengthOffset) * vehicleVector.x,
        y = y + 2,
        z = z - (maxWidthLeft) * ortho.z - ((maxLengthBack) - lengthOffset) * vehicleVector.z
    }
    boundingBox[3] = {
        x = x - (maxWidthLeft) * ortho.x + ((maxLengthFront) + lengthOffset) * vehicleVector.x,
        y = y + 2,
        z = z - (maxWidthLeft) * ortho.z + ((maxLengthFront) + lengthOffset) * vehicleVector.z
    }
    boundingBox[4] = {
        x = x + (maxWidthRight) * ortho.x + ((maxLengthFront) + lengthOffset) * vehicleVector.x,
        y = y + 2,
        z = z + (maxWidthRight) * ortho.z + ((maxLengthFront) + lengthOffset) * vehicleVector.z
    }

    --ADDrawingManager:addLineTask(boundingBox[1].x, boundingBox[1].y, boundingBox[1].z, boundingBox[2].x, boundingBox[2].y, boundingBox[2].z, 1, 1, 1, 0)
    --ADDrawingManager:addLineTask(boundingBox[2].x, boundingBox[2].y, boundingBox[2].z, boundingBox[3].x, boundingBox[3].y, boundingBox[3].z, 1, 1, 1, 0)
    --ADDrawingManager:addLineTask(boundingBox[3].x, boundingBox[3].y, boundingBox[3].z, boundingBox[4].x, boundingBox[4].y, boundingBox[4].z, 1, 1, 1, 0)
    --ADDrawingManager:addLineTask(boundingBox[4].x, boundingBox[4].y, boundingBox[4].z, boundingBox[1].x, boundingBox[1].y, boundingBox[1].z, 1, 1, 1, 0)

    return boundingBox
end

function AutoDrive.getBoundingBoxForVehicle(vehicle, force)
    local x, y, z = getWorldTranslation(vehicle.components[1].node)

    local position = {x = x, y = y, z = z}

    return AutoDrive.getBoundingBoxForVehicleAtPosition(vehicle, position, force)
end

function AutoDrive.getDistanceBetween(vehicleOne, vehicleTwo)
    if vehicleOne == nil or vehicleTwo == nil then
        printCallstack()
        return math.huge
    end
    if not entityExists(vehicleOne.components[1].node) or not entityExists(vehicleTwo.components[1].node) then
        if not ADTable.contains(AutoDrive.shownErrors, "getDistanceBetween") then
            table.insert(AutoDrive.shownErrors, "getDistanceBetween")
            AutoDrive.debugMsg(nil, "AutoDrive.getDistanceBetween ERROR - entity vehicleOne %s vehicleTwo %s"
            , tostring(entityExists(vehicleOne.components[1].node))
            , tostring(entityExists(vehicleTwo.components[1].node))
            )
            printCallstack()
        end
        return math.huge
    end
    local x1, _, z1 = getWorldTranslation(vehicleOne.components[1].node)
    local x2, _, z2 = getWorldTranslation(vehicleTwo.components[1].node)

    return math.sqrt(math.pow(x2 - x1, 2) + math.pow(z2 - z1, 2))
end

function AutoDrive.debugDrawBoundingBoxForVehicles()
    local vehicle = AutoDrive.getControlledVehicle()
    if vehicle ~= nil and AutoDrive:getIsEntered(vehicle) then
        local PosX, _, PosZ = getWorldTranslation(vehicle.components[1].node)
        local maxDistance = AutoDrive.drawDistance
        for _, otherVehicle in pairs(AutoDrive.getAllVehicles()) do
            if otherVehicle ~= nil and otherVehicle.components ~= nil and otherVehicle.components[1].node ~= nil and otherVehicle.size.width ~= nil and otherVehicle.size.length ~= nil and otherVehicle.rootNode ~= nil then
                local x, _, z = getWorldTranslation(otherVehicle.components[1].node)
                local distance = MathUtil.vector2Length(PosX - x, PosZ - z)
                if distance < maxDistance then
                    local boundingBox = AutoDrive.getBoundingBoxForVehicle(otherVehicle)
                    ADDrawingManager:addLineTask(boundingBox[1].x, boundingBox[1].y, boundingBox[1].z, boundingBox[2].x, boundingBox[2].y, boundingBox[2].z, 1, 1, 1, 0)
                    ADDrawingManager:addLineTask(boundingBox[2].x, boundingBox[2].y, boundingBox[2].z, boundingBox[3].x, boundingBox[3].y, boundingBox[3].z, 1, 1, 1, 0)
                    ADDrawingManager:addLineTask(boundingBox[3].x, boundingBox[3].y, boundingBox[3].z, boundingBox[4].x, boundingBox[4].y, boundingBox[4].z, 1, 1, 1, 0)
                    ADDrawingManager:addLineTask(boundingBox[4].x, boundingBox[4].y, boundingBox[4].z, boundingBox[1].x, boundingBox[1].y, boundingBox[1].z, 1, 1, 1, 0)
                end
            end
        end
    end
end

-- dimension measurement of vehicles
ADDimensionSensor = {}
function ADDimensionSensor:new(vehicle)
    local o = {}
    setmetatable(o, self)
    self.__index = self
    o.vehicle = vehicle
    o.mask = 0
    o.collisionHits = 0
    o.selfHits = 0
    return o
end

function ADDimensionSensor:getRealVehicleDimensions()
    self.mask = AutoDrive.collisionMaskVehicleDimesions
    self.collisionHits = 0
    self.selfHits = 0
    local measureRange = math.max(self.vehicle.size.width + 1, self.vehicle.size.length + 1)

    local maxWidthLeft, maxWidthRight, maxLengthFront, maxLengthBack = 0,0,0,0

    if not entityExists(self.vehicle.components[1].node) then
        -- if selling attachments, the vehicle chain is not updated complete in 1 frame, so need this
        return 0, 0
    end
    local rx, ry, rz = getWorldRotation(self.vehicle.components[1].node)

    local function leftright(dimStart, dimEnd)
        local ret = self.vehicle.size.width / 2
        local selfHitCount = 0
        local minDistance = math.huge
        local diff = dimEnd - dimStart
        local step = 0.1
        if diff < 0 then
            step = -step
        end
        for distance = dimStart, dimEnd, step do
            local x,y,z = AutoDrive.localToWorld(self.vehicle, distance, self.vehicle.size.height / 2, 0)
            self.selfHits = 0
            if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
                -- DebugUtil.drawOverlapBox(x,y,z, rx, ry, rz, 0.1, measureRange, measureRange, 1, 1, 1)
            end
            self.collisionHits = overlapBox(x,y,z, rx, ry, rz, 0.1, measureRange, measureRange, "getRealVehicleDimensions_Callback", self, self.mask, true, true, true, true)
            if self.selfHits == 0 then
                -- found no collision with vehicle itself
                if selfHitCount < 5 then
                    selfHitCount = selfHitCount + 1
                    minDistance = math.min(minDistance, math.abs(distance))
                else
                    -- if n consecutive hits are reached, take the min. distance
                    ret = minDistance
                    break
                end
            else
                -- if hit itself, reset the counting
                selfHitCount = 0
                minDistance = math.huge
            end
        end
        return ret
    end
    maxWidthLeft = leftright(0, measureRange) -- measure to left
    maxWidthRight = leftright(0, -measureRange) -- measure to right

    local function frontback(dimStart, dimEnd, includeImplements)
        local ret = self.vehicle.size.length / 2
        local selfHitCount = 0
        local minDistance = math.huge
        local diff = dimEnd - dimStart
        local step = 0.1
        if diff < 0 then
            step = -step
        end
        if includeImplements == 1 then
            self.frontImplements, self.mostFrontImplement = AutoDrive.getFrontImplements(self.vehicle)
        else
            self.frontImplements, self.mostFrontImplement = nil, nil
        end
        for distance = dimStart, dimEnd, step do
            local x,y,z = AutoDrive.localToWorld(self.vehicle, 0, self.vehicle.size.height / 2, distance)
            self.selfHits = 0
            if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
                -- DebugUtil.drawOverlapBox(x,y,z, rx, ry, rz, measureRange, measureRange, 0.1, 1, 1, 1)
            end
            self.collisionHits = overlapBox(x,y,z, rx, ry, rz, measureRange, measureRange, 0.1, "getRealVehicleDimensions_Callback", self, self.mask, true, true, true, true)
            if self.selfHits == 0 then
                -- found no collision with vehicle itself
                if selfHitCount < 5 then
                    selfHitCount = selfHitCount + 1
                    minDistance = math.min(minDistance, math.abs(distance))
                else
                    -- if n consecutive hits are reached, take the min. distance
                    ret = minDistance
                    break
                end
            else
                -- if hit itself, reset the counting
                selfHitCount = 0
                minDistance = math.huge
            end
        end
        return ret
    end
    maxLengthFront = frontback(0, measureRange + 20, 1) -- measure to front
    maxLengthBack = frontback(0, -measureRange) -- measure to back
    self.vehicle.ad.adDimensions.maxWidthLeft = maxWidthLeft + AutoDrive.DIMENSION_ADDITION
    self.vehicle.ad.adDimensions.maxWidthRight = maxWidthRight + AutoDrive.DIMENSION_ADDITION
    self.vehicle.ad.adDimensions.maxLengthFront = maxLengthFront + AutoDrive.DIMENSION_ADDITION
    self.vehicle.ad.adDimensions.maxLengthBack = maxLengthBack + AutoDrive.DIMENSION_ADDITION
    local realWidth = 2 * math.max(maxWidthLeft, maxWidthRight) + AutoDrive.DIMENSION_ADDITION
    local realLegth = 2 * math.max(maxLengthFront, maxLengthBack) + AutoDrive.DIMENSION_ADDITION
    return realWidth, realLegth
end

function ADDimensionSensor:getRealVehicleDimensions_Callback(transformId)
    if transformId ~= 0 and transformId ~= g_currentMission.terrainRootNode then
        local collisionObject = g_currentMission.nodeToObject[transformId]
        if collisionObject ~= nil then
            if self.frontImplements and ADTable.contains(self.frontImplements, collisionObject) then
                self.selfHits = self.selfHits + 1
            elseif collisionObject == self.vehicle then
                self.selfHits = self.selfHits + 1
            end
        end
    end
end

function AutoDrive.getVehicleDimensions(vehicle, force)
    if vehicle == nil then
        return 0,0
    end
    if vehicle.spec_pallet then
        -- do not measure pallets
        return vehicle.size.width, vehicle.size.length
    end
    if vehicle.ad == nil then
        vehicle.ad = {}
    end
    if vehicle.ad.adDimensions == nil then
        vehicle.ad.adDimensions = {}
    end
    -- default taken from vehicle definition
    vehicle.ad.adDimensions.width, vehicle.ad.adDimensions.length = vehicle.size.width, vehicle.size.length
    if force then -- only measure if force true
        vehicle.ad.adDimensions = {}
        if vehicle.ad.adDimSensor == nil or vehicle.ad.adDimSensor.vehicle ~= vehicle then
            vehicle.ad.adDimSensor = ADDimensionSensor:new(vehicle)
        end
        if vehicle.ad.adDimSensor and vehicle.ad.adDimSensor.getRealVehicleDimensions then
            vehicle.ad.adDimensions.width, vehicle.ad.adDimensions.length = vehicle.ad.adDimSensor:getRealVehicleDimensions()
        end
    end
    return vehicle.ad.adDimensions.width, vehicle.ad.adDimensions.length
end

-- ATTENTION: This shall only be called if all components of the complete vehicle train is folded, in transport position etc. !!!
function AutoDrive.getAllVehicleDimensions(vehicle, force)
    local trailers = AutoDrive.getAllImplements(vehicle, true)
    for _, trailer in ipairs(trailers) do
        if not AutoDrive.hasVehicleRotatingYComponents(trailer) then
            AutoDrive.getVehicleDimensions(trailer, force)
        end
    end
end
