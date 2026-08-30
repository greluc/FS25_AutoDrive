ADCollSensorSplit = ADInheritsFrom(ADSensor)
ADCollSensorSplit.maskVehicles = CollisionFlag.VEHICLE + CollisionFlag.TRAFFIC_VEHICLE + CollisionFlag.TRAFFIC_VEHICLE_BLOCKING
ADCollSensorSplit.maskObjects = CollisionFlag.DEFAULT + CollisionFlag.STATIC_OBJECT + CollisionFlag.DYNAMIC_OBJECT + CollisionFlag.TERRAIN_DELTA + CollisionFlag.TREE + CollisionFlag.BUILDING

function ADCollSensorSplit:new(vehicle, sensorParameters)
    local o = ADCollSensorSplit:create()
    o:init(vehicle, ADSensor.TYPE_COLLISION, sensorParameters)
    o.hit = false
    o.newHit = false
    o.vehicle = vehicle;
    o.boxes = nil
    return o
end

function ADCollSensorSplit:onUpdate(dt)
    -- Here i want to generate an array of boxes instead of a large rotated single one

    -- Old
    --  |--------|     /\
    --  |Vehicle-|    /  \
    --  |--------|   /    \
    --               \     \
    --                \    /
    --                 \  /
    --                  \/
    --

     -- New
    --  |--------|/\
    --  |Vehicle-|/\\
    --  |--------|/\\\
    --            \ \\\
    --             \ \\\
    --              \ \/
    --               \/


    self.newHit = false
    self.newHitName = nil
    self.boxes = nil

    if self.sensorParameters.minDynamicLengthForVehicles then
        -- check for vehicles, AI vehicles
        self.boxes = self:getBoxShapes(self.sensorParameters.minDynamicLengthForVehicles)
        for _, box in pairs(self.boxes) do
            local offsetCompensation = math.max(-math.tan(box.rx) * box.size[3], 0)
            box.y = math.max(getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, box.x, 300, box.z), box.y) + offsetCompensation
            overlapBox(box.x, box.y, box.z, box.rx, box.ry, 0, box.size[1], box.size[2], box.size[3], "collisionTestCallbackSplit", self, ADCollSensorSplit.maskVehicles, true, true, true, true)
        end
    end

    if not self.newHit then
        if self.sensorParameters.minDynamicLength then
            self.boxes = self:getBoxShapes(self.sensorParameters.minDynamicLength)
            for _, box in pairs(self.boxes) do
                local offsetCompensation = math.max(-math.tan(box.rx) * box.size[3], 0)
                box.y = math.max(getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, box.x, 300, box.z), box.y) + offsetCompensation
                overlapBox(box.x, box.y, box.z, box.rx, box.ry, 0, box.size[1], box.size[2], box.size[3], "collisionTestCallbackSplit", self, ADCollSensorSplit.maskObjects, true, true, true, true)
            end
        end
    end
    -- Latched AFTER the scans, not before them. See the note in ADCollSensor:onUpdate: the sensor is
    -- only actually run once per ADSensor.EXECUTION_DELAY polls, so answering with the previous
    -- scan's outcome cost a full ten frames on top of that throttle - measured at exactly ten in
    -- every phase. This is the sensor the driver is stopped on.
    self.hit = self.newHit
    self.hitName = self.newHitName
    self:setTriggered(self.hit)
    self:onDrawDebug(self.boxes)
end

function ADCollSensorSplit:collisionTestCallbackSplit(transformId)
    local unloadDriver = ADHarvestManager:getAssignedUnloader(self.vehicle)
    local collisionObject = g_currentMission.nodeToObject[transformId]

    if collisionObject == nil then
        -- let try if parent is a object
        local parent = getParent(transformId)
        if parent then
            collisionObject = g_currentMission.nodeToObject[parent]
        end
    end

    if collisionObject ~= nil then
        if collisionObject ~= self and collisionObject ~= self.vehicle and not AutoDrive:checkIsConnected(self.vehicle:getRootVehicle(), collisionObject) then
            if unloadDriver == nil or (collisionObject ~= unloadDriver and (not AutoDrive:checkIsConnected(unloadDriver:getRootVehicle(), collisionObject))) then
                self.newHit = true
                self.newHitName = collisionObject.getName ~= nil
                    and collisionObject:getName() or "unnamed vehicle"
            end
        end
    elseif self:isElementBlockingVehicle(transformId) then
        self.newHit = true
        self.newHitName = "scenery"
    end
end

function ADCollSensorSplit:buildBoxShape(x, y, z, width, height, length, vecZ, vecX)
    local vehicle = self.vehicle

    local box = {}
    box.offset = {}
    box.size = {}
    box.center = {}
    box.size[1] = width * 0.5
    box.size[2] = height * 0.5
    box.size[3] = length * 0.5
    box.offset[1] = x
    box.offset[2] = y
    box.offset[3] = z
    box.center[1] = box.offset[1] + vecZ.x * box.size[3]
    box.center[2] = box.offset[2] + box.size[2]
    box.center[3] = box.offset[3] + vecZ.z * box.size[3]

    box.topLeft = {}
    box.topLeft[1] = box.center[1] - vecX.x * box.size[1] + vecZ.x * box.size[3]
    box.topLeft[2] = box.center[2]
    box.topLeft[3] = box.center[3] - vecX.z * box.size[1] + vecZ.z * box.size[3]

    box.topRight = {}
    box.topRight[1] = box.center[1] + vecX.x * box.size[1] + vecZ.x * box.size[3]
    box.topRight[2] = box.center[2]
    box.topRight[3] = box.center[3] + vecX.z * box.size[1] + vecZ.z * box.size[3]

    box.downRight = {}
    box.downRight[1] = box.center[1] + vecX.x * box.size[1] - vecZ.x * box.size[3]
    box.downRight[2] = box.center[2]
    box.downRight[3] = box.center[3] + vecX.z * box.size[1] - vecZ.z * box.size[3]

    box.downLeft = {}
    box.downLeft[1] = box.center[1] - vecX.x * box.size[1] - vecZ.x * box.size[3]
    box.downLeft[2] = box.center[2]
    box.downLeft[3] = box.center[3] - vecX.z * box.size[1] - vecZ.z * box.size[3]

    box.dirX, box.dirY, box.dirZ =  AutoDrive.localDirectionToWorld(vehicle, 0, 0, 1)
    box.zx, box.zy, box.zz =  AutoDrive.localDirectionToWorld(vehicle, vecZ.x, 0, vecZ.z)
    
    box.ry = math.atan2(box.zx, box.zz)

    local angleOffset = 4
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    if not AutoDrive.checkIsOnField(x, y, z) and self.vehicle.ad.stateModule ~= nil and self.vehicle.ad.stateModule:isActive() then
        local heightDiff = self.vehicle.ad.drivePathModule:getApproachingHeightDiff()
        if heightDiff < 1.5 and heightDiff > -1 then
            angleOffset = 0
        end
    end
    box.rx = -MathUtil.getYRotationFromDirection(box.dirY, 1) * self.frontFactor - math.rad(angleOffset)
    box.x, box.y, box.z = AutoDrive.localToWorld(vehicle, box.center[1], box.center[2], box.center[3])

    box.topLeft.x, box.topLeft.y, box.topLeft.z = AutoDrive.localToWorld(vehicle, box.topLeft[1], box.topLeft[2], box.topLeft[3])
    box.topRight.x, box.topRight.y, box.topRight.z = AutoDrive.localToWorld(vehicle, box.topRight[1], box.topRight[2], box.topRight[3])
    box.downRight.x, box.downRight.y, box.downRight.z = AutoDrive.localToWorld(vehicle, box.downRight[1], box.downRight[2], box.downRight[3])
    box.downLeft.x, box.downLeft.y, box.downLeft.z = AutoDrive.localToWorld(vehicle, box.downLeft[1], box.downLeft[2], box.downLeft[3])

    return box
end

function ADCollSensorSplit:getBoxShapes(minLength)
    -- ADSensor.getTrainDimensions, not getVehicleDimensions(vehicle, false): that call overwrites
    -- adDimensions.width and .length from vehicle.size before returning, so it can only ever hand
    -- back the vehicle definition and the measured hull never reached this box at all - it spanned
    -- exactly vehicle.size.width in every configuration. This is the sensor the driver is actually
    -- stopped on, and it now agrees with the long one beside it. Asked here on every scan rather
    -- than cached, so unlike the long sensor it is not frozen at construction time.
    local width, length = ADSensor.getTrainDimensions(self.vehicle)

    local lookAheadDistance = math.clamp(self.vehicle.lastSpeedReal * 3600 * 15.5 / 40, minLength, 50)
    local steeringAngle = math.deg(math.abs(self.vehicle.rotatedTime))

    local vecZ = {x = math.sin(self.vehicle.rotatedTime), z = math.cos(self.vehicle.rotatedTime)}
    local vecX = {x = vecZ.z, z = -vecZ.x}

    local boxYPos = AutoDrive.getSetting("collisionHeigth", self.vehicle) or 2
    local boxHeight = 0.75

    local numberOfBoxes = 5
    local boxWidth = width / numberOfBoxes
    local boxes = {}
    local locationZ = self.location.z
    if self.position == ADSensor.POS_FRONT then
        if self.vehicle.ad and self.vehicle.ad.adDimensions and self.vehicle.ad.adDimensions.maxLengthFront and self.vehicle.ad.adDimensions.maxLengthFront > 0 then
            locationZ = self.vehicle.ad.adDimensions.maxLengthFront
        end
    end
    local firstBox = 1
    if steeringAngle > 30 then
        firstBox = 2
        numberOfBoxes = numberOfBoxes - 1
    end
    for i=firstBox, numberOfBoxes do
        local xOffset = (-width / 2) + (i - 0.5) * boxWidth
        boxes[i] = self:buildBoxShape(
            self.location.x + xOffset, boxYPos, locationZ,
            boxWidth, boxHeight, lookAheadDistance,
            vecZ, vecX
        )
    end

    return boxes
end

function ADCollSensorSplit:onDrawDebug(boxes)
    if self.drawDebug or AutoDrive.getDebugChannelIsSet(AutoDrive.DC_SENSORINFO) then
        if boxes == nil then
            return
        end
        local red = 1
        local green = 0
        local blue = 0
        local isTriggered = self:isTriggered()
        if isTriggered then
            if self.sensorType == ADSensor.TYPE_FRUIT then
                blue = 1
            end
            if self.sensorType == ADSensor.TYPE_FIELDBORDER then
                green = 1
            end
        end
        
        for _, box in pairs(boxes) do
            if isTriggered then
                DebugUtil.drawOverlapBox(box.x, box.y, box.z, box.rx, box.ry, 0, box.size[1], box.size[2], box.size[3], 1, 0, 0)
            else
                DebugUtil.drawOverlapBox(box.x, box.y, box.z, box.rx, box.ry, 0, box.size[1], box.size[2], box.size[3], 1, 1, 1)
            end
        end
    end
end