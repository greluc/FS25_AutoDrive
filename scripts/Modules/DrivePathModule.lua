ADDrivePathModule = {}

ADDrivePathModule.LOOKAHEADDISTANCE = 20
ADDrivePathModule.MAXLOOKAHEADPOINTS = 20
ADDrivePathModule.MAX_SPEED_DEVIATION = 6

-- Slowing down for a corner.
--
-- The rate a loaded train can shed speed without the load shifting and without the brakes grabbing.
-- It is what turns "there is a corner in 60 m that wants 12 km/h" into "you may be doing 45 now",
-- which is the whole point: a limit derived from the distance to the corner falls smoothly as the
-- corner approaches, where one derived from whether the corner is visible at all falls off a cliff.
ADDrivePathModule.COMFORTABLE_DECELERATION = 1.5    -- m/s^2

-- How far ahead corners are looked for. Deliberately NOT a function of current speed: a search
-- window that shrinks as the vehicle slows takes the corner out of view exactly when the braking
-- starts to work, which is a positive feedback loop with no stable point - brake, lose sight of the
-- corner, accelerate, see it again, brake. That is the stutter this replaces. Bounded instead by
-- the distance actually needed to slow down from the limit in force.
ADDrivePathModule.MAX_CORNER_SCAN_DISTANCE = 150    -- m
ADDrivePathModule.CORNER_SCAN_MARGIN = 20           -- m, so the ramp starts before it has to

-- How fast the speed limit may climb again once a corner has been passed, in km/h per second.
-- Dropping is immediate - that is safety - but snapping back to full road speed the instant the
-- last way point of a bend is behind us is the other half of the stutter.
ADDrivePathModule.SPEED_LIMIT_RISE = 12

-- What the load will tolerate sideways, and the bounds on a bend worth reacting to.
--
-- Corner speed is derived from the RADIUS of the bend, not from the angle between two way points.
-- Those are not the same thing: that angle is curvature times the local spacing, and the recorder
-- (RecordingModule.lua:186-203) deliberately shortens the spacing as the turn tightens - twelve
-- metres on a straight, a quarter of a metre above 27 degrees. The angle is therefore something the
-- recorder holds roughly constant, which makes it a measure of how the route was recorded rather
-- than of how sharp it is. Measured on a real 47,264 way point network: of 1,622 way point triples
-- describing bends under 20 m radius, the angle rule let 85 percent of them be taken above 25 km/h
-- and 68 percent above 40 - an eight metre yard turn read as 14.5 degrees and was allowed 42 km/h,
-- which is seventeen metres per second squared sideways. Radius does not have that defect.
ADDrivePathModule.CORNER_LATERAL_ACCELERATION = 3.0   -- m/s^2
-- Below this the geometry is recording noise on closely spaced points rather than a real bend; no
-- tractor and trailer turns inside three metres anyway.
ADDrivePathModule.MIN_CORNER_RADIUS = 3
-- Above this it is a straight as far as speed is concerned.
ADDrivePathModule.MAX_CORNER_RADIUS = 400
-- A route recorded at a quarter of a metre through a long bend would otherwise put several hundred
-- way points inside one scan. Generous enough that it only bites on that case, where the tight
-- geometry has already produced a low limit long before the cap is reached.
ADDrivePathModule.MAX_CORNER_SCAN_POINTS = 400
ADDrivePathModule.MAX_STEERING_ANGLE = 30
ADDrivePathModule.PAUSE_TIMEOUT = 3000
ADDrivePathModule.BLINK_TIMEOUT = 1000

function ADDrivePathModule:new(vehicle)
    local o = {}
    setmetatable(o, self)
    self.__index = self
    o.vehicle = vehicle
    o.min_distance = AutoDrive.defineMinDistanceByVehicleType(vehicle)
    o.minDistanceTimer = AutoDriveTON:new()
    -- Its own timer, not minDistanceTimer: that one is reset on every frame the vehicle is being
    -- held, which is exactly when this one has to keep counting.
    o.heldTimer = AutoDriveTON:new()
    o.waitTimer = AutoDriveTON:new()
    o.blinkTimer = AutoDriveTON:new()
    o.brakeHysteresisActive = false
    o.lastUsedWayPoint = nil
    ADDrivePathModule.reset(o)
    return o
end

function ADDrivePathModule:reset()
    if self.vehicle.spec_locomotive and self.vehicle.ad and self.vehicle.ad.trainModule then
        -- train
        self.vehicle.ad.trainModule:reset()
        return
    end
    self.turnAngle = 0
    self.isPaused = false
    self.atTarget = false
    self.wayPoints = nil
    self.currentWayPoint = 0
    self.onRoadNetwork = true
    self.minDistanceToNextWp = math.huge
    self.minDistanceTimer:timer(false, 5000, 0)
    self.waitTimer:timer(false, ADDrivePathModule.PAUSE_TIMEOUT, 0)
    self.blinkTimer:timer(false, ADDrivePathModule.BLINK_TIMEOUT, 0)
    self.vehicle.ad.stateModule:setCurrentWayPointId(-1)
    self.vehicle.ad.stateModule:setNextWayPointId(-1)
    self.isReversing = false
    self.vehicle:setTurnLightState(Lights.TURNLIGHT_OFF)
    self.distanceToTarget = math.huge
    -- followWaypoints only re-bases the limit every PERF_FRAMES_HIGH frames, so it has to start
    -- out on the road limit - a 0 here would latch the brake hysteresis until the next gate frame
    self.speedLimit = self.vehicle.ad.stateModule:getSpeedLimit()
    -- Starts level with the limit rather than at nil or zero: a route beginning mid-ramp would
    -- otherwise spend the first second creeping up to a speed it is already allowed to drive.
    self.smoothedSpeedLimit = self.speedLimit
    self.maxSpeedDiff = ADDrivePathModule.MAX_SPEED_DEVIATION
    self.lastUsedWayPoint = nil

    -- increase steering speed
    if self.vehicle.spec_aiJobVehicle ~= nil then
        self.vehicle.spec_aiJobVehicle.aiSteeringSpeed = 0.004
    end
    self.min_lookAhead = AutoDrive.getMinLookaheadByVehicleType(self.vehicle)
end

function ADDrivePathModule:setPathTo(wayPointId)
    self:reset()
    self.wayPoints = ADGraphManager:getPathTo(self.vehicle, wayPointId, self.lastUsedWayPoint)
    local destination = ADGraphManager:getMapMarkerByWayPointId(self:getLastWayPointId())
    self.vehicle.ad.stateModule:setCurrentDestination(destination)
    self:setDirtyFlag()
    self.minDistanceToNextWp = math.huge

    if self.wayPoints == nil or (self.wayPoints[2] == nil and (self.wayPoints[1] == nil or (self.wayPoints[1] ~= nil and self.wayPoints[1].id ~= wayPointId))) then
        self.vehicle.ad.isStoppingWithError = true
        Logging.devError("[AutoDrive] Encountered a problem during initialization 'setPathTo'")

        local target = self.vehicle.ad.stateModule:getFirstMarker().name
        local mapMarker = ADGraphManager:getMapMarkerByWayPointId(wayPointId)
        if mapMarker ~= nil and mapMarker.name ~= nil then
            target = mapMarker.name
        end

        AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_cannot_reach; %s", 5000, self.vehicle.ad.stateModule:getName(), target)
        self.vehicle.ad.taskModule:abortAllTasks()
        self.vehicle.ad.taskModule:addTask(StopAndDisableADTask:new(self.vehicle))
    else
        --skip first wp for a smoother start
        if self.wayPoints[2] ~= nil then
            self:setCurrentWayPointIndex(2)
        else
            self:setCurrentWayPointIndex(1)
        end

        if not self.vehicle.ad.trailerModule:isActiveAtTrigger() then
            self:setUnPaused()
        end

        self.atTarget = false
    end
    self:resetIsReversing()
end

function ADDrivePathModule:appendPathTo(startWayPointId, wayPointId)
    local appendWayPoints = ADGraphManager:getPathTo(self.vehicle, wayPointId)

    if appendWayPoints == nil or (appendWayPoints[2] == nil and (appendWayPoints[1] == nil or (appendWayPoints[1] ~= nil and appendWayPoints[1].id ~= wayPointId))) then
        self.vehicle.ad.isStoppingWithError = true
        Logging.devError("[AutoDrive] Encountered a problem during initialization 'appendPathTo'")

        local target = self.vehicle.ad.stateModule:getFirstMarker().name
        local mapMarker = ADGraphManager:getMapMarkerByWayPointId(wayPointId)
        if mapMarker ~= nil and mapMarker.name ~= nil then
            target = mapMarker.name
        end

        AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_cannot_reach; %s", 5000, self.vehicle.ad.stateModule:getName(), target)
        self.vehicle.ad.taskModule:abortAllTasks()
        self.vehicle.ad.taskModule:addTask(StopAndDisableADTask:new(self.vehicle))
    else
        --skip first wp for a smoother start
        for _, wp in ipairs(appendWayPoints) do
            table.insert(self.wayPoints, wp)
        end
    end
    self:resetIsReversing()
end

function ADDrivePathModule:setWayPoints(wayPoints)
    self:reset()
    self.wayPoints = wayPoints
    local destination = ADGraphManager:getMapMarkerByWayPointId(self:getLastWayPointId())
    self.vehicle.ad.stateModule:setCurrentDestination(destination)
    self.minDistanceToNextWp = math.huge
    self.atTarget = false
    if self.wayPoints[2] ~= nil then
        self:setCurrentWayPointIndex(2)
    else
        self:setCurrentWayPointIndex(1)
    end
    self:resetIsReversing()
    if self.wayPoints == nil or #self.wayPoints < 0 then
        self.atTarget = true
    end
    self.speedLimit = self.vehicle.ad.stateModule:getSpeedLimit()
    self.distanceToTarget = self:getDistanceToLastWaypoint(40)
end

function ADDrivePathModule:setPaused()
    self.isPaused = true
    self.waitTimer:timer(false)
end

function ADDrivePathModule:setUnPaused()
    self.isPaused = false
end

function ADDrivePathModule:setDirtyFlag()
    self.wayPointsDirtyFlag = true
end

function ADDrivePathModule:resetDirtyFlag()
    self.wayPointsDirtyFlag = false
end

function ADDrivePathModule:update(dt)
    if self.vehicle.spec_locomotive and self.vehicle.ad and self.vehicle.ad.trainModule then
        -- train new
        self.vehicle.ad.trainModule:update(dt)
        return
    end
    if self.waitTimer:timer(self.isPaused, ADDrivePathModule.PAUSE_TIMEOUT, dt) then        -- used to wait for the CP silo compacter
        self:setUnPaused()
    end
    if self.isPaused then
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        return
    end

    if self.wayPoints ~= nil and self:getCurrentWayPointIndex() <= #self.wayPoints then
        if self.isReversing then
            self.vehicle.ad.specialDrivingModule:handleReverseDriving(dt)
        else
            self:followWaypoints(dt)
            self:checkIfStuck(dt)

            if self:isCloseToWaypoint() then
                self:handleReachedWayPoint()
            end
        end

        self:checkActiveAttributesSet(dt)
    else
        --keep calling the reverse function as it is also handling the bunkersilo unload, even after reaching the target
        if self.isReversing then
            self.vehicle.ad.specialDrivingModule:handleReverseDriving(dt)
        end
    end
end

function ADDrivePathModule:getIsReversing()
    return self.isReversing
end

function ADDrivePathModule:resetIsReversing()
    self.isReversing = false
    self.vehicle.ad.specialDrivingModule:reset()
end

function ADDrivePathModule:isCloseToWaypoint()
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    if self.vehicle.getAISteeringNode ~= nil then
        x, _, z = getWorldTranslation(self.vehicle:getAISteeringNode())
    end

    local maxSkipWayPoints = 1
    local wp_ahead = self:getNextWayPoint()
    local wp_current = self:getCurrentWayPoint()
    local _, isLastForward, isLastReverse = self:checkForReverseSection()
    if isLastForward or isLastReverse then
        maxSkipWayPoints = 0
    end

    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
        if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "ADDrivePathModule:isCloseToWaypoint - start, wpIdx=%d, maxSkip=%d"
            , self:getCurrentWayPointIndex(), maxSkipWayPoints)
        end
    end

    for i = 0, maxSkipWayPoints do
        if self.wayPoints[self:getCurrentWayPointIndex() + i] ~= nil then
            local distanceToCurrentWp = MathUtil.vector2Length(x - self.wayPoints[self:getCurrentWayPointIndex() + i].x, z - self.wayPoints[self:getCurrentWayPointIndex() + i].z)
            if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
                if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
                    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "ADDrivePathModule:isCloseToWaypoint(%d/%d) distanceToCurrentWp=%.1f min_distance=%.1f"
                    , i, maxSkipWayPoints, distanceToCurrentWp, self.min_distance)
                end
            end
            if distanceToCurrentWp < self.min_distance then --and i == 0
                if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
                    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "ADDrivePathModule:isCloseToWaypoint return true")
                end
                return true
            end
            -- Check if the angle between vehicle and current wp and current wp to next wp is over 90° - then we should already make the switch
            if i == 1 and wp_current and wp_ahead then
                local angle = AutoDrive.angleBetween({x = wp_ahead.x - wp_current.x, z = wp_ahead.z - wp_current.z}, {x = wp_current.x - x, z = wp_current.z - z})
                angle = math.abs(angle)

                if angle >= 135 then
                    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
                        if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
                            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "ADDrivePathModule:isCloseToWaypoint(%d/%d) - true angle=%.1f"
                            , i, maxSkipWayPoints+1, angle)
                        end
                    end
                    return true
                else
                    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
                        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "ADDrivePathModule:isCloseToWaypoint angle < 135")
                    end
                end
            else
                if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
                    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
                        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "ADDrivePathModule:isCloseToWaypoint i %d  wp_ahead %s wp_ahead %s"
                        , i, tostring(wp_ahead), tostring(wp_current))
                    end
                end
            end
        else
            if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
                if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
                    AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "ADDrivePathModule:isCloseToWaypoint self.wayPoints[self:getCurrentWayPointIndex() + i] %s"
                    , tostring(self.wayPoints[self:getCurrentWayPointIndex() + i]))
                end
            end
        end
    end
    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "ADDrivePathModule:isCloseToWaypoint end - false")
    end
    return false
end

function ADDrivePathModule:followWaypoints(dt)
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    if self.vehicle.getAISteeringNode ~= nil then
        x, y, z = getWorldTranslation(self.vehicle:getAISteeringNode())
    end

    self.acceleration = 1
    self.distanceToLookAhead = 8

    -- speedLimit and maxSpeedDiff are members so the refinement below survives the frames on which
    -- the gate does not run - re-basing them every frame threw the corner and approach limits away
    -- again on 3 of every 4 frames
    if ((g_updateLoopIndex + self.vehicle.id) % AutoDrive.PERF_FRAMES_HIGH == 0) then
        self.maxSpeedDiff = ADDrivePathModule.MAX_SPEED_DEVIATION
        self.speedLimit = self.vehicle.ad.stateModule:getSpeedLimit()
        if AutoDrive.checkIsOnField(x, y, z) then
            self.speedLimit = self.vehicle.ad.stateModule:getFieldSpeedLimit() --math.min(self.vehicle.ad.stateModule:getFieldSpeedLimit(), self.speedLimit)
        end
        if self.wayPoints[self:getCurrentWayPointIndex() - 1] ~= nil and self:getNextWayPoint() ~= nil then
            -- The indicator window depends on this and so does the branch below, so it is set here
            -- rather than as a side effect of a scan that no longer runs.
            self.distanceToLookAhead = self:getCurrentLookAheadDistance()

            -- Pathfinder output is full of 45 degree steps that need not be crawled, so off the road
            -- network the corner speeds are relaxed. Passed in rather than applied to the result:
            -- scaling the ramp is the same as raising the braking rate, which would put the late
            -- hard brake back exactly where the ramp was meant to remove it.
            local cornerScale, cornerFloor = nil, nil
            if not self:isOnRoadNetwork() then
                cornerScale, cornerFloor = 2, 12
            end
            local cornerLimit = self:getCornerSpeedLimit(self.speedLimit, cornerScale, cornerFloor)
            -- Only when a corner actually takes speed off us. Straights are the common case and
            -- would bury the interesting frames; this way the log holds one line per braking
            -- decision, which is exactly what has to be checked against what the vehicle did.
            if cornerLimit < self.speedLimit then
                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO,
                    "ADDrivePathModule: corner brake %.1f -> %.1f km/h - corner wants %.1f km/h in %.1f m (radius %.1f m, turn %.1f deg)"
                    , self.speedLimit
                    , cornerLimit
                    , self.maxAngleSpeed or -1
                    , self.maxAngleDistance or -1
                    , self.maxAngleRadius or -1
                    , self.maxAngle or -1
                )
            end
            self.speedLimit = math.min(self.speedLimit, cornerLimit)
        end

        self.distanceToTarget = self:getDistanceToLastWaypoint(40)
        if self.distanceToTarget < self.distanceToLookAhead then
            local currentTask = self.vehicle.ad.taskModule:getActiveTask()
            local isCatchingCombine = currentTask.taskType ~= nil and self.vehicle.ad.taskModule:getActiveTask().taskType == "CatchCombinePipeTask"
            if not isCatchingCombine then
                self.speedLimit = ADDrivePathModule.approachSpeedLimit(self.speedLimit, self.distanceToTarget)
            end
        end

        if self:isOnRoadNetwork() then
            self.speedLimit = math.min(self.speedLimit, self:getSpeedLimitBySteeringAngle())
        else
            -- Let's increase the cornering speed for paths generated with the pathfinder module. There are many 45° angles in there that slow the process down otherwise.
            self.speedLimit = math.min(self.speedLimit, self:getSpeedLimitBySteeringAngle() * 1.5)
        end

        if self.vehicle.ad.trailerModule:isUnloadingToBunkerSilo() then
            -- drive through bunker silo
            self.speedLimit = math.min(self.vehicle.ad.trailerModule:getBunkerSiloSpeed(), self.speedLimit)
            self.maxSpeedDiff = 1
        else
            if self.distanceToTarget < (ADTriggerManager.getMaxBunkerSiloLength() + AutoDrive.getMaxTriggerDistance(self.vehicle)) and AutoDrive.isVehicleInBunkerSiloArea(self.vehicle) then
                -- vehicle enters drive through bunker silo
                self.speedLimit = math.min(12, self.speedLimit)
                self.maxSpeedDiff = 3
            else
                local isInRangeToLoadUnloadTarget = AutoDrive.isInRangeToLoadUnloadTarget(self.vehicle) and self.distanceToTarget <= AutoDrive.getMaxTriggerDistance(self.vehicle)
                if isInRangeToLoadUnloadTarget == true then
                    self.speedLimit = math.min(5, self.speedLimit)
                end
            end
        end
    end

    local maxAngle = 60
    if self.vehicle.maxRotation then
        if self.vehicle.maxRotation > (2 * math.pi) then
            maxAngle = self.vehicle.maxRotation
        else
            maxAngle = math.deg(self.vehicle.maxRotation)
        end
    end

    self.targetX, self.targetZ = self:getLookAheadTarget()
    local lx, lz = AutoDrive.getDriveDirection(self.vehicle, self.targetX, y, self.targetZ)
    if self.vehicle.getAISteeringNode ~= nil then
        lx, lz = AutoDrive.getDriveDirection(self.vehicle, self.targetX, y, self.targetZ, self.vehicle:getAISteeringNode())
    end

    if self.vehicle.ad.collisionDetectionModule:hasDetectedObstable(dt) then
        if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "ADDrivePathModule:followWaypoints - stopVehicle")
        end
        self.vehicle.ad.specialDrivingModule:stopVehicle((not self:isOnRoadNetwork()), lx, lz)
        self.vehicle.ad.specialDrivingModule:update(dt)
    else
        self.vehicle.ad.specialDrivingModule:releaseVehicle()

        -- The limit the vehicle is actually asked to hold. Falling is immediate, because a corner
        -- that just came into range is a reason to slow down now. Climbing is rationed, because the
        -- limit jumping back to full road speed the instant the last way point of a bend is behind
        -- us is a stamp on the throttle - and with the vehicle still in the bend, promptly followed
        -- by a stamp on the brake. Done every frame rather than in the gate above, so the ramp does
        -- not arrive in steps of its own.
        local targetLimit = self.speedLimit
        if self.smoothedSpeedLimit == nil or targetLimit <= self.smoothedSpeedLimit then
            self.smoothedSpeedLimit = targetLimit
        else
            self.smoothedSpeedLimit = math.min(targetLimit,
                self.smoothedSpeedLimit + ADDrivePathModule.SPEED_LIMIT_RISE * (dt / 1000))
        end

        local speedDiff = (self.vehicle.lastSpeedReal * 3600) - self.smoothedSpeedLimit
        -- Allow active braking if vehicle is not 'following' targetSpeed precise enough
        if speedDiff <= 0.25 then
            self.brakeHysteresisActive = false
        end
        if (speedDiff > self.maxSpeedDiff) or self.brakeHysteresisActive then
            self.brakeHysteresisActive = true
            
            self.acceleration = -math.min(0.6, speedDiff * 0.05)
        end
        
        -- if self.vehicle.getAISteeringNode ~= nil then
        --     local aix, aiy, aiz = getWorldTranslation(self.vehicle:getAISteeringNode())            
        --     ADDrawingManager:addLineTask(aix, aiy+3, aiz, self.targetX, y+3, self.targetZ, 1, 1, 0, 0)
        -- else            
        --     ADDrawingManager:addLineTask(x, y+3, z, self.targetX, y+3, self.targetZ, 1, 1, 0, 0)
        -- end
        if self.vehicle.startMotor then
            if not self.vehicle:getIsMotorStarted() and self.vehicle:getCanMotorRun() and not self.vehicle.ad.specialDrivingModule:shouldStopMotor() then
                self.vehicle:startMotor()
            end
        end
        self.vehicle.ad.trailerModule:handleTrailerReversing(false)
        AutoDrive.driveInDirection(self.vehicle, dt, maxAngle, self.acceleration, 0.8, maxAngle, true, true, lx, lz, self.smoothedSpeedLimit, 1)
        --local worldX, _, worldZ = AutoDrive.worldToLocal(self.vehicle, self.targetX, y, self.targetZ)
        --print("dt: " .. dt .. " acc: " .. self.acceleration .. " x: " .. worldX .. " z: " .. worldZ .. " speedLimit: " .. self.speedLimit)
        --AIVehicleUtil.driveToPoint(self.vehicle, dt, self.acceleration, true, true, worldX, worldZ, self.speedLimit)

        -- local tX, _, tZ = worldToLocal(self.vehicle:getAISteeringNode(), self.targetX, y, self.targetZ)
        -- AIVehicleUtil.driveToPoint(self.vehicle, dt, self.acceleration, true, true, tX, tZ, self.speedLimit)
    end
end

function ADDrivePathModule:handleReachedWayPoint()
    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "ADDrivePathModule:handleReachedWayPoint")
    end
    self.lastUsedWayPoint = self:getCurrentWayPoint()
    if self:getNextWayPoint() ~= nil then
        self:switchToNextWayPoint()
    else
        self:reachedTarget()
    end
end

function ADDrivePathModule:reachedTarget()
    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "ADDrivePathModule:reachedTarget")
    end
    self.atTarget = true
    self.wayPoints = nil
    self.currentWayPoint = 0
end

function ADDrivePathModule:isTargetReached()
    if self.vehicle.spec_locomotive and self.vehicle.ad and self.vehicle.ad.trainModule then
        -- train
        return self.vehicle.ad.trainModule:isTargetReached()
    end
    return self.atTarget
end

-- To differentiate between waypoints on the road and ones created from pathfinder
function ADDrivePathModule:isOnRoadNetwork()
    return (self.wayPoints ~= nil and self:getCurrentWayPoint() ~= nil and not self:getCurrentWayPoint().isPathFinderPoint)
end

function ADDrivePathModule:getWayPoints()
    return self.wayPoints, self:getCurrentWayPointIndex()
end

function ADDrivePathModule:getLastWayPoint()
    if self.wayPoints ~= nil then
        return self.wayPoints[#self.wayPoints]
    end
    return nil
end

function ADDrivePathModule:getLastWayPointId()
    local lastWp = self:getLastWayPoint()
    if lastWp ~= nil then
        return lastWp.id
    end
    return -1
end

function ADDrivePathModule:getCurrentLookAheadDistance()
    local totalMass = self.vehicle:getTotalMass(false)
    local massFactor = math.max(1, math.min(3, (totalMass + 20) / 30))
    local speedFactor = math.max(0.25, math.min(4, (((self.vehicle.lastSpeedReal * 3600) + 10) / 20.0)))
    if speedFactor <= 1 then
        massFactor = math.min(speedFactor, massFactor)
    end
    return math.min(ADDrivePathModule.LOOKAHEADDISTANCE * massFactor * speedFactor, 150)
end

function ADDrivePathModule:getDistanceBetweenWayPoints(indexStart, indexTarget)
    local distance = 0
    while indexStart < indexTarget do
        local wpStart = self.wayPoints[indexStart]
        local wpNext = self.wayPoints[indexStart + 1]
        distance = distance + MathUtil.vector2Length(wpStart.x - wpNext.x, wpStart.z - wpNext.z)
        indexStart = indexStart + 1
    end

    return distance
end

function ADDrivePathModule:getApproachingHeightDiff()
    local heightDiff = 0
    local maxLookAhead = 10
    local maxLookAheadDistance = 20
    local lookAhead = 1
    for i = 1, maxLookAhead do
        if self.wayPoints ~= nil and self:getCurrentWayPointIndex() ~= nil and self:getCurrentWayPoint() ~= nil and (self:getCurrentWayPointIndex() + lookAhead) <= #self.wayPoints then
            local p1 = self.wayPoints[self:getCurrentWayPointIndex()]
            local p2 = self.wayPoints[self:getCurrentWayPointIndex() + lookAhead]
            local refNodeDistance = self:getDistanceBetweenWayPoints(self:getCurrentWayPointIndex(), self:getCurrentWayPointIndex() + lookAhead)
            if refNodeDistance <= maxLookAheadDistance then
                heightDiff = heightDiff + (p2.y - p1.y)
            end
            lookAhead = lookAhead + 1
        end
    end
    return heightDiff
end

--- The old angle curve, kept because it covers the case the radius does not.
---
--- Radius assumes the way points SAMPLE a bend. Where they instead describe it - a hand placed
--- vertex, a reverse switch point, a junction between two long straights - there is no arc, and
--- ((d1 + d2) / 2) / angle reports the segment length rather than any real turning circle. A right
--- angle between two twelve metre segments comes out as a 7.6 m radius and 17 km/h where the vehicle
--- has to very nearly stop. The angle rule has the opposite blind spot: it reads a densely sampled
--- bend as gentle. Neither is right on its own, so the scan takes the lower of the two.
function ADDrivePathModule.speedForAngle(angle)
    if angle == nil or angle < 5 then
        return math.huge
    elseif angle < 50 then
        return 12 + 48 * (1 - math.clamp((angle - 5), 0, 25) / (30 - 5))
    end
    return 3
end

--- The speed a bend of this radius may be taken at, from a sideways acceleration budget:
--- v = sqrt(a_lat * R). Unlike the angle between two way points, radius does not change with how
--- densely the route happens to be recorded, so the same physical bend gives the same answer
--- whether it was driven slowly or quickly when it was laid down.
---
--- The per-vehicle cornerSpeed setting still scales the result, and is the knob for anyone who
--- wants the old, faster feel back.
---
--- No settings lookup of its own, so the scan can call it once per way point without going through
--- the settings layer several hundred times on a densely recorded route; getMaxSpeedForRadius and
--- the scan apply the setting.
function ADDrivePathModule.speedForRadius(radius)
    if radius == nil or radius >= ADDrivePathModule.MAX_CORNER_RADIUS then
        return math.huge
    end
    local effective = math.max(radius, ADDrivePathModule.MIN_CORNER_RADIUS)
    return math.sqrt(ADDrivePathModule.CORNER_LATERAL_ACCELERATION * effective) * 3.6
end

--- The speed cap on the final approach to the last way point.
---
--- A CAP, not a band. The intent - do not let the vehicle crawl the last few metres - lives entirely
--- in math.max(8, ...), which never drops below 8 on its own. Clamping between that and a matching
--- lower bound therefore only ever RAISED the limit, and it runs after the corner brake and after
--- the road and field limits, so everything they had decided was discarded over the last stretch of
--- every single route. Measured on a ninety degree turn into a yard: the corner rule wanted 3.2 km/h
--- and the vehicle took the vertex at 8.0, over eleven metres of approach - and it is a fixed point,
--- not a transient, because the look-ahead window that opens this branch grows with the very speed
--- the floor is producing. A driver whose own speed setting is below 8 - both limits go down to 2 -
--- had that setting ignored on every arrival.
---
--- Its own function so a test can drive the rule rather than restate it.
function ADDrivePathModule.approachSpeedLimit(currentLimit, distanceToTarget)
    return math.min(currentLimit, math.max(8, 2 + (distanceToTarget or 0)))
end

--- What a corner described by this radius AND this turn may be taken at: the lower of the two rules.
function ADDrivePathModule.cornerSpeedFor(radius, angle)
    return math.min(ADDrivePathModule.speedForRadius(radius), ADDrivePathModule.speedForAngle(angle))
end

function ADDrivePathModule:getMaxSpeedForRadius(radius, angle)
    return ADDrivePathModule.cornerSpeedFor(radius, angle)
        * AutoDrive.getSetting("cornerSpeed", self.vehicle)
end

--- The radius of the bend through three consecutive way points. On a circular arc the turn between
--- successive chords of length L is L/R, so R is the chord length over the turn in radians.
---
--- Returns radius, the turn in degrees, and the same turn with its sign - the sign says which way
--- the route bends, which is what the indicators are driven from.
function ADDrivePathModule:getCornerRadius(ref, current, ahead)
    if ref == nil or current == nil or ahead == nil then
        return nil
    end
    local d1 = MathUtil.vector2Length(current.x - ref.x, current.z - ref.z)
    local d2 = MathUtil.vector2Length(ahead.x - current.x, ahead.z - current.z)
    local signed = AutoDrive.angleBetween(
        {x = ahead.x - current.x, z = ahead.z - current.z},
        {x = current.x - ref.x, z = current.z - ref.z})
    if d1 <= 0 or d2 <= 0 then
        return nil, nil, signed
    end
    local angle = math.abs(signed)
    if angle <= 0 or angle >= 180 then
        return nil, nil, signed
    end
    return ((d1 + d2) * 0.5) / math.rad(angle), angle, signed
end

--- The speed we may be doing right now, given every corner that lies ahead of us.
---
--- For each corner within the scan this asks how fast we may be going here in order to arrive there
--- at the speed that corner wants, braking at a rate that does not throw the load about:
---
---     v_here = sqrt(v_corner^2 + 2 * a * distance)
---
--- and takes the tightest answer. A corner sixty metres off barely constrains anything; the same
--- corner ten metres off constrains a lot. In between the limit falls smoothly, which is what makes
--- the difference to the old behaviour - that asked only "is a corner visible" and applied the full
--- corner speed the moment one was, then dropped it again the moment the shrinking search window
--- lost sight of it.
---
--- cornerScale and cornerFloor relax the CORNER SPEED, never the ramp leading to it. That
--- distinction is the whole point: scaling the ramp's result instead is algebraically the same as
--- raising the braking rate, because doubling sqrt(v^2 + 2ad) gives sqrt(4v^2 + 8ad). Pathfinder
--- output wants the relaxation - it is full of 45 degree steps that need not be crawled - but it
--- wants to be reached at the same comfortable deceleration as everything else.
---
--- Returns math.huge when nothing ahead constrains us.
function ADDrivePathModule:getCornerSpeedLimit(currentLimit, cornerScale, cornerFloor)
    local wayPoints = self.wayPoints
    local index = self:getCurrentWayPointIndex()
    -- Before the guard, not after: the indicators read this every frame, and leaving the last
    -- accumulated turn standing when the route runs short would keep them blinking into a straight.
    self.turnAngle = 0
    if wayPoints == nil or index == nil or index < 2 or (index + 1) > #wayPoints then
        return math.huge
    end

    -- Look exactly as far as slowing down from the limit in force needs, and no further. Distance to
    -- shed speed grows with the square of it, so this is short in a yard and long on a road.
    local fromSpeed = (currentLimit or 0) / 3.6
    local scanDistance = math.min(
        (fromSpeed * fromSpeed) / (2 * ADDrivePathModule.COMFORTABLE_DECELERATION)
            + ADDrivePathModule.CORNER_SCAN_MARGIN,
        ADDrivePathModule.MAX_CORNER_SCAN_DISTANCE)

    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    local anchor = wayPoints[index]
    local baseDistance = MathUtil.vector2Length(anchor.x - x, anchor.z - z)
    local travelled = baseDistance
    local limit = math.huge
    local i = index
    -- Hoisted: the loop below runs once per way point, and a route recorded at a quarter of a metre
    -- would otherwise go through the settings layer several hundred times for one answer.
    local cornerSpeedSetting = AutoDrive.getSetting("cornerSpeed", self.vehicle) or 1
    local scanned = 0

    -- The indicators are driven from the accumulated signed turn ahead, over a window that is NOT
    -- the one used for speed: it is straight line distance from the current way point, capped at
    -- MAXLOOKAHEADPOINTS, and it grows with speed. Both windows walk the same triples in the same
    -- order, so they share this pass rather than each running their own - but each keeps its own
    -- rule for when to stop, or the indicators would start behaving like a brake.
    local turnAngleOpen = (index + 2) < #wayPoints
    local turnAngleReach = (self.distanceToLookAhead or 0) - baseDistance

    while (i + 1) <= #wayPoints and (travelled <= scanDistance or turnAngleOpen)
        and scanned < ADDrivePathModule.MAX_CORNER_SCAN_POINTS do
        scanned = scanned + 1
        local ref = wayPoints[i - 1]
        local current = wayPoints[i]
        local ahead = wayPoints[i + 1]
        if ref == nil or current == nil or ahead == nil then
            break
        end

        local radius, angle, signed = self:getCornerRadius(ref, current, ahead)

        if turnAngleOpen then
            -- Accumulate, THEN decide whether to keep going - the order the original used. Testing
            -- first drops the triple that straddles the edge of the window, and when the window is
            -- shorter than one segment it drops every triple and leaves the total at zero. That is
            -- reachable: way points twelve metres apart, the vehicle a full segment back from the
            -- one it is heading for, and crawling - at which point the indicator switched itself
            -- off in the last dozen metres of the approach to a junction.
            if signed ~= nil then
                self.turnAngle = self.turnAngle + math.clamp(signed, -90, 90)
            end
            if (i - index + 1) >= ADDrivePathModule.MAXLOOKAHEADPOINTS
                or MathUtil.vector2Length(anchor.x - ahead.x, anchor.z - ahead.z) > turnAngleReach then
                turnAngleOpen = false
            end
        end

        if radius ~= nil and travelled <= scanDistance then
            local cornerSpeed = ADDrivePathModule.cornerSpeedFor(radius, angle)
                * cornerSpeedSetting * (cornerScale or 1)
            if cornerFloor ~= nil then
                cornerSpeed = math.max(cornerSpeed, cornerFloor)
            end
            if cornerSpeed < math.huge then
                local vCorner = cornerSpeed / 3.6
                local allowed = math.sqrt(vCorner * vCorner
                    + 2 * ADDrivePathModule.COMFORTABLE_DECELERATION * math.max(travelled, 0)) * 3.6
                if allowed < limit then
                    limit = allowed
                    self.maxAngle = angle
                    self.maxAngleSpeed = cornerSpeed
                    self.maxAngleRadius = radius
                    -- Diagnostics only, and the one number that makes the ramp legible in a log:
                    -- the same corner constrains a lot at ten metres and almost nothing at sixty,
                    -- so without it the reported limit cannot be told apart from a flat cap.
                    self.maxAngleDistance = travelled
                end
            end
        end

        travelled = travelled + MathUtil.vector2Length(ahead.x - current.x, ahead.z - current.z)
        i = i + 1
    end

    return limit
end

function ADDrivePathModule:getSpeedLimitBySteeringAngle()
    local steeringAngle = math.deg(math.abs(self.vehicle.rotatedTime))

    local maxSpeed = math.huge

    local maxAngle = 60
    if self.vehicle.maxRotation then
        if self.vehicle.maxRotation > (2 * math.pi) then
            maxAngle = self.vehicle.maxRotation
        else
            maxAngle = math.deg(self.vehicle.maxRotation)
        end
    end

    -- This used to be a switch: no limit at all below 95 percent of full lock, 10 km/h above it. In
    -- a bend where the wheel sits around that mark the limit therefore alternated between the two
    -- from one gate frame to the next, and a vehicle can only answer that with alternate stamps on
    -- the brake and the throttle. It is the second half of the stutter, and the reactive half - it
    -- fires inside the corner rather than on the approach to it.
    --
    -- Now it tapers over the last fifth before full lock. The endpoints are unchanged: nothing below
    -- 80 percent, 10 km/h from 95 percent on. It is never more permissive than it was.
    local taperStart = maxAngle * 0.80
    local taperEnd = maxAngle * 0.95
    if steeringAngle >= taperEnd then
        maxSpeed = 10
    elseif steeringAngle > taperStart then
        maxSpeed = 60 - 50 * ((steeringAngle - taperStart) / (taperEnd - taperStart))
    end
    return maxSpeed
end

function ADDrivePathModule:getDistanceToLastWaypoint(maxLookAheadPar)
    local distance = math.huge
    local maxLookAhead = maxLookAheadPar
    if maxLookAhead == nil then
        maxLookAhead = 10
    end

    if self.wayPoints ~= nil and self:getCurrentWayPointIndex() ~= nil and self:getCurrentWayPoint() ~= nil and (self:getCurrentWayPointIndex() + maxLookAheadPar) >= #self.wayPoints then
        distance = 0
        local lookAhead = 1
        while self.wayPoints[self:getCurrentWayPointIndex() + lookAhead] ~= nil and lookAhead < maxLookAhead do
            local p1 = self.wayPoints[self:getCurrentWayPointIndex() + lookAhead]
            local p2 = self.wayPoints[self:getCurrentWayPointIndex() + lookAhead - 1]
            local pointDistance = MathUtil.vector2Length(p2.x - p1.x, p2.z - p1.z)
            if pointDistance ~= nil then
                distance = distance + pointDistance
            end
            lookAhead = lookAhead + 1
        end
    end

    return distance
end

function ADDrivePathModule:getNextWayPoint()
    return self.wayPoints[self:getNextWayPointIndex()]
end

function ADDrivePathModule:getNextWayPointId()
    local nWp = self:getNextWayPoint()
    if nWp ~= nil and nWp.id ~= nil then
        return nWp.id
    end
    return -1
end

function ADDrivePathModule:getNextWayPoints()
    local cId = self:getCurrentWayPointIndex()
    return self.wayPoints[cId + 1], self.wayPoints[cId + 2], self.wayPoints[cId + 3], self.wayPoints[cId + 4], self.wayPoints[cId + 5]
end

function ADDrivePathModule:setCurrentWayPointIndex(waypointId)
    self.currentWayPoint = waypointId
    self.vehicle.ad.stateModule:setCurrentWayPointId(self:getCurrentWayPointId())
    self.vehicle.ad.stateModule:setNextWayPointId(self:getNextWayPointId())
end

function ADDrivePathModule:getCurrentWayPointIndex()
    return self.currentWayPoint
end

function ADDrivePathModule:getCurrentWayPoint()
    if self.wayPoints ~= nil then
        return self.wayPoints[self:getCurrentWayPointIndex()]
    end

    return nil
end

function ADDrivePathModule:getCurrentWayPointId()
    local nWp = self:getCurrentWayPoint()
    if nWp ~= nil and nWp.id ~= nil then
        return nWp.id
    end
    return -1
end

function ADDrivePathModule:getNextWayPointIndex()
    return self:getCurrentWayPointIndex() + 1
end

function ADDrivePathModule:switchToNextWayPoint()
    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "ADDrivePathModule:switchToNextWayPoint")
    end
    self:setCurrentWayPointIndex(self:getNextWayPointIndex())
    self.minDistanceToNextWp = math.huge

    local isReverse, _, _ = self:checkForReverseSection()
    if isReverse ~= self.isReversing then
        self.isReversing = isReverse
        self.vehicle.ad.specialDrivingModule:reset()
        self.vehicle.ad.specialDrivingModule.currentWayPointIndex = self:getCurrentWayPointIndex()
    end
end

function ADDrivePathModule:getLookAheadTarget()
    --start driving to the nextWayPoint when closing in on current waypoint in order to avoid harsh steering angles and oversteering

    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    if self.vehicle.getAISteeringNode ~= nil then
        x, _, z = getWorldTranslation(self.vehicle:getAISteeringNode())
    end

    local wp_current = self:getCurrentWayPoint()
    if wp_current == nil then
        return x, z
    end

    local distanceToCurrentTarget = MathUtil.vector2Length(x - wp_current.x, z - wp_current.z)
    local lookAheadDistance = self.min_lookAhead
    local lookAheadRemaining = lookAheadDistance - distanceToCurrentTarget

    local lookAheadID = 0
    local wp_ahead = wp_current
    local distanceToNextTarget = 0

    while lookAheadRemaining > distanceToNextTarget do
        lookAheadRemaining = lookAheadRemaining - distanceToNextTarget
        lookAheadID = lookAheadID + 1

        local wp_next = self.wayPoints[self:getCurrentWayPointIndex() + lookAheadID]
        if wp_next == nil or ADGraphManager:isReverseRoad(wp_ahead, wp_next) then
            break
        end
        wp_current, wp_ahead = wp_ahead, wp_next
        distanceToNextTarget = MathUtil.vector2Length(wp_current.x - wp_ahead.x, wp_current.z - wp_ahead.z)
    end

    local targetX, targetZ = wp_current.x, wp_current.z
    if lookAheadRemaining > 0.1 and distanceToNextTarget > 0.1 then
        local length = math.min(lookAheadRemaining, distanceToNextTarget)
        local addX, addZ = MathUtil.vector2SetLength(wp_ahead.x - wp_current.x, wp_ahead.z - wp_current.z, length)
        targetX = targetX + addX
        targetZ = targetZ + addZ
    end

    if AutoDrive.isEditorModeEnabled() and AutoDrive.getDebugChannelIsSet(AutoDrive.DC_VEHICLEINFO) then
        ADDrawingManager:addLineTask(x, y+2.2, z, wp_current.x, y+2.2, wp_current.z, 1.5, 0, 0, 1)
        ADDrawingManager:addLineTask(x, y+2.3, z, wp_ahead.x, y+2.4, wp_ahead.z, 1.5, 1, 0, 0)
        ADDrawingManager:addLineTask(x, y+2.4, z, targetX, y+2.6, targetZ, 1.5, 0, 1, 0)
        ADDrawingManager:addLineTask(targetX, y+2.4, targetZ, targetX, y+5, targetZ, 1.5, 0, 1, 0)
    end
    return targetX, targetZ
end

function ADDrivePathModule:checkActiveAttributesSet(dt)
    if self.vehicle.isServer then
        self.vehicle.forceIsActive = true
        self.vehicle.spec_motorized.stopMotorOnLeave = false
        self.vehicle.spec_enterable.disableCharacterOnLeave = false

        if self.vehicle.spec_aiVehicle ~= nil and self.vehicle.spec_aiVehicle.aiTrafficCollisionTranslation ~= nil then
            self.vehicle.spec_aiVehicle.aiTrafficCollisionTranslation[2] = -1000
        end

        if ((g_updateLoopIndex + self.vehicle.id) % AutoDrive.PERF_FRAMES == 0) then
            if self.vehicle.setBeaconLightsVisibility ~= nil and AutoDrive.getSetting("useBeaconLights", self.vehicle) then
                local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
                if not AutoDrive.checkIsOnField(x, y, z) and self.vehicle:getIsMotorStarted() then
                    self.vehicle:setBeaconLightsVisibility(true)
                else
                    self.vehicle:setBeaconLightsVisibility(false)
                end
            end
            local blinkangle = AutoDrive.getSetting("blinkValue") or 0

            if blinkangle > 0 then
                if self.blinkTimer:timer(math.abs(self.turnAngle) < blinkangle, ADDrivePathModule.BLINK_TIMEOUT, dt * AutoDrive.PERF_FRAMES) then -- Rough estimate for the dt time. But should be fine
                    self.vehicle:setTurnLightState(Lights.TURNLIGHT_OFF)
                else
                    if self.turnAngle > blinkangle and self:isOnRoadNetwork() then
                        self.vehicle:setTurnLightState(Lights.TURNLIGHT_LEFT)
                    elseif self.turnAngle < - blinkangle and self:isOnRoadNetwork() then
                        self.vehicle:setTurnLightState(Lights.TURNLIGHT_RIGHT)
                    end
                end
            end
        end

    end
end

--- How long a vehicle may be held at a standstill on a route before it counts as stuck.
---
--- Generous on purpose. Waiting for traffic is legitimate and common - a junction, a vehicle
--- unloading ahead, a player in the way - and none of that should raise an error. Waiting for ever
--- is the one thing that cannot be legitimate, and it is the only case this catches.
ADDrivePathModule.MAX_HELD_TIME = 60000

--- Whether the vehicle is getting nowhere.
---
--- Two ways of getting nowhere, and this used to see only one of them. The distance rule below
--- measures progress towards the next way point, and its timer runs ONLY while the vehicle is not
--- being stopped - the else branch resets it. So the moment anything holds the vehicle, the stuck
--- clock goes to zero and stays there: a vehicle held by an obstacle could never be found stuck, no
--- matter how long it stood.
---
--- Measured in game: four vehicles nose to tail at a field exit, every one of them reporting an
--- obstacle and braking every frame, four and a half minutes of standstill, and not one stuck
--- message in the whole log. Nothing else was going to break it either - the off-route yield does
--- not run on the road network, and a make-way request only goes to a vehicle that is PARKED, which
--- none of them was.
function ADDrivePathModule:checkIfStuck(dt)
    if not self.vehicle.isServer then
        return
    end

    local held = self.vehicle.ad.specialDrivingModule:isStoppingVehicle()
    local wp = self:getCurrentWayPoint()

    if not held and wp ~= nil then
        local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
        local distanceToNextWayPoint = MathUtil.vector2Length(x - wp.x, z - wp.z)
        self.minDistanceTimer:timer(distanceToNextWayPoint >= self.minDistanceToNextWp, 8000, dt)
        self.minDistanceToNextWp = math.min(self.minDistanceToNextWp, distanceToNextWayPoint)
        if self.minDistanceTimer:done() then
            self:handleBeingStuck()
            return
        end
    else
        self.minDistanceTimer:timer(false)
    end

    if self:isHeldTooLong(held, dt) then
        self:handleBeingStuck()
    end
end

--- Whether the vehicle has been held still longer than any traffic wait should last.
---
--- Tasks that park on purpose are exempt: a driver waiting to be called is not stuck, and it may
--- stand for as long as the harvest takes. They are exactly the tasks that advertise canMakeWay,
--- which is the same property AutoDrive:requestMakeWay uses to tell a parked vehicle from a driving
--- one, so the two agree by construction rather than by a second list that could drift.
function ADDrivePathModule:isHeldTooLong(held, dt)
    local taskModule = self.vehicle.ad ~= nil and self.vehicle.ad.taskModule or nil
    local activeTask = taskModule ~= nil and taskModule.getActiveTask ~= nil and taskModule:getActiveTask() or nil
    if activeTask ~= nil and activeTask.canMakeWay == true then
        self.heldTimer:timer(false)
        return false
    end
    return self.heldTimer:timer(held == true, ADDrivePathModule.MAX_HELD_TIME, dt)
end

function ADDrivePathModule:handleBeingStuck()
    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "handleBeingStuck")
    end
    if self.vehicle.isServer then
        AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.ERROR, "$l10n_AD_Driver_of; %s $l10n_AD_got_stuck;", 5000, self.vehicle.ad.stateModule:getName())
        self.vehicle.ad.taskModule:stopAndRestartAD()
    end
end

function ADDrivePathModule:checkForReverseSection()
    -- returns [current segment is reversed], [current segment is the last forward segment], [current segment is the last reverse segment]
    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "checkForReverseSection start")
    end

    if self.wayPoints == nil or self:getCurrentWayPointIndex() < 1 or #self.wayPoints <= self:getCurrentWayPointIndex() + 1 then
        if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
            if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
                AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_PATHINFO, "ADDrivePathModule:checkForReverseSection wpIdx=%d - first or last segment"
                , self:getCurrentWayPointIndex())
            end
        end
        return self.isReversing, false, false
    end

    -- check current segment (required either way)
    local wp_ref = self.wayPoints[self:getCurrentWayPointIndex() - 1]
    local wp_current = self.wayPoints[self:getCurrentWayPointIndex() - 0]
    local wp_ahead = self.wayPoints[self:getCurrentWayPointIndex() + 1]

    local isReverse = ADGraphManager:isReverseRoad(wp_ref, wp_current)
    local aheadIsReverse = ADGraphManager:isReverseRoad(wp_current, wp_ahead)

    if self.isReversing then
        -- we're allowed to continue reversing on dual segments
        isReverse = isReverse or ADGraphManager:isDualRoad(wp_ref, wp_current)
        aheadIsReverse = aheadIsReverse or ADGraphManager:isDualRoad(wp_current, wp_ahead)
    end

    -- special case for harvester-task reversed sections
    isReverse = isReverse or Utils.getNoNil(wp_current.isReverse, false)
    isReverse = isReverse and not Utils.getNoNil(wp_current.isForward, false)
    aheadIsReverse = aheadIsReverse or Utils.getNoNil(wp_ahead.isReverse, false)
    aheadIsReverse = aheadIsReverse and not Utils.getNoNil(wp_ahead.isForward, false)

    local angle = AutoDrive.angleBetween({x = wp_ahead.x - wp_current.x, z = wp_ahead.z - wp_current.z}, {x = wp_current.x - wp_ref.x, z = wp_current.z - wp_ref.z})
    local isSteepTurn = math.abs(angle) > 100

    local isLastForwardSection = not isReverse and aheadIsReverse and isSteepTurn
    local isLastReverseSection = isReverse and not aheadIsReverse and isSteepTurn

    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
        if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_VEHICLEINFO) then
            AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO, "checkForReverseSection end - isReverse %s isLastForwardSection %s isLastReverseSection %s"
            , tostring(isReverse), tostring(isLastForwardSection), tostring(isLastReverseSection))
        end
    end
    return isReverse, isLastForwardSection, isLastReverseSection
end
