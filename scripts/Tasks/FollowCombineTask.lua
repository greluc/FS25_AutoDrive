FollowCombineTask = ADInheritsFrom(AbstractTask)
FollowCombineTask.debug = false

FollowCombineTask.STATE_CHASING = {}
FollowCombineTask.STATE_WAIT_FOR_TURN = {}
FollowCombineTask.STATE_REVERSING = {}
FollowCombineTask.STATE_REVERSING_FROM_CHOPPER = {}
FollowCombineTask.STATE_WAIT_FOR_PASS_BY = {}
FollowCombineTask.STATE_FINISHED = {}
FollowCombineTask.STATE_WAIT_BEFORE_FINISH = {}
FollowCombineTask.STATE_WAIT_FOR_COMBINE_TO_PASS_BY = {}
FollowCombineTask.STATE_GENERATE_UTURN_PATH = {}
FollowCombineTask.STATE_DRIVE_UTURN_PATH = {}

FollowCombineTask.MAX_REVERSE_DISTANCE = 20
FollowCombineTask.RETREAT_DISTANCE = 6
FollowCombineTask.MIN_COMBINE_DISTANCE = 25
FollowCombineTask.COMBINE_DISTANCE_MARGIN = 3
FollowCombineTask.MAX_REVERSE_TIME = 30000
FollowCombineTask.MAX_TURN_TIME = 60000
--- The stuck criterion is "commanded to drive but standing still", which cannot be triggered by
--- waiting for the harvester. That makes a short timeout safe - the old 60 s only existed because
--- "standing still" alone could not tell being stuck from deliberately waiting.
FollowCombineTask.MAX_STUCK_TIME = 8000
FollowCombineTask.MIN_STUCK_TIME = 3000
FollowCombineTask.STUCK_SPEED = 0.0002
FollowCombineTask.MIN_COMMANDED_SPEED = 1
-- Beyond this angle the chase point lies behind the unloader; driving to it during a turn
-- means turning into the harvester. 90 degrees is "no longer ahead of us at all".
FollowCombineTask.MAX_TURN_CHASE_ANGLE = 90
-- How long a backup request from a reversing Courseplay harvester stays valid. Long enough to
-- survive a frame or two of latency, short enough that a stale request cannot make the
-- unloader retreat from a harvester that has long since driven off.
FollowCombineTask.BACKUP_REQUEST_VALID_TIME = 3000
FollowCombineTask.WAIT_BEFORE_FINISH_TIME = 8000

function FollowCombineTask:new(vehicle, combine)
    local o = FollowCombineTask:create()
    o.vehicle = vehicle
    o.combine = combine
    o.state = FollowCombineTask.STATE_CHASING
    o.reverseStartLocation = nil
    o.angleWrongTimer = AutoDriveTON:new()
    o.waitForTurnTimer = AutoDriveTON:new()
    o.stuckTimer = AutoDriveTON:new()
    o.dischargeTimer = AutoDriveTON:new()
    o.fillingTimer = AutoDriveTON:new()
    o.lastChaseSide = -10
    o.waitForPassByTimer = AutoDriveTON:new()
    o.chaseTimer = AutoDriveTON:new()
    o.startedChasing = false
    o.reverseTimer = AutoDriveTON:new()
    o.waitTimer = AutoDriveTON:new()
    o.waitForChasePos = false
    o.stuckReactions = 0
    o.reverseDistance = FollowCombineTask.MAX_REVERSE_DISTANCE
    o.resumeChasingAfterReverse = false
    o.chasePos, o.chaseSide = vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getPipeChasePosition()
    o.angleToCombineHeading = vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getAngleToCombineHeading()
    o.angleToCombine = vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getAngleToCombine()
    o.trailers = nil
    o.combineRootVehicle = o.combine:getRootVehicle()
    o.activeUnloading = AutoDrive.getSetting("activeUnloading", o.combineRootVehicle)
    FollowCombineTask.setStateNames(o)
    return o
end

function FollowCombineTask:setUp()
    FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask setUp")
    self.lastChaseSide = self.chaseSide
    self.trailers, _ = AutoDrive.getAllUnits(self.vehicle)
    self.tractorTrainLength = AutoDrive.getTractorTrainLength(self.vehicle, true, false)
    self.activeUnloading = AutoDrive.getSetting("activeUnloading", self.combineRootVehicle)
    AutoDrive.setTrailerCoverOpen(self.vehicle, self.trailers, true)
end

function FollowCombineTask:update(dt)
    if self.combine == nil or g_currentMission.nodeToObject[self.combine.components[1].node] == nil then
        self:finished()
        return
    end

    self:updateStates(dt)

    if self.lastState ~= self.state then
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update %s -> %s", tostring(self:getStateName(self.lastState)), tostring(self:getStateName()))
        self.lastState = self.state
        self:resetAllTimers()
    end

    -- standing still is only being stuck while the driver is actually asked to move - the wait
    -- states park the vehicle on purpose and must not feed the stuck timer
    local checkForStuck = self:isCommandedToDrive() and (self.vehicle.lastSpeedReal <= self.STUCK_SPEED) and (
        self.state == FollowCombineTask.STATE_CHASING
        or self.state == FollowCombineTask.STATE_WAIT_FOR_TURN
        or self.state == FollowCombineTask.STATE_WAIT_FOR_COMBINE_TO_PASS_BY
        or self.state == FollowCombineTask.STATE_GENERATE_UTURN_PATH
        or self.state == FollowCombineTask.STATE_DRIVE_UTURN_PATH
    )

    -- inside the harvester's safety distance there is nothing to gain from waiting the full
    -- timeout - we are grinding against it, so react quickly
    local stuckTime = self.MAX_STUCK_TIME
    if self.distanceToCombine < self:getMinCombineDistance() then
        stuckTime = self.MIN_STUCK_TIME
    end

    self.stuckTimer:timer(checkForStuck, stuckTime, dt)
    if self.stuckTimer:done() then
        self:reactToBeingStuck()
    end

    -- C2: a Courseplay harvester cannot reverse out of its turn because we are behind it.
    --
    -- Courseplay looks for a Courseplay drive strategy on the blocking vehicle and asks it to back
    -- up. We have no such strategy, so before this the request reached nobody: the harvester could
    -- not reverse, we never learned we were in the way, and both sat there until something else
    -- timed out. AutoDrive:requestBackupForReversingCombine records the request, this acts on it.
    if self:hasPendingBackupRequest() then
        self:startBackupForReversingCombine()
    end

    if self.state == FollowCombineTask.STATE_CHASING then
        self.chaseTimer:timer(true, 4000, dt)

        -- C1: tell a Courseplay harvester it may let us close.
        --
        -- Without this the harvester treats its own unloader as a generic obstacle: it slows down
        -- and eventually stops for the very vehicle that came to empty it. The registration on the
        -- Courseplay side expires after about a second, so it has to be repeated while we still
        -- want to be tolerated - which is exactly the duration of the chase.
        AutoDrive:requestCourseplayProximity(self.vehicle, self.combine)

        if self.combine.ad.isChopper then
            if self.filled and self.chaseSide ~= nil and self.chaseSide ~= AutoDrive.CHASEPOS_REAR then
                --skip reversing
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_CHASING - filled chopper")
                self.state = FollowCombineTask.STATE_FINISHED -- finish immediate
                return
            elseif self.filledToUnload and self.chaseSide ~= nil and self.chaseSide == AutoDrive.CHASEPOS_REAR then
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_CHASING - filledToUnload chopper")
                local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
                self.reverseStartLocation = {x = x, y = y, z = z}
                self.state = FollowCombineTask.STATE_REVERSING -- reverse to get room from harvester
                return
            end
        elseif self.filled or (self.combine.ad.isHarvester and self.combineFillPercent <= 0.1 and (not self.activeUnloading)) then
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_CHASING - filled harvester")
            self.state = FollowCombineTask.STATE_WAIT_BEFORE_FINISH -- unload after some time to let harvester drive away
            return
        end

        local wrongChopperHeading = self.combine.ad.isChopper and (self.angleToCombineHeading > 90 and self.distanceToCombine < 30)

        if (self.angleWrongTimer.elapsedTime > 15000) or wrongChopperHeading then
            -- if stuck with harvester - try reverse
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_CHASING - stuck -> stuckTimer:%s angleWrongTimer:%s"
            , tostring(self.stuckTimer:done()), tostring(self.angleWrongTimer.elapsedTime > 15000))
            local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
            self.reverseStartLocation = {x = x, y = y, z = z}
            if self.combine.ad.isChopper then
                self.state = FollowCombineTask.STATE_REVERSING_FROM_CHOPPER
                return
            else
                self.state = FollowCombineTask.STATE_REVERSING -- reverse to get room from harvester
                return
            end
        end

        if not self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:isUnloaderOnCorrectSide() then
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_CHASING - not UnloaderOnCorrectSide -> finished")
            self:finished()
            return
        end

        if self.combine.ad.isHarvester and ((self.combineFillPercent > 90 and AutoDrive.getDistanceBetween(self.vehicle, self.combine) < self.MIN_COMBINE_DISTANCE) -- if to close -> reverse
            or AutoDrive:getIsCPTurning(self.combine))
            then
            -- Stop chasing and wait for a normal unload call while standing
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_CHASING - to close to harvester -> reverse")
            local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
            self.reverseStartLocation = {x = x, y = y, z = z}
            self.state = FollowCombineTask.STATE_REVERSING -- reverse to get room from harvester
            return
        end

        if (not self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:isUnloaderOnCorrectSide(self.chaseSide)) and (not AutoDrive.combineIsTurning(self.combine)) then
            if self.lastChaseSide ~= AutoDrive.CHASEPOS_REAR then
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_CHASING - switching chase side from side to elsewhere - let's wait for passby next")
                self.state = FollowCombineTask.STATE_WAIT_FOR_PASS_BY
                return
            end
        end

        local movingDirection = self.combine.movingDirection
        if self.combine.ad.isReverseAttached then
            movingDirection = -self.combine.movingDirection
        end
        if AutoDrive.combineIsTurning(self.combine) then
            -- harvester turns
            --print("Waiting for turn now - 1- t:" ..  tostring(AutoDrive.combineIsTurning(self.combine)) .. " anglewrongtimer: " .. tostring(self.angleWrongTimer.elapsedTime > 10000))      
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_CHASING - combineIsTurning")
            self.state = FollowCombineTask.STATE_WAIT_FOR_TURN
            return
        elseif ((self.combine.lastSpeedReal *  movingDirection) <= -0.00005) then
            self.vehicle.ad.specialDrivingModule:driveReverse(dt, self.combine.lastSpeedReal * 3600 * 1.3, 1, self.vehicle.ad.trailerModule:canBeHandledInReverse())
        else
            self:followChasePoint(dt)
        end
    elseif self.state == FollowCombineTask.STATE_WAIT_FOR_TURN then
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN")
        self.waitForTurnTimer:timer(true, self.MAX_TURN_TIME, dt)
        if self.waitForTurnTimer:done() then
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - combine turn took to long - set finished now")
            self.state = FollowCombineTask.STATE_FINISHED
            return
        end

        if AutoDrive.combineIsTurning(self.combine) then
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - combineIsTurning")
            if self.combine.ad.isHarvester and (self.distanceToCombine < ((self.vehicle.size.length + self.combine.size.length) / 2 + 10)) then
                -- harvester
                -- if combine drive reverse to turn -> reverse to keep distance
                self:reverse(dt)
            elseif self.combine.ad.isChopper and AutoDrive:getIsCPActive(self.combine) then
                -- CP chopper turn
                if self.combine.ad.isAutoAimingChopper then
                    local movingDirection = self.combine.movingDirection
                    if self.combine.ad.isReverseAttached then
                        movingDirection = -self.combine.movingDirection
                    end
                    local isdrivingReverse = ((self.combine.lastSpeedReal * movingDirection) <= -0.00051) 
                    local combineIsDriving = (self.combine.lastSpeedReal > 0.001) 

                    if isdrivingReverse then
                        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN -> self:reverse")
                        self:reverse(dt)
                    elseif (not combineIsDriving and (self:getAngleToCobine() > 45)) then
                        -- if stuck with harvester - try reverse
                        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - stuck / getAngleToCobine() > 45 -> STATE_REVERSING_FROM_CHOPPER combineIsDriving %s", tostring(combineIsDriving))
                        local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
                        self.reverseStartLocation = {x = x, y = y, z = z}
                        self.state = FollowCombineTask.STATE_REVERSING_FROM_CHOPPER -- reverse to get room from harvester
                        return
                    elseif combineIsDriving then
                        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - combineIsDriving -> stopVehicle")
                        self.vehicle.ad.specialDrivingModule:stopVehicle()
                        self.vehicle.ad.specialDrivingModule:update(dt)
                    else
                        if self:isChasePointReachableDuringTurn() then
                            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN -> followChasePoint self.chaseSide %s", tostring(self.chaseSide))
                            self:followChasePoint(dt)
                        else
                            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - chase point behind us (%.0f deg) -> stopVehicle", self:getAngleToChasePos())
                            self.vehicle.ad.specialDrivingModule:stopVehicle()
                            self.vehicle.ad.specialDrivingModule:update(dt)
                        end
                    end
                else
                    -- isFixedPipeChopper
                    FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - noMovementTimer %d", self.combine.ad.noMovementTimer.elapsedTime)
                    local dischargeState = self.combine:getDischargeState()
                    self.fillingTimer:timer(not self.combine.spec_combine.isFilling, 100, dt)
                    if self.fillingTimer:done() and self.combine.ad.noMovementTimer.elapsedTime < 5000 then
                        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - fillingTimer:done")
                        -- harvested to end of row
                        AutoDrive:holdCPCombine(self.combine)
                        self.vehicle.ad.specialDrivingModule:stopVehicle()
                        self.vehicle.ad.specialDrivingModule:update(dt)
                    else
                        if self:isChasePointReachableDuringTurn() then
                            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN -> followChasePoint no AutoAimingChopper")
                            self:followChasePoint(dt)
                        else
                            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - chase point behind us (%.0f deg) -> stopVehicle", self:getAngleToChasePos())
                            self.vehicle.ad.specialDrivingModule:stopVehicle()
                            self.vehicle.ad.specialDrivingModule:update(dt)
                        end
                    end
                    self.dischargeTimer:timer(dischargeState ~= Dischargeable.DISCHARGE_STATE_OBJECT , 500, dt)
                    if self.dischargeTimer:done() and self.fillingTimer:done() and self.combine.ad.noMovementTimer.elapsedTime < 5000 then
                        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - dischargeTimer:done")
                        local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
                        self.reverseStartLocation = {x = x, y = y, z = z}
                        self.state = FollowCombineTask.STATE_REVERSING_FROM_CHOPPER -- reverse to get room from harvester
                        return
                    end
                end
            else
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN -> stopVehicle")
                -- stop while combine is turning
                self.vehicle.ad.specialDrivingModule:stopVehicle()
                self.vehicle.ad.specialDrivingModule:update(dt)
            end
        else
            -- The harvester has stopped turning, and none of the conditions below has released us
            -- yet. Without this the whole state issued nothing at all on such a frame - no drive, no
            -- brake, and no specialDrivingModule:update, so stopAndHoldVehicle never ran and nothing
            -- whatsoever reached the vehicle. Nothing else in the frame covers it either: the task
            -- module runs the active task and monitorTasks, and monitorTasks never drives.
            --
            -- It is the normal case, not an edge one. combineIsTurning goes false as soon as the
            -- harvester has not moved for three seconds, which is exactly what a full AI or
            -- Courseplay harvester does while it waits for us. Measured on the real task: up to
            -- fourteen seconds of consecutive silent frames, ended only by the fifteen second turn
            -- timeout. The task's own stuck detector cannot notice, because the last command before
            -- the flag dropped was a stop and it only counts frames we were commanded to drive.
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - not turning, not released yet -> stopVehicle")
            self.vehicle.ad.specialDrivingModule:stopVehicle()
            self.vehicle.ad.specialDrivingModule:update(dt)
        end

        -- check if we could continue
        local combineSensors = self.combine.ad.sensors or self.combineRootVehicle.ad.sensors
        if not AutoDrive.combineIsTurning(self.combine) and 
            (
                (
                    combineSensors.frontSensorFruit:pollInfo() and 
                    (
                        self.combine.ad.isChopper -- chopper
                        or self.combine.ad.driveForwardTimer.elapsedTime > 8000 -- Harvester moves
                    ) 
                ) 
                or self.waitForTurnTimer.elapsedTime > 15000 -- turn longer than 15 sec
            ) then
            if (self.angleToCombineHeading + self.angleToCombine) < 180 and self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:isUnloaderOnCorrectSide(self.chaseSide) then
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - combine turn finished - Heading looks good - start chasing again")
                self.state = FollowCombineTask.STATE_CHASING
                return
            elseif self.angleToCombineHeading > 150 and self.angleToCombineHeading < 210 and self.distanceToCombine < 80 and self.combine.ad.isHarvester then
                -- Instead of directly trying a long way around to get behind the harvester, let's wait for him to pass us by and then U-turn
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - combine turn finished - Heading inverted - wait for passby, then U-turn")
                self.state = FollowCombineTask.STATE_WAIT_FOR_COMBINE_TO_PASS_BY
                return
            else
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_TURN - combine turn finished - Heading looks bad - stop to be able to start pathfinder")
                self.stayOnField = true
                self.state = FollowCombineTask.STATE_FINISHED
                return
            end
        end
    elseif self.state == FollowCombineTask.STATE_WAIT_FOR_PASS_BY then
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_PASS_BY")
        self.waitForPassByTimer:timer(true, 2200, dt)
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        if self.waitForPassByTimer:done() then
            if (self.angleToCombineHeading + self.angleToCombine) < 180 and self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:isUnloaderOnCorrectSide() then
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_PASS_BY - passby timer elapsed - heading looks good - chasing again")
                self.state = FollowCombineTask.STATE_CHASING
                return
            else
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_PASS_BY - passby timer elapsed - heading looks bad - set finished now")
                self.stayOnField = true
                self.state = FollowCombineTask.STATE_WAIT_BEFORE_FINISH
                return
            end
        end
    elseif self.state == FollowCombineTask.STATE_REVERSING then
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_REVERSING")
        local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
        local distanceToReverseStart = MathUtil.vector2Length(x - self.reverseStartLocation.x, z - self.reverseStartLocation.z)
        self.reverseTimer:timer(true, self.MAX_REVERSE_TIME, dt)
        local doneReversing = distanceToReverseStart > self.reverseDistance or (not self.startedChasing)
        if doneReversing or self.reverseTimer:done() then
            if self.resumeChasingAfterReverse then
                -- a short retreat is meant to free the driver, not to hand the job back to the mode
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_REVERSING - retreat done - chasing again")
                self.resumeChasingAfterReverse = false
                self.reverseDistance = self.MAX_REVERSE_DISTANCE
                self.state = FollowCombineTask.STATE_CHASING
                return
            end
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_REVERSING - done reversing - set finished")
            self.state = FollowCombineTask.STATE_WAIT_BEFORE_FINISH
            return
        else
            self:reverse(dt)
        end
    elseif self.state == FollowCombineTask.STATE_REVERSING_FROM_CHOPPER then
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_REVERSING_FROM_CHOPPER")
        local cx, _, cz = getWorldTranslation(self.combine.components[1].node)
        local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
        local distanceToReverseStart = MathUtil.vector2Length(vx - self.reverseStartLocation.x, vz - self.reverseStartLocation.z)
        local distanceToCombine = MathUtil.vector2Length(cx - vx, cz - vz)
        self.reverseTimer:timer(true, self.MAX_REVERSE_TIME, dt)
        local doneReversing = distanceToReverseStart > self.MAX_REVERSE_DISTANCE or distanceToCombine > self.MAX_REVERSE_DISTANCE
        if doneReversing or self.reverseTimer:done() then
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_REVERSING_FROM_CHOPPER - done reversing - set finished")
            if self.combine.ad.isFixedPipeChopper and AutoDrive:getIsCPActive(self.combine) then
                -- wait for CP to finish a turn maneuver before invoke pathfinder
                self.state = FollowCombineTask.STATE_WAIT_BEFORE_FINISH
                return
            else
                self.state = FollowCombineTask.STATE_FINISHED
                return
            end
        else
            if self.combine.ad.isFixedPipeChopper then
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_REVERSING_FROM_CHOPPER - not AutoAimingChopper -> holdCPCombine")
                AutoDrive:holdCPCombine(self.combine)
            else
                if self:getAngleToCobine() > 30 then
                    FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_REVERSING_FROM_CHOPPER - AngleToCobine > 30 -> holdCPCombine")
                    AutoDrive:holdCPCombine(self.combine)
                end
            end
            self:reverse(dt)
        end
    elseif self.state == FollowCombineTask.STATE_WAIT_BEFORE_FINISH then
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_BEFORE_FINISH")
        -- wait for CP to finish a turn maneuver before invoke pathfinder
        -- TODO: check if following is useful!
        local combineIsDriving = self.combine.ad.isFixedPipeChopper and AutoDrive:getIsCPActive(self.combine) and (self.combine.lastSpeedReal > 0.001)
        self.waitTimer:timer(not combineIsDriving, self.WAIT_BEFORE_FINISH_TIME, dt)
        if self.waitTimer:done() then
            self.state = FollowCombineTask.STATE_FINISHED
            return
        else
            self.vehicle.ad.specialDrivingModule:stopVehicle()
            self.vehicle.ad.specialDrivingModule:update(dt)
        end
    elseif self.state == FollowCombineTask.STATE_WAIT_FOR_COMBINE_TO_PASS_BY then
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_COMBINE_TO_PASS_BY")
        self.waitForPassByTimer:timer(true, 15000, dt)
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        if self.waitForPassByTimer:done() then
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_COMBINE_TO_PASS_BY - passby timer elapsed - heading looks bad - set finished now")
            self.stayOnField = true
            self.state = FollowCombineTask.STATE_WAIT_BEFORE_FINISH
            return
        else
            local cx, cy, cz = getWorldTranslation(self.combine.components[1].node)
            local _, _, offsetZ = AutoDrive.worldToLocal(self.vehicle, cx, cy, cz)
            if offsetZ <= -10 then
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_WAIT_FOR_COMBINE_TO_PASS_BY - combine passed us. Calculate U-turn now")
                local cx, cy, cz = getWorldTranslation(self.combine.components[1].node)
                local offsetX, _, _ = AutoDrive.worldToLocal(self.vehicle, cx, cy, cz)
                self.vehicle:generateUTurn(offsetX > 0)
                self.state = FollowCombineTask.STATE_GENERATE_UTURN_PATH
                return
            end
        end
    elseif self.state == FollowCombineTask.STATE_GENERATE_UTURN_PATH then
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_GENERATE_UTURN_PATH")
        if self.vehicle.ad.uTurn ~= nil and self.vehicle.ad.uTurn.inProgress then
            self.vehicle:generateUTurn(true)
        elseif self.vehicle.ad.uTurn ~= nil and not self.vehicle.ad.uTurn.inProgress then
            if self.vehicle.ad.uTurn.colliFound or self.vehicle.ad.uTurn.points == nil or #self.vehicle.ad.uTurn.points < 5 then
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_GENERATE_UTURN_PATH - U-Turn generation failed due to collision - set finished now")
                self.stayOnField = true
                self.state = FollowCombineTask.STATE_WAIT_BEFORE_FINISH
                return
            else
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_GENERATE_UTURN_PATH - U-Turn generation finished - passing points to drivePathModule now")
                self.vehicle.ad.drivePathModule:setWayPoints(self.vehicle.ad.uTurn.points)
                self.state = FollowCombineTask.STATE_DRIVE_UTURN_PATH
                return
            end
        end
    elseif self.state == FollowCombineTask.STATE_DRIVE_UTURN_PATH then
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_DRIVE_UTURN_PATH")
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_DRIVE_UTURN_PATH - U-Turn finished")
            if (self.angleToCombineHeading + self.angleToCombine) < 180 and self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:isUnloaderOnCorrectSide() then
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_DRIVE_UTURN_PATH - passby timer elapsed - heading looks good - chasing again")
                self.state = FollowCombineTask.STATE_CHASING
                return
            else
                FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_DRIVE_UTURN_PATH - passby timer elapsed - heading looks bad - set finished now")
                self.stayOnField = true
                self.state = FollowCombineTask.STATE_WAIT_BEFORE_FINISH
                return
            end
        else
            self.vehicle.ad.drivePathModule:update(dt)
        end
    elseif self.state == FollowCombineTask.STATE_FINISHED then
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:update STATE_FINISHED")
        self:finished()
        return
    end
end

function FollowCombineTask:startPathPlanningForCircling()
    local sideOffset = 0
    if self.chaseSide ~= nil and self.chaseSide == AutoDrive.CHASEPOS_LEFT then
        sideOffset = 8
    elseif self.chaseSide ~= nil and self.chaseSide == AutoDrive.CHASEPOS_RIGHT then
        sideOffset = -8
    end

    local targetPos = AutoDrive.createWayPointRelativeToVehicle(self.vehicle, sideOffset, 0)
    local directionX, directionY, directionZ = AutoDrive.localToWorld(self.vehicle, 0, 0, 0, self.vehicle.ad.ADRootNode)
    local direction = {x = directionX - targetPos.x, z = directionZ - targetPos.z}
    self.vehicle.ad.pathFinderModule:reset()
    self.vehicle.ad.pathFinderModule:startPathPlanningTo(targetPos, direction)
end

function FollowCombineTask:updateStates(dt)
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local cx, cy, cz = getWorldTranslation(self.combine.components[1].node)

    self.chasePos, self.chaseSide = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getPipeChasePosition()

    self.angleToCombineHeading = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getAngleToCombineHeading()
    self.angleToCombine = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getAngleToCombine()

    self.lastChaseSide = self.chaseSide

    self.distanceToCombine = MathUtil.vector2Length(x - cx, z - cz)

    if (g_updateLoopIndex  % AutoDrive.PERF_FRAMES == 0) or self.updateStatesFirst ~= true then
        self.updateStatesFirst = true

        local cmaxCapacity = 0
        local cfillLevel = 0
        cfillLevel, cmaxCapacity, _ = AutoDrive.getObjectFillLevels(self.combine)
        self.combineFillPercent = cmaxCapacity > 0 and (cfillLevel / cmaxCapacity) * 100 or 0

        local fillFreeCapacity = 0
        _, _, self.filledToUnload, fillFreeCapacity = AutoDrive.getAllFillLevels(self.trailers)
        self.filled = fillFreeCapacity <= 0.1
        
        self.activeUnloading = AutoDrive.getSetting("activeUnloading", self.combineRootVehicle)
    end
    -- Evaluated once per frame and cached: shouldWaitForChasePos advances angleWrongTimer, so a
    -- second call from followChasePoint would add dt twice and halve the timer's configured time.
    self.waitForChasePos = self:shouldWaitForChasePos(dt)
end

function FollowCombineTask:reverse(dt)
    self.vehicle.ad.specialDrivingModule:driveReverse(dt, 15, 1, self.vehicle.ad.trailerModule:canBeHandledInReverse())
end

--- True while the driver is commanded to move. Chasing and reversing go through the special
--- driving module, which reports a deliberate halt as "stopping"; the U-turn path is driven by the
--- drive path module, which publishes the speed it commands.
function FollowCombineTask:isCommandedToDrive()
    if self.vehicle.ad.specialDrivingModule:isStoppingVehicle() then
        return false
    end
    if self.state == FollowCombineTask.STATE_DRIVE_UTURN_PATH then
        return (self.vehicle.ad.drivePathModule.speedLimit or 0) > self.MIN_COMMANDED_SPEED
    end
    return true
end

--- Distance below which the driver is too close to the harvester to keep working with it. Derived
--- from the two machines and what we tow, because a fixed 25 m is met by a normal side chase on a
--- large harvester and missed by a long train on a small one.
function FollowCombineTask:getMinCombineDistance()
    -- the train length is measured once in setUp, this runs every frame
    local trainLength = math.max(self.vehicle.size.length, self.tractorTrainLength or 0)
    return (trainLength + self.combine.size.length) / 2 + self.COMBINE_DISTANCE_MARGIN
end

--- Escalating answer to being stuck: ask for a fresh chase position first, then retreat a few
--- metres and try again, and only give the job back to the mode when neither helped.
-- True while a backup request from the harvester is still fresh.
function FollowCombineTask:hasPendingBackupRequest()
    local requestedAt = self.vehicle.ad ~= nil and self.vehicle.ad.reverseForCombineRequest or nil
    if requestedAt == nil then
        return false
    end
    if (g_time - requestedAt) > FollowCombineTask.BACKUP_REQUEST_VALID_TIME then
        self.vehicle.ad.reverseForCombineRequest = nil
        return false
    end
    return true
end

-- Get out of the harvester's way, then pick the chase back up.
--
-- Deliberately the same retreat the stuck handling uses rather than a second mechanism: the
-- situation is the same one - we are too close to a harvester that needs room - and the difference
-- is only who noticed it first. Reversing states are left alone; we are already moving away.
function FollowCombineTask:startBackupForReversingCombine()
    self.vehicle.ad.reverseForCombineRequest = nil

    if self.state == FollowCombineTask.STATE_REVERSING
        or self.state == FollowCombineTask.STATE_REVERSING_FROM_CHOPPER then
        return
    end

    FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:startBackupForReversingCombine - harvester needs room, retreating from state %s"
    , tostring(self:getStateName()))

    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    self.reverseStartLocation = {x = x, y = y, z = z}
    self.reverseDistance = self.RETREAT_DISTANCE
    self.resumeChasingAfterReverse = true

    if self.combine.ad.isChopper then
        self.state = FollowCombineTask.STATE_REVERSING_FROM_CHOPPER
    else
        self.state = FollowCombineTask.STATE_REVERSING
    end
end

function FollowCombineTask:reactToBeingStuck()
    self.stuckTimer:timer(false)
    self.stuckReactions = self.stuckReactions + 1
    FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:reactToBeingStuck - reaction %d in state %s"
    , self.stuckReactions, tostring(self:getStateName()))

    if self.stuckReactions == 1 and self.state == FollowCombineTask.STATE_CHASING then
        -- the chase position may simply have gone stale - the pipe folded, the side switched
        self.chasePos, self.chaseSide = self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:getPipeChasePosition()
        self.lastChaseSide = self.chaseSide
        self.angleWrongTimer:timer(false)
        self.chaseTimer:timer(false)
        return
    end

    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    self.reverseStartLocation = {x = x, y = y, z = z}

    if self.combine.ad.isChopper then
        self.state = FollowCombineTask.STATE_REVERSING_FROM_CHOPPER
        return
    end

    self.resumeChasingAfterReverse = self.stuckReactions < 3
    if self.resumeChasingAfterReverse then
        self.reverseDistance = self.RETREAT_DISTANCE
    else
        self.reverseDistance = self.MAX_REVERSE_DISTANCE
    end
    self.state = FollowCombineTask.STATE_REVERSING -- reverse to get room from harvester
end

-- Whether driving towards the chase point is safe while the harvester is turning.
--
-- During a turn the harvester swings around and drags its chase position with it. For a rear chase
-- the position ends up BEHIND the unloader - measured in game at a steady 177 degrees - so driving
-- "towards" it means turning into the harvester that is swinging back at the same time. That is
-- the "unloader drives into the harvester from behind while it turns" case.
--
-- waitForChasePos does not cover this: it is only recomputed in the chasing state, so during the
-- turn it keeps whatever value it had when the turn started.
function FollowCombineTask:isChasePointReachableDuringTurn()
    return self:getAngleToChasePos() <= FollowCombineTask.MAX_TURN_CHASE_ANGLE
end

function FollowCombineTask:followChasePoint(dt)
    if self.waitForChasePos then
        FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:followChasePoint getAngleToChasePos %.0f -> stopVehicle", self:getAngleToChasePos())
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
    else
        self.startedChasing = true
        local combineSpeed = self.combine.lastSpeedReal * 3600
        local acc = 1
        local totalSpeedLimit = 40
        -- Let's start driving a little slower when we are switching sides
        if not self.chaseTimer:done() or not self:isCaughtCurrentChaseSide() then
            acc = 1
            totalSpeedLimit = math.max(combineSpeed + 20, 10)
        end
        self.vehicle.ad.specialDrivingModule:driveToPoint(dt, self.chasePos, combineSpeed, false, acc, totalSpeedLimit)
    end
end

function FollowCombineTask:shouldWaitForChasePos(dt)
    local angle = self:getAngleToChasePos()
    self.angleWrongTimer:timer(angle > 50, 3000, dt)
    local _, _, diffZ = AutoDrive.worldToLocal(self.vehicle, self.chasePos.x, self.chasePos.y, self.chasePos.z)
    return self.angleWrongTimer:done() or  diffZ <= -1 --or (not self.combine.ad.sensors.frontSensorFruit:pollInfo())
end

function FollowCombineTask:isCaughtCurrentChaseSide()
    local caught = false
    local angle = self:getAngleToChasePos()
    local vehicleX, vehicleY, vehicleZ = getWorldTranslation(self.vehicle.components[1].node)
    local _, _, diffZ = AutoDrive.worldToLocal(self.vehicle, self.chasePos.x, self.chasePos.y, self.chasePos.z)

    local diffX, _, _ = AutoDrive.worldToLocal(self.combine, vehicleX, vehicleY, vehicleZ, self.combine.ad.ADRootNode)
    if ((angle < 15 and diffZ >= 0) or (angle > 165 and diffZ < 0)) and (self.angleToCombineHeading < 15) and (AutoDrive.sign(diffX) == self.chaseSide or self.chaseSide == AutoDrive.CHASEPOS_REAR) then
        caught = true
    end
    return caught
end


function FollowCombineTask:getAngleToCombineHeading()
    if self.vehicle == nil or self.combine == nil then
        return math.huge
    end

    local combineRx, _, combineRz = AutoDrive.localDirectionToWorld(self.combine, 0, 0, 1, self.combine.ad.ADRootNode)
    local rx, _, rz =  AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)

    return math.abs(AutoDrive.angleBetween({x = rx, z = rz}, {x = combineRx, z = combineRz}))
end

function FollowCombineTask:getAngleToChasePos()
    local worldX, _, worldZ = getWorldTranslation(self.vehicle.components[1].node)
    local rx, _, rz =  AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local angle = math.abs(AutoDrive.angleBetween({x = rx, z = rz}, {x = self.chasePos.x - worldX, z = self.chasePos.z - worldZ}))
    return angle
end

function FollowCombineTask:getAngleToCobine()
    local worldX, _, worldZ = getWorldTranslation(self.vehicle.components[1].node)
    local rx, _, rz =  AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local referenceAxis = self.combine.components[1].node
    if self.combine.components[2] ~= nil and self.combine.components[2].node ~= nil then
        referenceAxis = self.combine.components[2].node
    end

    local combineX, _, combineZ = getWorldTranslation(referenceAxis)
    local angle = math.abs(AutoDrive.angleBetween({x = rx, z = rz}, {x = combineX - worldX, z = combineZ - worldZ}))
    return angle
end

function FollowCombineTask:abort()
end

function FollowCombineTask:finished()
    FollowCombineTask.debugMsg(self.vehicle, "FollowCombineTask:finished()")
    self.vehicle.ad.taskModule:setCurrentTaskFinished()
end

function FollowCombineTask:getExcludedVehiclesForCollisionCheck()
    local excludedVehicles = {}
    if self.state == FollowCombineTask.STATE_CHASING or self.state == FollowCombineTask.STATE_WAIT_FOR_TURN then
        table.insert(excludedVehicles, self.combine:getRootVehicle())
    end
    return excludedVehicles
end

function FollowCombineTask:setStateNames()
    if self.statesToNames == nil then
        self.statesToNames = {}
        for name, id in pairs(FollowCombineTask) do
            if string.sub(name, 1, 6) == "STATE_" then
                self.statesToNames[id] = name
            end
        end
    end
end

function FollowCombineTask:getStateName(state)
    local requestedState = state
    if requestedState == nil then
        requestedState = self.state
    end
    if requestedState == nil then
        Logging.error("[AD] FollowCombineTask: Could not find name for state ->%s<- !", tostring(requestedState))
    end
    return self.statesToNames[requestedState] or ""
end

function FollowCombineTask:resetAllTimers()
    -- self.stuckTimer:timer(false) -- stuckTimer reset by speed changes
    self.angleWrongTimer:timer(false)
    self.waitForTurnTimer:timer(false)
    self.dischargeTimer:timer(false)
    self.fillingTimer:timer(false)
    self.waitForPassByTimer:timer(false)
    self.chaseTimer:timer(false)
    self.reverseTimer:timer(false)
    self.waitTimer:timer(false)
end


function FollowCombineTask:getI18nInfo()
    local text = "$l10n_AD_task_chasing_combine;"
    if self.state == FollowCombineTask.STATE_CHASING then
        if not self:isCaughtCurrentChaseSide() then
            text = text .. " - " .. "$l10n_AD_task_catching_chase_side;" .. ": "
        else
            text = text .. " - " .. "$l10n_AD_task_chase_side;" .. ": "
        end
        if self.chaseSide == AutoDrive.CHASEPOS_LEFT then
            text = text .. " - " .. "$l10n_AD_task_chase_side_left;"
        elseif self.chaseSide == AutoDrive.CHASEPOS_REAR then
            text = text .. " - " .. "$l10n_AD_task_chase_side_rear;"
        elseif self.chaseSide == AutoDrive.CHASEPOS_RIGHT then
            text = text .. " - " .. "$l10n_AD_task_chase_side_right;"
        end
    elseif self.state == FollowCombineTask.STATE_REVERSING_FROM_CHOPPER or self.state == FollowCombineTask.STATE_WAIT_FOR_TURN then
        text = text .. " - " .. "$l10n_AD_task_wait_for_combine_turn;"
    elseif self.state == FollowCombineTask.STATE_REVERSING then
        text = text .. " - " .. "$l10n_AD_task_reversing_from_combine;"
    elseif self.state == FollowCombineTask.STATE_WAIT_FOR_PASS_BY then
        text = text .. " - " .. "$l10n_AD_task_wait_for_combine_pass_by;"
    elseif self.state == FollowCombineTask.STATE_WAIT_BEFORE_FINISH then
        text = text .. " - " .. "$l10n_AD_task_waiting_for_room;"
    end
    return text
end

function FollowCombineTask.debugMsg(vehicle, debugText, ...)
    if FollowCombineTask.debug == true then
        AutoDrive.debugMsg(vehicle, debugText, ...)
    else
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_COMBINEINFO, debugText, ...)
    end
end
