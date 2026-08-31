ExitFieldTask = ADInheritsFrom(AbstractTask)
ExitFieldTask.debug = false

ExitFieldTask.STATE_PATHPLANNING = 1
ExitFieldTask.STATE_DRIVING = 2
ExitFieldTask.STATE_DELAY_PATHPLANNING = 3

ExitFieldTask.STRATEGY_START = 0
ExitFieldTask.STRATEGY_BEHIND_START = 1
ExitFieldTask.STRATEGY_CLOSEST = 2

--- mustPlan: leave the field by PLANNING a way out, not by assuming the network is reachable.
---
--- Set by the caller when the vehicle has just been recovered from a standstill. In that case being
--- near a way point is no evidence at all that the vehicle can drive, and the shortcut below - which
--- finishes the task on the spot when the network is within a driver radius - hands it straight back
--- to the follower that drove it into the obstacle in the first place.
function ExitFieldTask:new(vehicle, mustPlan)
    local o = ExitFieldTask:create()
    o.vehicle = vehicle
    o.trailers = nil
    o.failedPathFinder = 0
    o.mustPlan = mustPlan == true
    o.waitForCheckTimer = AutoDriveTON:new()
    return o
end

function ExitFieldTask:setUp()
    self.state = ExitFieldTask.STATE_DELAY_PATHPLANNING
    self.nextExitStrategy = AutoDrive.getSetting("exitField", self.vehicle)
    self.trailers, _ = AutoDrive.getAllUnits(self.vehicle)
    AutoDrive.setTrailerCoverOpen(self.vehicle, self.trailers, false)
end

function ExitFieldTask:update(dt)
    if self.state == ExitFieldTask.STATE_PATHPLANNING then
        if self.vehicle.ad.pathFinderModule:hasFinished() then
            self.wayPoints = self.vehicle.ad.pathFinderModule:getPath()
            if self.wayPoints == nil or #self.wayPoints == 0 then
                self.failedPathFinder = self.failedPathFinder + 1
                if self.failedPathFinder > 5 then
                    self.failedPathFinder = 0
                    self.vehicle.ad.modes[AutoDrive.MODE_UNLOAD]:notifyAboutFailedPathfinder()
                    self:selectNextStrategy()
                elseif self.vehicle.ad.pathFinderModule:isTargetBlocked() then
                    -- If the selected field exit isn't reachable, try the next strategy and restart without delay
                    self:startPathPlanning()
                elseif self.vehicle.ad.pathFinderModule:timedOut() or self.vehicle.ad.pathFinderModule:isBlocked() then
                    -- Add some delay to give the situation some room to clear itself
                    self:startPathPlanning()
                    self.vehicle.ad.pathFinderModule:addDelayTimer(10000)
                else
                    self:startPathPlanning()
                    self.vehicle.ad.pathFinderModule:addDelayTimer(10000)
                end
            else
                self.vehicle.ad.drivePathModule:setWayPoints(self.wayPoints)
                self.state = ExitFieldTask.STATE_DRIVING
            end
        else
            self.vehicle.ad.pathFinderModule:update(dt)
            self.vehicle.ad.specialDrivingModule:stopVehicle()
            self.vehicle.ad.specialDrivingModule:update(dt)
        end
    elseif self.state == ExitFieldTask.STATE_DELAY_PATHPLANNING then
        ExitFieldTask.debugMsg(self.vehicle, "ExitFieldTask:update - STATE_DELAY_PATHPLANNING")
        if self.waitForCheckTimer:timer(true, 1000, dt) then
            if self:startPathPlanning() then
                self.vehicle.ad.pathFinderModule:addDelayTimer(6000)
                self.state = ExitFieldTask.STATE_PATHPLANNING
                return
            end
        end
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
        return
    elseif self.state == ExitFieldTask.STATE_DRIVING then
        if self.vehicle.ad.drivePathModule:isTargetReached() then
            self.state = ExitFieldTask.STATE_FINISHED
        else
            self.vehicle.ad.drivePathModule:update(dt)
        end
    elseif self.state == ExitFieldTask.STATE_FINISHED then
        self:finished()
    end
end

function ExitFieldTask:abort()
end

function ExitFieldTask:finished()
    self.vehicle.ad.taskModule:setCurrentTaskFinished()
end

function ExitFieldTask:startPathPlanning()
    ExitFieldTask.debugMsg(self.vehicle, "ExitFieldTask:startPathPlanning")
    local closest, closestDistance = self.vehicle:getClosestWayPoint()
    if self.nextExitStrategy == ExitFieldTask.STRATEGY_CLOSEST then
        local closestNode = ADGraphManager:getWayPointById(closest)
        local wayPoints = ADGraphManager:pathFromTo(closest, self.vehicle.ad.stateModule:getSecondWayPoint())
        if wayPoints ~= nil and #wayPoints > 1 then
            -- Where to rejoin the network.
            --
            -- The CLOSEST way point is the wrong answer for a vehicle that has just been freed from
            -- standing behind something, and it is the answer that made the recovery pointless.
            -- Observed in game, three times in one session: a full unloader reversed away from the
            -- machine in front of it, then rejoined at the nearest way point - which is on the very
            -- field course it had just been queued on - and drove back into the same queue. Reversing
            -- achieves nothing if the way back in is where you came from.
            --
            -- So aim further along the route instead, and let the pathfinder find its own way to
            -- there. Four way points on is the same distance the exitField == 1 setting uses in the
            -- branch below, so this borrows a number the mod already lives with rather than inventing
            -- one. A short route keeps the old target, because there is nowhere further to aim.
            local targetNode = closestNode
            local nextNode = wayPoints[2]
            if self.mustPlan and #wayPoints > 6 then
                targetNode = wayPoints[5]
                nextNode = wayPoints[6]
            end

            -- Far enough away that the search will run at all.
            --
            -- setupNew refuses a target under MIN_TARGET_CELLS grid cells and returns
            -- completelyBlocked before its first expansion, so aiming nearer is not a hard search,
            -- it is a guaranteed failure - and this task answers failure by planning again.
            -- Measured, and the reason this exists: 527 searches by one vehicle, every one of them
            -- with the identical start and target one cell apart, every one refused before it began,
            -- zero paths found, for the whole time the player watched.
            --
            -- Four way points on was a guess at "far enough". This is the actual threshold, taken
            -- from the pathfinder's own precondition rather than borrowed from a setting.
            if self.mustPlan then
                local minDistance = AutoDrive.minPlannableDistance(self.vehicle)
                local vx, _, vz = getWorldTranslation(self.vehicle.components[1].node)
                local reachable = nil
                for index = 1, #wayPoints - 1 do
                    local wayPoint = wayPoints[index]
                    local dx, dz = wayPoint.x - vx, wayPoint.z - vz
                    if (dx * dx + dz * dz) >= (minDistance * minDistance) then
                        reachable = index
                        break
                    end
                end
                if reachable == nil then
                    -- The whole way out is shorter than the pathfinder can plan. There is nothing to
                    -- plan, and nothing to retry: drive it.
                    ExitFieldTask.debugMsg(self.vehicle, "ExitFieldTask:startPathPlanning - route shorter than the pathfinder can plan - driving it")
                    self:finished()
                    return false
                end
                targetNode = wayPoints[reachable]
                nextNode = wayPoints[reachable + 1]
            end

            -- "Already close enough" is true for a vehicle that can drive, and false for one that
            -- has just proved it cannot. Measured: both unloaders reported zero pathfinder runs in a
            -- whole session - every single ExitFieldTask ended here, on the shortcut.
            if closestDistance > AutoDrive.getDriverRadius(self.vehicle) or self.mustPlan then
                -- initiate pathFinder only if distance to closest wayPoint is enought to find a path
                local vecToNextPoint = {x = nextNode.x - targetNode.x, z = nextNode.z - targetNode.z}
                self.vehicle.ad.pathFinderModule:reset()
                self.vehicle.ad.pathFinderModule:startPathPlanningTo(targetNode, vecToNextPoint)
                return true
            else
                -- close to network, set task finished
                self:finished()
            end
        else
            AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.WARN, "$l10n_AD_Driver_of; %s $l10n_AD_cannot_find_path;", 5000, self.vehicle.ad.stateModule:getName())
            self.vehicle.ad.taskModule:abortAllTasks()
            self.vehicle.ad.taskModule:addTask(StopAndDisableADTask:new(self.vehicle))
        end
    else
        local targetNode = ADGraphManager:getWayPointById(self.vehicle.ad.stateModule:getFirstWayPoint())
        local wayPoints = ADGraphManager:pathFromTo(self.vehicle.ad.stateModule:getFirstWayPoint(), self.vehicle.ad.stateModule:getSecondWayPoint())
        if wayPoints ~= nil and #wayPoints > 1 then
            local vecToNextPoint = {x = wayPoints[2].x - targetNode.x, z = wayPoints[2].z - targetNode.z}
            if AutoDrive.getSetting("exitField", self.vehicle) == 1 and #wayPoints > 6 then
                targetNode = wayPoints[5]
                vecToNextPoint = {x = wayPoints[6].x - targetNode.x, z = wayPoints[6].z - targetNode.z}
            end
            self.vehicle.ad.pathFinderModule:reset()
            self.vehicle.ad.pathFinderModule:startPathPlanningTo(targetNode, vecToNextPoint)
            return true
        else
            AutoDriveMessageEvent.sendMessageOrNotification(self.vehicle, ADMessagesManager.messageTypes.WARN, "$l10n_AD_Driver_of; %s $l10n_AD_cannot_find_path;", 5000, self.vehicle.ad.stateModule:getName())
            self.vehicle.ad.taskModule:abortAllTasks()
            self.vehicle.ad.taskModule:addTask(StopAndDisableADTask:new(self.vehicle))
        end
    end
    return false
end

function ExitFieldTask:selectNextStrategy()
    self.nextExitStrategy = (self.nextExitStrategy + 1) % (ExitFieldTask.STRATEGY_CLOSEST + 1)
end

function ExitFieldTask:continue()
end

function ExitFieldTask:getInfoText()
    if self.state == ExitFieldTask.STATE_PATHPLANNING then
        local actualState, maxStates = self.vehicle.ad.pathFinderModule:getCurrentState()
        return g_i18n:getText("AD_task_pathfinding") .. string.format(" %d / %d ", actualState, maxStates)
    else
        return g_i18n:getText("AD_task_exiting_field")
    end
end

function ExitFieldTask:getI18nInfo()
    if self.state == ExitFieldTask.STATE_PATHPLANNING then
        local actualState, maxStates, steps, max_pathfinder_steps = self.vehicle.ad.pathFinderModule:getCurrentState()
        return "$l10n_AD_task_pathfinding;" .. string.format(" %d / %d - %d / %d", actualState, maxStates, steps, max_pathfinder_steps)
    else
        return "$l10n_AD_task_exiting_field;"
    end
end

function ExitFieldTask.debugMsg(vehicle, debugText, ...)
    if ExitFieldTask.debug == true then
        AutoDrive.debugMsg(vehicle, debugText, ...)
    else
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_COMBINEINFO, debugText, ...)
    end
end
