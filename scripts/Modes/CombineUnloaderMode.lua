CombineUnloaderMode = ADInheritsFrom(AbstractMode)
CombineUnloaderMode.debug = false

CombineUnloaderMode.STATE_INIT = {}
CombineUnloaderMode.STATE_WAIT_TO_BE_CALLED = {}
CombineUnloaderMode.STATE_DRIVE_TO_COMBINE = {}
CombineUnloaderMode.STATE_DRIVE_TO_PIPE = {}
CombineUnloaderMode.STATE_LEAVE_CROP = {}
CombineUnloaderMode.STATE_DRIVE_TO_START = {}
CombineUnloaderMode.STATE_DRIVE_TO_UNLOAD = {}
CombineUnloaderMode.STATE_FOLLOW_COMBINE = {}
CombineUnloaderMode.STATE_ACTIVE_UNLOAD_COMBINE = {}
CombineUnloaderMode.STATE_FOLLOW_CURRENT_UNLOADER = {}
CombineUnloaderMode.STATE_EXIT_FIELD = {}
CombineUnloaderMode.STATE_REVERSE_FROM_BAD_LOCATION = {}

CombineUnloaderMode.MAX_COMBINE_FILLLEVEL_CHASING = 101
CombineUnloaderMode.STATIC_X_OFFSET_FROM_HEADER = 0

-- Lateral clearance the mod keeps on top of the player's pipeOffset setting, derived from the width
-- of the whole train. See CombineUnloaderMode.calculatePipeSafetyMargin.
CombineUnloaderMode.PIPE_SAFETY_MARGIN_FACTOR = 0.15
CombineUnloaderMode.PIPE_SAFETY_MARGIN_MIN = 0.5
CombineUnloaderMode.PIPE_SAFETY_MARGIN_MAX = 1.5

-- Beyond this the straight line to a chase position says nothing useful, so only the pathfinder
-- gets to answer. See CombineUnloaderMode:isChaseSideReachable.
CombineUnloaderMode.MAX_CHASE_REACHABILITY_DISTANCE = 60

function CombineUnloaderMode:new(vehicle)
    local o = CombineUnloaderMode:create()
    o.vehicle = vehicle
    o.trailers = nil
    o.trailerCount = 0
    o.tractorTrainLength = 0
    o.tractorTrainWidth = 0
    CombineUnloaderMode.reset(o)
    CombineUnloaderMode.setStateNames(o)
    return o
end

function CombineUnloaderMode:reset()
    self.state = self.STATE_INIT
    self.activeTask = nil
    ADHarvestManager:unregisterAsUnloader(self.vehicle)
    self.combine = nil
    self.followingUnloader = nil
    self.breadCrumbs = Queue:new()
    self.lastBreadCrumb = nil
    self.failedPathFinder = 0
    self.trailers, self.trailerCount = AutoDrive.getAllUnits(self.vehicle)
    self.tractorTrainLength = AutoDrive.getTractorTrainLength(self.vehicle, true, false)
    -- 0 means "not measured yet" - the width is taken lazily because the measured dimensions of the
    -- implements may only be filled in after the train has been assembled
    self.tractorTrainWidth = 0
    self.vehicle.ad.trailerModule:reset()
    AutoDrive.getAllDischargeableUnits(self.vehicle, true) -- force initialisation
end

function CombineUnloaderMode:start(user)
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:start start self.state %s", tostring(self:getStateName()))
    if not self.vehicle.ad.stateModule:isActive() then
        self.vehicle:startAutoDrive()
    end

    if self.vehicle.ad.stateModule:getFirstMarker() == nil or self.vehicle.ad.stateModule:getSecondMarker() == nil then
        return
    end

    self:reset()

    -- getNextTask may enqueue a task itself (setToWaitForCall) and return nil. Overwriting
    -- self.activeTask with that nil would lose the task that is actually running.
    local nextTask = self:getNextTask()
    if nextTask ~= nil then
        self.activeTask = nextTask
        self.vehicle.ad.taskModule:addTask(nextTask)
    end
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:start end self.state %s", tostring(self:getStateName()))
end

function CombineUnloaderMode:monitorTasks(dt)
    if self.combine ~= nil and (self.state == self.STATE_DRIVE_TO_START or self.state == self.STATE_DRIVE_TO_UNLOAD or self.state == self.STATE_EXIT_FIELD) then
        if AutoDrive.getDistanceBetween(self.vehicle, self.combine) > 25 then
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:monitorTasks -> unregisterAsUnloader")
            ADHarvestManager:unregisterAsUnloader(self.vehicle)
            self.followingUnloader = nil
            self.combine = nil
        end
    end
    if self.combine ~= nil and self.state == self.STATE_ACTIVE_UNLOAD_COMBINE then
        self:leaveBreadCrumbs()
    end
    --We are stuck
    if self.failedPathFinder >= 5 or ((self.vehicle.ad.specialDrivingModule:shouldStopMotor() or self.vehicle.ad.specialDrivingModule.stoppedTimer:done()) and self.vehicle.ad.specialDrivingModule.isBlocked) then
        -- Before backing out: if what we are stuck behind is one of our own vehicles parked with
        -- nothing to do, asking it to move is both cheaper and likelier to help than reversing and
        -- approaching again from somewhere else - the parked one would still be there.
        --
        -- Once per stuck episode. A vehicle stuck on a tree must still reach the reverse manoeuvre,
        -- and asking again every frame would keep resetting its own stuck detection forever.
        if not self.askedNearbyVehiclesForWay and AutoDrive:askNearbyVehiclesToMakeWay(self.vehicle) > 0 then
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:monitorTasks() - stuck, asked parked vehicles to make way")
            self.askedNearbyVehiclesForWay = true
            -- restarts the stuck detection, giving the asked vehicles time to actually move
            self.vehicle.ad.specialDrivingModule:releaseVehicle()
            self.failedPathFinder = 0
        else
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:monitorTasks() - detected stuck vehicle - try reversing out of it now")
            self.vehicle.ad.specialDrivingModule:releaseVehicle()
            self.vehicle.ad.taskModule:abortAllTasks()
            self.activeTask = ReverseFromBadLocationTask:new(self.vehicle)
            self.state = self.STATE_REVERSE_FROM_BAD_LOCATION
            self.vehicle.ad.taskModule:addTask(self.activeTask)
            self.failedPathFinder = 0
            self.askedNearbyVehiclesForWay = false
        end
    end

    -- Moving again means the episode is over and the next one may ask afresh.
    if self.vehicle.lastSpeedReal > 0.0013 then
        self.askedNearbyVehiclesForWay = false
    end

    if self.vehicle.lastSpeedReal > 0.0013 then
        self.failedPathFinder = 0
    end
end

function CombineUnloaderMode:notifyAboutFailedPathfinder()
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:notifyAboutFailedPathfinder self.failedPathFinder %s ADGraphManager:getDistanceFromNetwork(self.vehicle) > 20 %s", tostring(self.failedPathFinder), tostring(ADGraphManager:getDistanceFromNetwork(self.vehicle) > 20))
    --print("CombineUnloaderMode:notifyAboutFailedPathfinder() - blocked: " .. tostring(self.vehicle.ad.pathFinderModule.completelyBlocked) .. " distance: " .. ADGraphManager:getDistanceFromNetwork(self.vehicle))
    if self.vehicle.ad.pathFinderModule.completelyBlocked and ADGraphManager:getDistanceFromNetwork(self.vehicle) > 20 then
        self.failedPathFinder = self.failedPathFinder + 1
    --print("Increased Failed pathfinder count to: " .. self.failedPathFinder)
    end
end

function CombineUnloaderMode:leaveBreadCrumbs()
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local rx, _, rz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)

    if self.lastBreadCrumb == nil then
        self.lastBreadCrumb = {x = x, y = y, z = z, dirX = rx, dirZ = rz}
        self.breadCrumbs:Enqueue(self.lastBreadCrumb)
    else
        if (self.vehicle.lastSpeedReal * self.vehicle.movingDirection) > 0 then
            local _, _, diffZ = AutoDrive.worldToLocal(self.vehicle, self.lastBreadCrumb.x, self.lastBreadCrumb.y, self.lastBreadCrumb.z)
            local vec1 = {x = x - self.lastBreadCrumb.x, z = z - self.lastBreadCrumb.z}
            local angleToNewPoint = AutoDrive.angleBetween({x = self.lastBreadCrumb.dirX, z = self.lastBreadCrumb.dirZ}, vec1)
            local minDistance = 2.5
            if math.abs(angleToNewPoint) > 40 then
                minDistance = 15
            end
            if diffZ < -1 and MathUtil.vector2Length(x - self.lastBreadCrumb.x, z - self.lastBreadCrumb.z) > minDistance and math.abs(angleToNewPoint) < 90 then
                self.lastBreadCrumb = {x = x, y = y, z = z, dirX = vec1.x, dirZ = vec1.z}
                self.breadCrumbs:Enqueue(self.lastBreadCrumb)
            end
        end
    end
end

function CombineUnloaderMode:getBreadCrumbs()
    return self.breadCrumbs
end

function CombineUnloaderMode:promoteFollowingUnloader(combine)
    self.combine = combine
    if self.vehicle.ad.taskModule.activeTask ~= nil and self.vehicle.ad.taskModule.activeTask.signalPromotion ~= nil then
        self.vehicle.ad.taskModule.activeTask:signalPromotion()
    end
end

function CombineUnloaderMode:handleFinishedTask()
    -- CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:handleFinishedTask")
    self.vehicle.ad.trailerModule:reset()
    self.lastTask = self.activeTask
    self.activeTask = nil
    -- getNextTask may enqueue a task itself (setToWaitForCall) and return nil - in that case
    -- setToWaitForCall has already put the running task into self.activeTask and it must survive.
    local nextTask = self:getNextTask()
    if nextTask ~= nil then
        self.activeTask = nextTask
        self.vehicle.ad.taskModule:addTask(nextTask)
    end
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:handleFinishedTask self.state %s", tostring(self:getStateName()))
end

function CombineUnloaderMode:stop()
end

function CombineUnloaderMode:continue()
    -- AD_continue is a bindable key and reaches a driver that is not running at all. Without this
    -- the keypress queues a task on a stopped driver, which then executes on the next start.
    if self.vehicle == nil or self.vehicle.ad == nil or self.vehicle.ad.stateModule == nil or not self.vehicle.ad.stateModule:isActive() then
        return
    end

    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local point = nil
    local distanceToStart = 0
    if
        self.vehicle.ad ~= nil and ADGraphManager.getWayPointById ~= nil and self.vehicle.ad.stateModule ~= nil and self.vehicle.ad.stateModule.getFirstMarker ~= nil and self.vehicle.ad.stateModule:getFirstMarker() ~= nil and
            self.vehicle.ad.stateModule:getFirstMarker() ~= 0 and
            self.vehicle.ad.stateModule:getFirstMarker().id ~= nil
     then
        point = ADGraphManager:getWayPointById(self.vehicle.ad.stateModule:getFirstMarker().id)
        if point ~= nil then
            distanceToStart = MathUtil.vector2Length(x - point.x, z - point.z)
        end
    end

    -- STATE_DRIVE_TO_UNLOAD is reached from several places and not all of them leave a task behind
    -- that knows how to resume: setToWaitForCall enqueues one without going through
    -- handleFinishedTask, and a task aborted with DONT_PROPAGATE leaves the previous one in place.
    -- Only the task that actually implements continue may be asked to; everything else rebuilds the
    -- unload task below instead of calling a method that is not there.
    local resumed = false
    if self.state == self.STATE_DRIVE_TO_UNLOAD and self.activeTask ~= nil and type(self.activeTask.continue) == "function" then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:continue self.STATE_DRIVE_TO_UNLOAD")
        self.activeTask:continue()
        resumed = true
    end

    if not resumed then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:continue self.state" .. tostring(self:getStateName()))
        self.vehicle.ad.taskModule:abortCurrentTask()

        if AutoDrive.checkIsOnField(x, y, z) and distanceToStart > 30 then
            -- is activated on a field - use ExitFieldTask to leave field according to setting
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:continue ExitFieldTask...")
            self.activeTask = ExitFieldTask:new(self.vehicle)
            self.state = self.STATE_EXIT_FIELD
        else
            if (AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_ONLYDELIVER or AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_PICKUPANDDELIVER) and AutoDrive.getSetting("useFolders") then
                local nextTarget = ADMultipleTargetsManager:getNextTarget(self.vehicle, false)
                if nextTarget ~= nil then
                    self.vehicle.ad.stateModule:setSecondMarker(nextTarget)
                end
            end
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:continue UnloadAtDestinationTask...")
            self.activeTask = UnloadAtDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getSecondMarker().id)
            self.state = self.STATE_DRIVE_TO_UNLOAD
        end

        ADHarvestManager:unregisterAsUnloader(self.vehicle)
        self.followingUnloader = nil
        self.combine = nil
        self.vehicle.ad.taskModule:addTask(self.activeTask)
    end
end

--- Whether the harvester we are working for still exists in the world.
---
--- Same test ADHarvestManager already uses on its own list. A vehicle table survives its deletion -
--- the reference we hold keeps it alive - but its scene node does not, so anything that reaches
--- through to the engine fails.
function CombineUnloaderMode:isCombineAlive()
    local combine = self.combine
    if combine == nil or combine.components == nil or combine.components[1] == nil then
        return false
    end
    local node = combine.components[1].node
    return g_currentMission.nodeToObject[node] ~= nil and entityExists(node)
end

function CombineUnloaderMode:getNextTask()
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask start self.state %s", tostring(self:getStateName()))
    local nextTask

    -- A harvester sold or deleted mid-chase is noticed by the tasks, which end themselves - but
    -- they end by propagating, and the branches below then build the NEXT task out of the same dead
    -- vehicle. Two of them reach through to the engine, and because the active task is cleared only
    -- after this returns, the failure repeats every frame rather than once. Checking here covers
    -- every branch at their single entry point.
    if self.combine ~= nil and not self:isCombineAlive() then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - harvester is gone, standing down")
        ADHarvestManager:unregisterAsUnloader(self.vehicle)
        self.followingUnloader = nil
        self.combine = nil
        self.combineRootVehicle = nil
        -- enqueues its own task and records it in self.activeTask, which is what a nil return means
        -- to the callers of this function
        self:setToWaitForCall()
        return nil
    end

    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local point = nil
    local _, _, filledToUnload, _ = AutoDrive.getAllFillLevels(self.trailers)

    if self.state == self.STATE_INIT then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask STATE_INIT filledToUnload %s", tostring(filledToUnload))
        if filledToUnload then -- fill level above setting unload level
            if (AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_ONLYDELIVER or AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_PICKUPANDDELIVER) and AutoDrive.getSetting("useFolders") then
                local nextTarget = ADMultipleTargetsManager:getNextTarget(self.vehicle, false)
                if nextTarget ~= nil then
                    self.vehicle.ad.stateModule:setSecondMarker(nextTarget)
                end
            end

            nextTask = self:getTaskAfterUnload(filledToUnload)

            ADHarvestManager:unregisterAsUnloader(self.vehicle)
            self.followingUnloader = nil
            self.combine = nil
        else
            if not AutoDrive.checkIsOnField(x, y, z) then
                nextTask = DriveToDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getFirstMarker().id)
                self.state = self.STATE_DRIVE_TO_START
            else
                self:setToWaitForCall()
            end
        end
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_INIT end self.state %s", tostring(self:getStateName()))
    elseif self.state == self.STATE_DRIVE_TO_COMBINE then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_DRIVE_TO_COMBINE")
        -- we finished the precall to combine route
        -- check if we should wait / pull up to combines pipe
        nextTask = FollowCombineTask:new(self.vehicle, self.combine)
        self.state = self.STATE_ACTIVE_UNLOAD_COMBINE
        self.breadCrumbs = Queue:new()
        self.lastBreadCrumb = nil
    elseif self.state == self.STATE_DRIVE_TO_PIPE or self.state == self.STATE_REVERSE_FROM_BAD_LOCATION then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_DRIVE_TO_PIPE | STATE_REVERSE_FROM_BAD_LOCATION")
        --Drive to pipe can be finished when combine is emptied or when vehicle has reached 'old' pipe position and should switch to active mode
        nextTask = self:getTaskAfterUnload(filledToUnload)
    elseif self.state == self.STATE_LEAVE_CROP then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_LEAVE_CROP")
        self:setToWaitForCall()
    elseif self.state == self.STATE_DRIVE_TO_UNLOAD then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_DRIVE_TO_UNLOAD")
        nextTask = DriveToDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getFirstMarker().id)
        if (AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_ONLYDELIVER or AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_PICKUPANDDELIVER) and AutoDrive.getSetting("useFolders") then
            local nextTarget = ADMultipleTargetsManager:getNextTarget(self.vehicle, true)
            if nextTarget ~= nil then
                self.vehicle.ad.stateModule:setSecondMarker(nextTarget)
            end
        end
        self.state = self.STATE_DRIVE_TO_START
    elseif self.state == self.STATE_DRIVE_TO_START then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_DRIVE_TO_START")
        self:setToWaitForCall()
    elseif self.state == self.STATE_ACTIVE_UNLOAD_COMBINE then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_ACTIVE_UNLOAD_COMBINE")
        if filledToUnload then
            nextTask = self:getTaskAfterUnload(filledToUnload)
        else
            nextTask = HandleHarvesterTurnTask:new(self.vehicle, self.combine)
            self.state = self.STATE_DRIVE_TO_COMBINE
        end
    elseif self.state == self.STATE_FOLLOW_CURRENT_UNLOADER then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_FOLLOW_CURRENT_UNLOADER")
        if self.targetUnloader ~= nil then
            self.targetUnloader.ad.modes[AutoDrive.MODE_UNLOAD]:unregisterFollowingUnloader()
        end
        nextTask = self:getTaskAfterUnload(filledToUnload)
    elseif self.state == self.STATE_EXIT_FIELD then
        CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask - STATE_EXIT_FIELD")
        if (AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_ONLYDELIVER or AutoDrive.getSetting("rotateTargets", self.vehicle) == AutoDrive.RT_PICKUPANDDELIVER) and AutoDrive.getSetting("useFolders") then
            local nextTarget = ADMultipleTargetsManager:getNextTarget(self.vehicle, false)
            if nextTarget ~= nil then
                self.vehicle.ad.stateModule:setSecondMarker(nextTarget)
            end
        end
        nextTask = UnloadAtDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getSecondMarker().id)
        self.state = self.STATE_DRIVE_TO_UNLOAD
    end

    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getNextTask end self.state %s", tostring(self:getStateName()))
    return nextTask
end

function CombineUnloaderMode:setToWaitForCall(keepCombine)
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:setToWaitForCall start self.state %s keepCombine %s", tostring(self:getStateName()), tostring(keepCombine))
    -- We just have to wait to be wait to be called (again)
    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local point = nil
    local distanceToStart = 0
    if
        self.vehicle.ad ~= nil and ADGraphManager.getWayPointById ~= nil and self.vehicle.ad.stateModule ~= nil and self.vehicle.ad.stateModule.getFirstMarker ~= nil
        and self.vehicle.ad.stateModule:getFirstMarker() ~= nil and self.vehicle.ad.stateModule:getFirstMarker() ~= 0 and self.vehicle.ad.stateModule:getFirstMarker().id ~= nil
    then
        point = ADGraphManager:getWayPointById(self.vehicle.ad.stateModule:getFirstMarker().id)
        if point ~= nil then
            distanceToStart = MathUtil.vector2Length(x - point.x, z - point.z)
        end
    end

    local _, _, filledToUnload, _ = AutoDrive.getAllFillLevels(self.trailers)
    -- Every branch records the task it enqueues: this is reached without going through
    -- handleFinishedTask (CatchCombinePipeTask and HandleHarvesterTurnTask finish with
    -- DONT_PROPAGATE first), so self.activeTask would otherwise keep pointing at the task that
    -- just ended - or at nothing at all - while the state says a new one is running.
    if filledToUnload then
        if AutoDrive.checkIsOnField(x, y, z) and distanceToStart > 30 then
            -- is activated on a field - use ExitFieldTask to leave field according to setting
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:setToWaitForCall ExitFieldTask...")
            self.activeTask = ExitFieldTask:new(self.vehicle)
            self.vehicle.ad.taskModule:addTask(self.activeTask)
            self.state = self.STATE_EXIT_FIELD
        else
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:setToWaitForCall UnloadAtDestinationTask...")
            self.activeTask = UnloadAtDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getSecondMarker().id)
            self.vehicle.ad.taskModule:addTask(self.activeTask)
            self.state = self.STATE_DRIVE_TO_UNLOAD
        end
    else
	    self.state = self.STATE_WAIT_TO_BE_CALLED
	    self.activeTask = WaitForCallTask:new(self.vehicle)
	    self.vehicle.ad.taskModule:addTask(self.activeTask)
	    if self.combine ~= nil and self.combine.ad ~= nil and (keepCombine == nil or keepCombine ~= true) then
	        self.combine = nil
	    end
    end
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:setToWaitForCall end self.state %s self.combine %s", tostring(self:getStateName()), tostring(self.combine))
end

function CombineUnloaderMode:assignToHarvester(harvester)
    if self.state == self.STATE_WAIT_TO_BE_CALLED then

        self.vehicle.ad.taskModule:abortCurrentTask()
        self.combine = harvester
        self.combineRootVehicle = self.combine:getRootVehicle()
        AutoDrive.validateCachedPipeData(harvester)

        -- if combine has extended pipe, aim for that. Otherwise DriveToVehicle and choose from there
        if AutoDrive.isPipeOut(self.combine) then

            local cfillLevel, cfillCapacity, _, cleftCapacity = AutoDrive.getObjectFillLevels(self.combine)
            local cFillRatio = cfillCapacity > 0 and cfillLevel / cfillCapacity or 0

            if (self.combine.ad.isHarvester) 
                and (self.combine.ad.noMovementTimer.elapsedTime > 500 or cleftCapacity < 0.1 or cFillRatio > 0.945)
            then
                -- default unloading - no movement
                self.state = self.STATE_DRIVE_TO_PIPE
                self.activeTask = EmptyHarvesterTask:new(self.vehicle, self.combine)
                self.vehicle.ad.taskModule:addTask(self.activeTask)
            else
                -- Probably active unloading for choppers and moving combines
                self.state = self.STATE_DRIVE_TO_COMBINE
                self.activeTask = CatchCombinePipeTask:new(self.vehicle, self.combine)
                self.vehicle.ad.taskModule:addTask(self.activeTask)
            end
        else
            self.state = self.STATE_DRIVE_TO_COMBINE
            self.activeTask = CatchCombinePipeTask:new(self.vehicle, self.combine)
            self.vehicle.ad.taskModule:addTask(self.activeTask)
        end
    end
end

function CombineUnloaderMode:driveToUnloader(unloader)
    if self.state == self.STATE_WAIT_TO_BE_CALLED then
        self.vehicle.ad.taskModule:abortCurrentTask()
        self.activeTask = DriveToVehicleTask:new(self.vehicle, unloader)
        self.vehicle.ad.taskModule:addTask(self.activeTask)
        unloader.ad.modes[AutoDrive.MODE_UNLOAD]:registerFollowingUnloader(self.vehicle)
        self.targetUnloader = unloader
        self.state = self.STATE_FOLLOW_CURRENT_UNLOADER
    end
end

function CombineUnloaderMode:getTaskAfterUnload(filledToUnload)
    local nextTask
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getTaskAfterUnload start filledToUnload %s", tostring(filledToUnload))

    local x, y, z = getWorldTranslation(self.vehicle.components[1].node)
    local point = nil
    local distanceToStart = 0
    if
        self.vehicle.ad ~= nil and ADGraphManager.getWayPointById ~= nil and self.vehicle.ad.stateModule ~= nil and self.vehicle.ad.stateModule.getFirstMarker ~= nil and self.vehicle.ad.stateModule:getFirstMarker() ~= nil and
            self.vehicle.ad.stateModule:getFirstMarker() ~= 0 and
            self.vehicle.ad.stateModule:getFirstMarker().id ~= nil
     then
        point = ADGraphManager:getWayPointById(self.vehicle.ad.stateModule:getFirstMarker().id)
        if point ~= nil then
            distanceToStart = MathUtil.vector2Length(x - point.x, z - point.z)
        end
    end

    if filledToUnload then
        --ADHarvestManager:unregisterAsUnloader(self.vehicle)
        --self.followingUnloader = nil
        --self.combine = nil
        if AutoDrive.checkIsOnField(x, y, z) and distanceToStart > 30 then
            -- is activated on a field - use ExitFieldTask to leave field according to setting
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getTaskAfterUnload ExitFieldTask...")
            nextTask = ExitFieldTask:new(self.vehicle)
            self.state = self.STATE_EXIT_FIELD
        else
            CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getTaskAfterUnload UnloadAtDestinationTask...")
            nextTask = UnloadAtDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getSecondMarker().id)
            self.state = self.STATE_DRIVE_TO_UNLOAD
        end
    else
        -- Should we park in the field?
        if (self.combine and self.combine.ad.isChopper) or (AutoDrive.getSetting("parkInField", self.vehicle) or (self.lastTask ~= nil and self.lastTask.stayOnField)) then
            -- If we are in fruit, we should clear it
            if AutoDrive.isVehicleOrTrailerInCrop(self.vehicle, true) then
                CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getTaskAfterUnload ClearCropTask...")
                nextTask = ClearCropTask:new(self.vehicle, self.combine)
                self.state = self.STATE_LEAVE_CROP
            else
                CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getTaskAfterUnload setToWaitForCall")
                self:setToWaitForCall()
            end
        else
            ADHarvestManager:unregisterAsUnloader(self.vehicle)
            nextTask = DriveToDestinationTask:new(self.vehicle, self.vehicle.ad.stateModule:getFirstMarker().id)
            self.state = self.STATE_DRIVE_TO_START
        end
    end
    CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getTaskAfterUnload end")
    return nextTask
end

function CombineUnloaderMode:shouldLoadOnTrigger()
    return false
end

function CombineUnloaderMode:shouldUnloadAtTrigger()
    return self.state == self.STATE_DRIVE_TO_UNLOAD
end

function CombineUnloaderMode:getUnloaderOnSide()
    local vehicleX, vehicleY, vehicleZ = getWorldTranslation(self.vehicle.components[1].node)
    local diffX, _, diffZ = AutoDrive.worldToLocal(self.combine, vehicleX, vehicleY, vehicleZ, self.combine.ad.ADRootNode)
    local leftright = AutoDrive.CHASEPOS_UNKNOWN
    local frontback = AutoDrive.CHASEPOS_FRONT
    -- If we're not clearly on one side or the other of the combine, we don't
    -- give a clear answer
    if math.abs(diffX) > self.combine.size.width / 2 then
        leftright = AutoDrive.sign(diffX)
    end

    local maxZ = self.tractorTrainLength
    if diffZ < maxZ then
        frontback = AutoDrive.CHASEPOS_REAR
    end

    return leftright, frontback
end

function CombineUnloaderMode:isUnloaderOnCorrectSide(chaseSide)
    local sideIndex = chaseSide

    if sideIndex == nil and self.chasePosIndex == nil then
        return false
    elseif sideIndex == nil then
        sideIndex = self.chasePosIndex
    end

    local leftRight, frontBack = self:getUnloaderOnSide()

    if (leftRight == sideIndex and frontBack == AutoDrive.CHASEPOS_REAR) or (leftRight == AutoDrive.CHASEPOS_UNKNOWN and frontBack == AutoDrive.CHASEPOS_REAR) or frontBack == sideIndex then
        return true
    else
        return false
    end
end

function CombineUnloaderMode:getPipeSlopeCorrection2()
    local dischargeX, dichargeY, dischargeZ = getWorldTranslation(AutoDrive.getDischargeNode(self.combine))
    local diffX, diffY, _ = AutoDrive.worldToLocal(self.combine, dischargeX, dichargeY, dischargeZ, self.combine.ad.ADRootNode)
    if math.abs(diffX) < self.combine.size.width / 2 then
        -- Some pipes curl up so tight they cause a collisions.
        -- We just don't try to correct in this case.
        return 0
    end

    local heightUnderPipe = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, dischargeX, dichargeY, dischargeZ)
    -- It would be nice if we could use the discharge node direction and the terrain under the harveser,
    -- but discharge node rotations are untrustworthy.
    local wux, wuy, wuz = getTerrainNormalAtWorldPos(g_currentMission.terrainRootNode, dischargeX, dichargeY - heightUnderPipe, dischargeZ)
    local ux, uy, uz = AutoDrive.worldDirectionToLocal(self.combine, wux, wuy, wuz, self.combine.ad.ADRootNode)

    -- This is backwards from the usual order of these variables, but I need deviation from 0 and pi, not
    -- pi/2 and 3pi/4, so we adjust the coordinate system
    local theta = math.atan(ux / uy)

    local pipePosition = diffX * math.cos(theta) + diffY * math.sin(theta)
    local currentElevationCorrection = pipePosition - diffX

    if math.abs(currentElevationCorrection) > 1 then
        -- Assume something has gone very wrong if the correction gets too large.
        return 0
    end

    return currentElevationCorrection
end

function CombineUnloaderMode:getPipeSlopeCorrection()
    return self:getPipeSlopeCorrection2()
end

--- The widest unit of the whole train, not just the tractor. A chaser bin is regularly wider than
--- the vehicle pulling it, and everything that keeps us clear of the harvester is measured from the
--- centre line, so the tractor's width is the wrong number to clear the trailer with. The length
--- side of the same geometry already asks the whole train - see the use of
--- AutoDrive.getTractorTrainLength in PathFinderModule:startPathPlanningToPipe.
function CombineUnloaderMode.getTrainWidth(vehicle)
    local widest = 0
    if vehicle ~= nil then
        local units, _ = AutoDrive.getAllUnits(vehicle)
        if units ~= nil then
            for _, unit in ipairs(units) do
                local unitWidth = nil
                local dimensions = unit.ad ~= nil and unit.ad.adDimensions or nil
                if dimensions ~= nil and dimensions.maxWidthLeft ~= nil and dimensions.maxWidthRight ~= nil then
                    -- measured dimensions are half widths and they are not symmetric on every implement
                    unitWidth = dimensions.maxWidthLeft + dimensions.maxWidthRight
                elseif unit.size ~= nil and unit.size.width ~= nil then
                    unitWidth = unit.size.width
                end
                if unitWidth ~= nil then
                    widest = math.max(widest, unitWidth)
                end
            end
        end
        if widest <= 0 and vehicle.size ~= nil and vehicle.size.width ~= nil then
            -- nothing measured and no units reported - fall back to the vehicle definition
            widest = vehicle.size.width
        end
    end
    return widest
end

function CombineUnloaderMode:getTractorTrainWidth()
    if self.tractorTrainWidth == nil or self.tractorTrainWidth <= 0 then
        self.tractorTrainWidth = CombineUnloaderMode.getTrainWidth(self.vehicle)
    end
    return self.tractorTrainWidth
end

--- The clearance the mod insists on, on top of whatever the player dialled in. Wider trains need
--- more of it, but a fixed floor keeps a narrow tractor from ending up flush against the header.
function CombineUnloaderMode.calculatePipeSafetyMargin(trainWidth)
    local margin = (trainWidth or 0) * CombineUnloaderMode.PIPE_SAFETY_MARGIN_FACTOR
    return math.min(math.max(margin, CombineUnloaderMode.PIPE_SAFETY_MARGIN_MIN), CombineUnloaderMode.PIPE_SAFETY_MARGIN_MAX)
end

function CombineUnloaderMode:getPipeSafetyMargin()
    return CombineUnloaderMode.calculatePipeSafetyMargin(self:getTractorTrainWidth())
end

--- The lateral distance we keep from the harvester is the player's preference PLUS the safety
--- margin above. Keeping the margin out of the setting means a player who dials the offset to 0 - or
--- to a negative value, which the setting still allows - does not end up scraping the header.
function CombineUnloaderMode:getPipeOffset()
    return AutoDrive.getSetting("pipeOffset", self.vehicle) + self:getPipeSafetyMargin()
end

--- One-time migration for savegames written before the safety margin existed. Back then the stored
--- pipeOffset was the only clearance there was, so players folded the margin into it by hand;
--- adding the margin on top of an unchanged setting would push every unloader a margin too far out.
--- Reduce the stored preference by exactly the margin we now add ourselves, floored at 0 so a
--- previously negative offset cannot eat into the margin.
--- Must run once per vehicle after its settings have been read, and never a second time.
function CombineUnloaderMode.migratePipeOffsetForSafetyMargin(vehicle)
    if vehicle == nil or vehicle.ad == nil or vehicle.ad.pipeOffsetSafetyMarginMigrated then
        return false
    end

    -- only the vehicle's own copy may be touched: writing the shared template would migrate every
    -- other driver as a side effect
    local setting = vehicle.ad.settings ~= nil and vehicle.ad.settings.pipeOffset or nil
    if setting == nil or setting.values == nil then
        return false
    end

    local currentValue = AutoDrive.getSetting("pipeOffset", vehicle)
    if currentValue == nil then
        return false
    end

    local margin = CombineUnloaderMode.calculatePipeSafetyMargin(CombineUnloaderMode.getTrainWidth(vehicle))
    local targetValue = math.max(currentValue - margin, 0)

    -- The setting is an index into a fixed grid of values. Take the largest entry that does not
    -- exceed the target, so the migration can only ever reduce the stored preference.
    local newIndex = nil
    for index, value in ipairs(setting.values) do
        if value <= targetValue and (newIndex == nil or value > setting.values[newIndex]) then
            newIndex = index
        end
    end
    if newIndex == nil then
        return false
    end

    vehicle.ad.pipeOffsetSafetyMarginMigrated = true
    AutoDrive.setSettingState("pipeOffset", newIndex, vehicle)
    return true
end

function CombineUnloaderMode:getSideChaseOffsetX()
    -- NB: We cannot apply slope correction until after we have chosen which side
    -- we are chasing on! This function only finds the base X offset "to the left".
    -- Slope and side correction MUST be applied in CombineUnloaderMode:getPipeChasePosition
    -- AFTER determining the chase side. Or this function needs to be rewritten.
    local pipeOffset = self:getPipeOffset()
    local unloaderWidest = self:getTractorTrainWidth()
    local headerExtra = math.max((AutoDrive.getFrontToolWidth(self.combine) - self.combine.size.width) / 2, 0)

    local sideChaseTermPipeIn = self.combine.size.width / 2 + unloaderWidest / 2 + headerExtra
    local sideChaseTermPipeOut = AutoDrive.getPipeLength(self.combine)
    -- Some combines fold up their pipe so tight that targeting it could cause a collision.
    -- So, choose the max between the two to avoid a collison
    local sideChaseTermX = math.max(sideChaseTermPipeIn, sideChaseTermPipeOut)

    if self.combine.ad.isSugarcaneHarvester then
        -- check for SugarcaneHarvester has to be first, as it also is IsBufferCombine!
        sideChaseTermX = AutoDrive.getPipeLength(self.combine)
    elseif self.combine.ad.isAutoAimingChopper then
        sideChaseTermX = sideChaseTermPipeIn + CombineUnloaderMode.STATIC_X_OFFSET_FROM_HEADER
    elseif (self.combine.ad ~= nil and self.combine.ad.storedPipeLength ~= nil) or AutoDrive.isPipeOut(self.combine) then
        -- If the pipe is extended, though, target it regardless
        sideChaseTermX = sideChaseTermPipeOut
    end

    return sideChaseTermX + pipeOffset
end

function CombineUnloaderMode:getSideChaseOffsetX_new()
    -- NB: We cannot apply slope correction until after we have chosen which side
    -- we are chasing on! This function only finds the base X offset "to the left".
    -- Slope and side correction MUST be applied in CombineUnloaderMode:getPipeChasePosition
    -- AFTER determining the chase side. Or this function needs to be rewritten.
    local pipeOffset = AutoDrive.getSetting("pipeOffset", self.vehicle)
    local unloaderWidest = self.vehicle.size.width
    local headerExtra = math.max((AutoDrive.getFrontToolWidth(self.combine) - self.combine.size.width) / 2, 0)

    local sideChaseTermPipeIn = self.combine.size.width / 2 + unloaderWidest / 2 + headerExtra
    -- Some combines fold up their pipe so tight that targeting it could cause a collision.
    -- So, choose the max between the two to avoid a collison
    local sideChaseTermX = 0

    if self.combine.ad.isSugarcaneHarvester then
        -- check for SugarcaneHarvester has to be first, as it also is IsBufferCombine!
        sideChaseTermX = AutoDrive.getPipeLength(self.combine)
    elseif self.combine.ad.isAutoAimingChopper then
        sideChaseTermX = sideChaseTermPipeIn + CombineUnloaderMode.STATIC_X_OFFSET_FROM_HEADER
    elseif (self.combine.ad ~= nil and self.combine.ad.storedPipeLength ~= nil) or AutoDrive.isPipeOut(self.combine) then
        -- If the pipe is extended, though, target it regardless
        sideChaseTermX = 0
    end

    return sideChaseTermX + pipeOffset
end

function CombineUnloaderMode:getPipeChaseWayPoint(offsetX, offsetZ)
    local wayPoint = {x = 0, y = 0, z = 0}
    local trailer = g_currentMission.nodeToObject[self.targetFillNode]
    if trailer == nil then
        CombineUnloaderMode.debugMsg(self.vehicle, "ERROR: CombineUnloaderMode:getPipeChaseWayPoint trailer == nil")
        trailer = self.vehicle
    end
    local _, trailerDistY, _ = localToLocal(self.targetFillNode, trailer.components[1].node, 0, 0, 0)
    local dischargeNode = AutoDrive.getDischargeNode(self.combine)
    if dischargeNode == nil then
        CombineUnloaderMode.debugMsg(self.vehicle, "ERROR: CombineUnloaderMode:getPipeChaseWayPoint dischargeNode == nil")
        dischargeNode = self.combine.components[1].node
    end
    local combinePipeRootNode = AutoDrive.getPipeRoot(self.combine)
    if combinePipeRootNode == nil then
        CombineUnloaderMode.debugMsg(self.vehicle, "ERROR: CombineUnloaderMode:getPipeChaseWayPoint combinePipeRootNode == nil")
        combinePipeRootNode = self.combine.components[1].node
    end
    local targetX, targetY, targetZ = getWorldTranslation(dischargeNode)
    local combineX, combineY, combineZ = getWorldTranslation(combinePipeRootNode)
    local _, combineDistY, _ = localToLocal(dischargeNode, combinePipeRootNode, 0, 0, 0)
    local diffY = combineDistY - trailerDistY -- distance discharge to fill point
    local pipeSide = self.pipeSide
    if pipeSide == 0 then
        pipeSide = 1
    end
    local pipeOffset = self:getPipeOffset()
    local worldOffsetX1, worldOffsetY1, worldOffsetZ1 = AutoDrive.localDirectionToWorld(self.combine, 0, -diffY, 0, dischargeNode)
    local targetDistX, _, targetDistZ = AutoDrive.worldToLocal(self.combine, targetX + worldOffsetX1, targetY + worldOffsetY1, targetZ + worldOffsetZ1, combinePipeRootNode)
    local worldOffsetX2, worldOffsetY2, worldOffsetZ2 = AutoDrive.localDirectionToWorld(self.combine, targetDistX + (offsetX or 0) + (pipeSide * pipeOffset), 0, targetDistZ + (offsetZ or 0), combinePipeRootNode)
    wayPoint.x = combineX + worldOffsetX2
    wayPoint.y = combineY + worldOffsetY2
    wayPoint.z = combineZ + worldOffsetZ2
    return wayPoint
end

function CombineUnloaderMode:getDynamicSideChaseOffsetZ()
    local nodeX, nodeY, nodeZ = getWorldTranslation(AutoDrive.getDischargeNode(self.combine))
    local _, _, pipeZOffsetToCombine = AutoDrive.worldToLocal(self.combine, nodeX, nodeY, nodeZ, self.combine.ad.ADRootNode)
    local targetX, targetY, targetZ = getWorldTranslation(self.targetFillNode)

    local _, _, vehicleZOffsetToTarget = AutoDrive.worldToLocal(self.vehicle, targetX, targetY, targetZ)

    local sideChaseTermZ = pipeZOffsetToCombine - vehicleZOffsetToTarget

    return sideChaseTermZ
end

function CombineUnloaderMode:getDynamicSideChaseOffsetZ_fromDischargeNode(planningPhase)
    local targetX, targetY, targetZ = getWorldTranslation(self.targetFillNode)
    
    local _, _, vehicleZOffsetToTarget = AutoDrive.worldToLocal(self.vehicle, targetX, targetY, targetZ)
    local offset = -vehicleZOffsetToTarget
    if planningPhase then
        offset = offset + (self.vehicle.size.length/2)
    end
    return offset
end

function CombineUnloaderMode:getSideChaseOffsetZ(dynamic)
    if dynamic then
        return self:getDynamicSideChaseOffsetZ()
    else
        return (self.combine.size.length - self.vehicle.size.length + AutoDrive.getFrontToolLength(self.combine)) / 2
    end
end

function CombineUnloaderMode:getRearChaseOffsetX(leftBlocked, rightBlocked)
    local rearChaseOffset = (self.combine.size.width / 2 + self.vehicle.size.width / 2) + 1

    if self.combine.ad.isAutoAimingChopper and not self.combine.ad.isSugarcaneHarvester then
        return 0
    elseif rightBlocked and leftBlocked then
        return 0
    elseif leftBlocked then
        return -rearChaseOffset
    else
        return rearChaseOffset
    end
end

function CombineUnloaderMode:getRearChaseOffsetZ(planningPhase)
    local followDistance = AutoDrive.getSetting("followDistance", self.vehicle)
    local rearChaseOffset = -followDistance - (self.combine.size.length / 2)
    if self.combine.ad.isAutoAimingChopper and not self.combine.ad.isSugarcaneHarvester then
        if planningPhase then
            rearChaseOffset = - (self.combine.size.length / 2)
        end
    else
        -- math.sqrt(2) ensures the trailer could straighten if it was turned 90 degrees, and it makes this point further
        -- back than the pathfinder (straightening) target in PathFinderModule:startPathPlanningToPipe
        -- math.sqrt(2) gives the hypotenuse of an isosceles right trangle with side length equal to the length
        -- of the trailer
        if self.combine.ad.isSugarcaneHarvester then
            rearChaseOffset = -self.combine.size.length / 2 - self.tractorTrainLength * math.sqrt(2)
        else
            local combineSensors = self.combine.ad.sensors or self.combineRootVehicle.ad.sensors
            if self.combine.lastSpeedReal > 0.002 and combineSensors.frontSensorFruit:pollInfo() then
                rearChaseOffset = -10
            else
                --there is no need to be close to the rear of the harvester here. We can make it hard on the pathfinder since we have no strong desire to chase there anyway for normal harvesters
                --Especially when they are CP driven, we have to be prepared for that massive reverse maneuver when the combine is filled and wants to avoid the crop.
                rearChaseOffset = -45
            end
        end
    end

    return rearChaseOffset
end

--- Can we actually get to that chase position, or is something parked in between?
--- A straight line is not a path and is not meant to be one - the pathfinder still has the last
--- word. It is here to rule out the obviously bad side before we commit to it, because the flags
--- the chase side is otherwise chosen from come from the HARVESTER's sensors and know nothing about
--- where the unloader is standing. Only vehicles are in the mask: they are the case the harvester
--- cannot see (a second unloader, a parked tractor), and anything static in a field is the
--- pathfinder's business. Returns true whenever it cannot say otherwise, so a frame in which
--- nothing is in the way behaves exactly as it did before this check existed.
function CombineUnloaderMode:isChaseSideReachable(chasePos)
    if chasePos == nil or self.vehicle == nil or self.combine == nil then
        return true
    end

    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    local deltaX, deltaZ = chasePos.x - x, chasePos.z - z
    local distance = MathUtil.vector2Length(deltaX, deltaZ)
    if distance < 1 or distance > CombineUnloaderMode.MAX_CHASE_REACHABILITY_DISTANCE then
        -- on top of it already, or far enough away that only the pathfinder can answer
        return true
    end

    local centerX, centerZ = x + deltaX / 2, z + deltaZ / 2
    local centerY = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, centerX, 300, centerZ) + 1.5
    local angleRad = AutoDrive.normalizeAngle(math.atan2(deltaX, deltaZ))
    local halfWidth = self:getTractorTrainWidth() / 2
    local halfLength = distance / 2
    local mask = AutoDrive.collisionMaskVehicleDimesions or CollisionFlag.VEHICLE

    self.chaseSideBlocked = false
    overlapBox(centerX, centerY, centerZ, 0, angleRad, 0, halfWidth, 1.5, halfLength, "isChaseSideReachable_Callback", self, mask, true, true, true, true)

    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_COMBINEINFO) then
        local r, g, b = 0, 1, 0
        if self.chaseSideBlocked then
            r, g, b = 1, 0, 0
        end
        DebugUtil.drawOverlapBox(centerX, centerY, centerZ, 0, angleRad, 0, halfWidth, 1.5, halfLength, r, g, b)
    end

    return not self.chaseSideBlocked
end

function CombineUnloaderMode:isChaseSideReachable_Callback(transformId)
    if transformId == nil or transformId == 0 or transformId == g_currentMission.terrainRootNode then
        return
    end
    local collisionObject = g_currentMission.nodeToObject[transformId]
    if collisionObject == nil then
        -- let try if parent is a object
        local parent = getParent(transformId)
        if parent then
            collisionObject = g_currentMission.nodeToObject[parent]
        end
    end
    if collisionObject == nil then
        -- only vehicles are in the mask, so a shape that resolves to nothing is not ours to judge
        return
    end
    -- the line starts inside our own train and ends alongside the harvester, both by construction
    if collisionObject == self.vehicle or AutoDrive:checkIsConnected(self.vehicle, collisionObject) then
        return
    end
    if collisionObject == self.combine or AutoDrive:checkIsConnected(self.combineRootVehicle or self.combine, collisionObject) then
        return
    end
    self.chaseSideBlocked = true
end

function CombineUnloaderMode:getPipeChasePosition(planningPhase)
    local chaseNode
    local sideIndex = AutoDrive.CHASEPOS_REAR

    -- in case of rotated attached combine use the trailing vehicle sensors
    local combineSensors = self.combine.ad.sensors or self.combineRootVehicle.ad.sensors

    local leftBlocked = (AutoDrive.getSetting("avoidFruit", self.vehicle) and combineSensors.leftSensorFruit:pollInfo())
    or combineSensors.leftSensor:pollInfo()
    or (AutoDrive.getSetting("followOnlyOnField", self.vehicle) and (not combineSensors.leftSensorField:pollInfo()))

    local leftFrontBlocked = (AutoDrive.getSetting("avoidFruit", self.vehicle) and combineSensors.leftFrontSensorFruit:pollInfo())
    or combineSensors.leftFrontSensor:pollInfo()

    local rightBlocked = (AutoDrive.getSetting("avoidFruit", self.vehicle) and combineSensors.rightSensorFruit:pollInfo())
    or combineSensors.rightSensor:pollInfo()
    or (AutoDrive.getSetting("followOnlyOnField", self.vehicle) and (not combineSensors.rightSensorField:pollInfo()))

    local rightFrontBlocked = (AutoDrive.getSetting("avoidFruit", self.vehicle) and combineSensors.rightFrontSensorFruit:pollInfo())
    or combineSensors.rightFrontSensor:pollInfo()

    rightBlocked = rightBlocked or rightFrontBlocked
    leftBlocked = leftBlocked or leftFrontBlocked

-- TODO: does this make sense, i.e. leftBlocked = leftBlocked or leftFrontBlocked <-> elseif leftFrontBlocked and (not rightFrontBlocked) then
    -- prefer side where front is also free
    if (not leftBlocked) and (not rightBlocked) then
        if (not leftFrontBlocked) and rightFrontBlocked then
            rightBlocked = true
        elseif leftFrontBlocked and (not rightFrontBlocked) then
            leftBlocked = true
        end
    end

    self.pipeSide = AutoDrive.getPipeSide(self.combine)
    self.targetFillUnit, self.targetFillNode = AutoDrive.getNextFreeDischargeableUnit(self.vehicle)

    local sideChaseTermX = self:getSideChaseOffsetX()
    local sideChaseTermZ = self:getSideChaseOffsetZ(AutoDrive.dynamicChaseDistance or self.combine.ad.isHarvester)
    local rearChaseTermZ = self:getRearChaseOffsetZ(planningPhase)

    if self.combine.ad.isChopper then
        -- any chopper
        --CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getPipeChasePosition=IsBufferCombine")
        -- the correction reads the terrain normal under the pipe, so it is the same value for both
        -- sides within one frame - asking for it twice only paid for the raycast twice
        local pipeSlopeCorrection = self:getPipeSlopeCorrection()
        local leftChasePos = AutoDrive.createWayPointRelativeToVehicle(self.combine, sideChaseTermX + pipeSlopeCorrection, sideChaseTermZ - 2)
        local rightChasePos = AutoDrive.createWayPointRelativeToVehicle(self.combine, -(sideChaseTermX + pipeSlopeCorrection), sideChaseTermZ - 2)
        local rearChasePos = AutoDrive.createWayPointRelativeToVehicle(self.combine, 0, rearChaseTermZ)
        local angleToLeftChaseSide = self:getAngleToChasePos(leftChasePos)
        local angleToRearChaseSide = self:getAngleToChasePos(rearChasePos)
        local chaseSideSetting = AutoDrive.getSetting("chaseSide", self.combine)

        if chaseSideSetting == AutoDrive.CHASEPOS_AUTO and not self.combine.ad.isFixedPipeChopper then
            -- The flags above are the harvester's own sensors: they answer whether a side is clear
            -- for the harvester, not whether we could get to it. Ask that separately, and only where
            -- the answer can still change the outcome: an explicit chaseSide is the player's call and
            -- a fixed pipe chopper never uses these two positions at all. A side already ruled out
            -- cannot be ruled out twice either.
            if (not leftBlocked) and (not self:isChaseSideReachable(leftChasePos)) then
                leftBlocked = true
            end
            if (not rightBlocked) and (not self:isChaseSideReachable(rightChasePos)) then
                rightBlocked = true
            end
        end

        -- Default to the side of the harvester the unloader is already on
        -- then check if there is a better side
        chaseNode = leftChasePos
        sideIndex = AutoDrive.CHASEPOS_LEFT
        local unloaderPos, _ = self:getUnloaderOnSide()
        if unloaderPos == AutoDrive.CHASEPOS_RIGHT then
            chaseNode = rightChasePos
            sideIndex = AutoDrive.CHASEPOS_RIGHT
        end
        -- following order is important!
        if self.combine.ad.isFixedPipeChopper then
            -- chopper with fixed unloading
            local sideOffsetZ = self:getDynamicSideChaseOffsetZ_fromDischargeNode(planningPhase)
            chaseNode = self:getPipeChaseWayPoint(0, sideOffsetZ)
            sideIndex = self.pipeSide
        elseif AutoDrive:getIsCPActive(self.combine) and AutoDrive.combineIsTurning(self.combine) then
            chaseNode = rearChasePos
            sideIndex = AutoDrive.CHASEPOS_REAR
        elseif chaseSideSetting ~= AutoDrive.CHASEPOS_AUTO then
            -- consider chaseSide setting
            sideIndex = chaseSideSetting
            if chaseSideSetting == AutoDrive.CHASEPOS_LEFT then
                chaseNode = leftChasePos
            elseif chaseSideSetting == AutoDrive.CHASEPOS_RIGHT then
                chaseNode = rightChasePos
            else
                chaseNode = rearChasePos
            end
        elseif (not leftBlocked) and self:mayUseChaseSide(AutoDrive.CHASEPOS_LEFT) and ((self:isUnloaderOnCorrectSide(AutoDrive.CHASEPOS_LEFT) and angleToLeftChaseSide < angleToRearChaseSide) or planningPhase) then
            chaseNode = leftChasePos
            sideIndex = AutoDrive.CHASEPOS_LEFT
        elseif (not rightBlocked) and self:mayUseChaseSide(AutoDrive.CHASEPOS_RIGHT) and (self:isUnloaderOnCorrectSide(AutoDrive.CHASEPOS_RIGHT) or planningPhase) then
            chaseNode = rightChasePos
            sideIndex = AutoDrive.CHASEPOS_RIGHT
        elseif not self.combine.ad.isSugarcaneHarvester then
            chaseNode = rearChasePos
            sideIndex = AutoDrive.CHASEPOS_REAR
        end
    else
        -- harveser with bunker
        --CombineUnloaderMode.debugMsg(self.vehicle, "CombineUnloaderMode:getPipeChasePosition:IsNormalCombine")
        local sideChasePos = AutoDrive.createWayPointRelativeToVehicle(self.combine, self.pipeSide * (sideChaseTermX + self:getPipeSlopeCorrection()), sideChaseTermZ)

        AutoDrive.useNewPipeOffsets = true

        if AutoDrive.useNewPipeOffsets and AutoDrive.isPipeOut(self.combine) then
            local sideOffsetZ = self:getDynamicSideChaseOffsetZ_fromDischargeNode(planningPhase)
            -- local sideOffsetX = self.pipeSide * self:getSideChaseOffsetX_new()
            -- sideChasePos = AutoDrive.createWayPointRelativeToDischargeNode(self.combine, sideOffsetX, sideOffsetZ)
            sideChasePos = self:getPipeChaseWayPoint(0, sideOffsetZ)
        end

        -- Same as for the chopper: the sensors belong to the harvester and cannot tell us whether we
        -- can get to the pipe side. Done before the rear chase offset is derived, so that a side we
        -- cannot reach is not the one the rear position leans towards either.
        if self.pipeSide == AutoDrive.CHASEPOS_LEFT and (not leftBlocked) and (not self:isChaseSideReachable(sideChasePos)) then
            leftBlocked = true
        elseif self.pipeSide == AutoDrive.CHASEPOS_RIGHT and (not rightBlocked) and (not self:isChaseSideReachable(sideChasePos)) then
            rightBlocked = true
        end

        local rearChaseTermX = self:getRearChaseOffsetX(leftBlocked, rightBlocked)
        local rearChasePos = AutoDrive.createWayPointRelativeToVehicle(self.combine, rearChaseTermX, rearChaseTermZ)
        local angleToSideChaseSide = self:getAngleToChasePos(sideChasePos)
        local angleToRearChaseSide = self:getAngleToChasePos(rearChasePos)

        -- Unlike the chopper above, this branch has only one flank to offer, so the reservation is
        -- checked around the whole decision rather than per side: if another unloader holds the pipe
        -- side, the rear is the only remaining answer. The planning phase is inside the guard on
        -- purpose - a plan that ignores the reservation would predict a position the drive itself
        -- then refuses to take.
        local mayUsePipeSide = self:mayUseChaseSide(self.pipeSide)

        if
            mayUsePipeSide
            and
            (
                (
                    (
                        (self.pipeSide == AutoDrive.CHASEPOS_LEFT and not leftBlocked)
                        or (self.pipeSide == AutoDrive.CHASEPOS_RIGHT and not rightBlocked)
                    )
                    and
                    (
                        (
                            self:isUnloaderOnCorrectSide(self.pipeSide)
                            -- and math.abs(angleToSideChaseSide) < math.abs(angleToRearChaseSide)
                        )
                        or (planningPhase == true)
                    )
                )
                or
                (
                    (planningPhase == true) and (self.combine.ad.noMovementTimer.elapsedTime > 1000)
                )
            )
         then
            -- Take into account a right sided harvester, e.g. potato harvester.
            chaseNode = sideChasePos
            sideIndex = self.pipeSide
         else
            sideIndex = AutoDrive.CHASEPOS_REAR
            chaseNode = rearChasePos
         end
    end

    -- Long term coordination between several unloaders: reserve the side we just picked.
    --
    -- Everything above derives the side from the harvester alone, so two unloaders sent to the same
    -- harvester reach the same answer and converge on the same few square metres. Whoever asks
    -- first holds the side; the other one is told so and can pick differently, instead of both
    -- discovering the problem when a sensor finally sees the other vehicle.
    --
    -- Nothing is claimed while merely planning: that call is a look-ahead, not a commitment, and
    -- claiming there would let a vehicle reserve a spot it never drives to.
    if not planningPhase and self.combine ~= nil and self.vehicle ~= nil then
        ADHarvestManager:claimApproach(self.vehicle, self.combine, sideIndex)
    end

    self.chasePosIndex = sideIndex
    return chaseNode, self.chasePosIndex
end

--- Whether this unloader may use a side of its harvester, i.e. nobody else has reserved it.
function CombineUnloaderMode:mayUseChaseSide(side)
    if self.combine == nil or self.vehicle == nil then
        return true
    end
    return not ADHarvestManager:isApproachClaimedByOther(self.vehicle, self.combine, side)
end

function CombineUnloaderMode:getAngleToCombineHeading()
    if self.vehicle == nil or self.combine == nil then
        return math.huge
    end

    local combineRx, _, combineRz = AutoDrive.localDirectionToWorld(self.combine, 0, 0, 1, self.combine.ad.ADRootNode)
    local rx, _, rz = AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)

    return math.abs(AutoDrive.angleBetween({x = rx, z = rz}, {x = combineRx, z = combineRz}))
end

function CombineUnloaderMode:getAngleToCombine()
    if self.vehicle == nil or self.combine == nil then
        return math.huge
    end

    local vehicleX, _, vehicleZ = getWorldTranslation(self.vehicle.components[1].node)
    local combineX, _, combineZ = getWorldTranslation(self.combine.components[1].node)
    local rx, _, rz =  AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1, self.combine.ad.ADRootNode)

    return math.abs(AutoDrive.angleBetween({x = rx, z = rz}, {x = combineX - vehicleX, z = combineZ - vehicleZ}))
end

function CombineUnloaderMode:getAngleToChasePos(chasePos)
    local worldX, _, worldZ = getWorldTranslation(self.vehicle.components[1].node)
    local rx, _, rz =  AutoDrive.localDirectionToWorld(self.vehicle, 0, 0, 1)
    local angle = AutoDrive.angleBetween({x = rx, z = rz}, {x = chasePos.x - worldX, z = chasePos.z - worldZ})

    return angle
end

function CombineUnloaderMode:getFollowingUnloader()
    return self.followingUnloader
end

function CombineUnloaderMode:registerFollowingUnloader(followingUnloader)
    self.followingUnloader = followingUnloader
end

function CombineUnloaderMode:unregisterFollowingUnloader()
    self.followingUnloader = nil
end

function CombineUnloaderMode:setStateNames()
    if self.statesToNames == nil then
        self.statesToNames = {}
        for name, id in pairs(CombineUnloaderMode) do
            if string.sub(name, 1, 6) == "STATE_" then
                self.statesToNames[id] = name
            end
        end
    end
end

function CombineUnloaderMode:getStateName(state)
    local requestedState = state
    if requestedState == nil then
        requestedState = self.state
    end
    if requestedState == nil then
        Logging.error("[AD] CombineUnloaderMode: Could not find name for state ->%s<- !", tostring(requestedState))
    end
    return self.statesToNames[requestedState] or ""
end

function CombineUnloaderMode.debugMsg(vehicle, debugText, ...)
    if CombineUnloaderMode.debug == true then
        AutoDrive.debugMsg(vehicle, debugText, ...)
    else
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_COMBINEINFO, debugText, ...)
    end
end
