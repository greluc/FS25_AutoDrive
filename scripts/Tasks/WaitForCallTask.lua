--[[
Waiting for the harvest manager to call this unloader to a harvester.

The vehicle stands still while it waits. What it must not do is stand still unconditionally: an
unloader with nothing to do parks itself wherever it last was, which on a field is regularly the
spot another unloader - or the harvester - needs next. Somebody who is blocked can therefore ask
this vehicle to clear the spot, and it moves a short distance and parks again.

Whether it also drives somewhere sensible before parking in the first place is decided in
CombineUnloaderMode:setToWaitForCall through the waitingPosition setting; this task only handles
standing still and stepping aside.
]]

WaitForCallTask = ADInheritsFrom(AbstractTask)

WaitForCallTask.STATE_WAITING = 1
WaitForCallTask.STATE_MAKING_WAY = 2

--- Read by AutoDrive:requestMakeWay to tell a parked vehicle from a driving one.
WaitForCallTask.canMakeWay = true

--- Read by CombineUnloaderMode's stuck detection, which has to hold off while this runs: a nudge out
--- of a tight spot meets resistance by design, and the manoeuvre has its own timeout.
function WaitForCallTask:isMakingWay()
    return self.state == WaitForCallTask.STATE_MAKING_WAY
end

--- Stop stepping aside after this long even if the target was not reached; the point is to free the
--- spot, not to complete a manoeuvre.
WaitForCallTask.MAKE_WAY_TIMEOUT = 20000

--- How far astern a target has to be, beyond the length of the train, before it is reversed to, and
--- how far off dead astern it may sit.
---
--- The reverse controller measures arrival from the REVERSE NODE - the towed implement's axle, well
--- behind the root this target was measured from - and treats anything more than eighty degrees off
--- that node's rear axis as already reached. It then returns without commanding drive or brake, so a
--- target out to the side would leave the vehicle rolling free for the whole timeout. Forward steers
--- towards anything; reverse only works for something genuinely behind the whole train.
WaitForCallTask.REVERSE_ASTERN_MARGIN = 4
WaitForCallTask.REVERSE_CONE = 0.4

function WaitForCallTask:new(vehicle)
    local o = WaitForCallTask:create()
    o.vehicle = vehicle
    o.state = WaitForCallTask.STATE_WAITING
    o.makeWayTimer = AutoDriveTON:new()
    return o
end

function WaitForCallTask:setUp()
    ADHarvestManager:registerAsUnloader(self.vehicle)
    self.state = WaitForCallTask.STATE_WAITING
    self.clearanceAttempts = 0
    self.vehicle.ad.specialDrivingModule:stopVehicle()
end

function WaitForCallTask:update(dt)
    if self.state == WaitForCallTask.STATE_WAITING then
        self:checkParkedOnNetwork()
        if AutoDrive.getMakeWayRequest(self.vehicle) ~= nil then
            self:startMakingWay()
        end
    end

    if self.state == WaitForCallTask.STATE_MAKING_WAY then
        -- Drives the vehicle itself. Deliberately no specialDrivingModule:update afterwards: the
        -- driving call already advances the module's stopped timer with this frame's dt, and doing
        -- it twice made that timer run at double rate - so a manoeuvre that met any resistance was
        -- declared stuck in five seconds instead of ten, and the mode tore the driver out of this
        -- task to reverse it out of a spot it was only trying to leave.
        self:updateMakingWay(dt)
    else
        self.vehicle.ad.specialDrivingModule:stopVehicle()
        self.vehicle.ad.specialDrivingModule:update(dt)
    end
end

--- Whether we are standing on the destination the player sent us to. Their choice outranks any
--- tidying of our own, and a marker is a way point, so without this the clearance check fires on
--- every driver that has finished its route.
function WaitForCallTask:isParkedOnItsOwnMarker()
    local stateModule = self.vehicle.ad ~= nil and self.vehicle.ad.stateModule or nil
    if stateModule == nil or stateModule.getFirstMarker == nil then
        return false
    end
    local marker = stateModule:getFirstMarker()
    if marker == nil or marker.id == nil then
        return false
    end
    local wayPoint = ADGraphManager:getWayPointById(marker.id)
    if wayPoint == nil then
        return false
    end
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    return MathUtil.vector2Length(wayPoint.x - x, wayPoint.z - z) < AutoDrive.WAITING_NETWORK_CLEARANCE
end

--- Nobody has to complain for a spot to be a bad one. A vehicle that comes to rest on the way point
--- network is standing on somebody's route - on many fields a collection route runs along the inside
--- of the border, all the way round to the exit - and every full trailer heading that way has to get
--- past it. So it checks its own spot and moves clear without being asked.
---
--- Capped, because on a densely recorded field the clearance may not exist anywhere nearby, and a
--- vehicle wandering the field in search of it is worse than one parked slightly awkwardly.
function WaitForCallTask:checkParkedOnNetwork()
    if not AutoDrive.getSetting("waitingPosition", self.vehicle) then
        return
    end
    if self.clearanceAttempts >= AutoDrive.WAITING_CLEARANCE_MAX_TRIES then
        return
    end
    if AutoDrive.getMakeWayRequest(self.vehicle) ~= nil then
        return -- already on the way somewhere
    end
    -- A destination marker IS a way point on the network - that is what a marker is - so a driver
    -- that has just parked on the spot the player chose for it sits within metres of one, and this
    -- check would fire on every single one of them, unconditionally, and shuffle the vehicle off the
    -- place it was sent to. The network only has to be kept clear where the player did NOT put us.
    if self:isParkedOnItsOwnMarker() then
        return
    end
    -- Same throttle and per-vehicle phase offset as every other spatial scan. A parked vehicle is
    -- not going anywhere between frames, so asking once in twenty answers the same question.
    if ((g_updateLoopIndex + self.vehicle.id) % AutoDrive.PERF_FRAMES) ~= 0 then
        return
    end

    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    -- reused, so the query does not allocate a point per call
    local query = self.networkQueryPoint
    if query == nil then
        query = {x = 0, z = 0}
        self.networkQueryPoint = query
    end
    query.x, query.z = x, z
    local wayPoint = ADGraphManager:getNearestWayPointWithin(query, AutoDrive.WAITING_NETWORK_CLEARANCE)
    if wayPoint == nil then
        return
    end

    self.clearanceAttempts = self.clearanceAttempts + 1
    AutoDrive.setMakeWayRequest(self.vehicle, wayPoint.x, wayPoint.z)
    if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_VEHICLEINFO) then
        AutoDrive.debugPrint(self.vehicle, AutoDrive.DC_VEHICLEINFO,
            "WaitForCallTask: parked on the network at way point %s - moving clear (attempt %d)",
            tostring(wayPoint.id), self.clearanceAttempts)
    end
end

--- Pick a target to step aside to.
---
--- The target is placed in WORLD space, on the line running from whatever we are clearing out
--- through us and onward. That direction is the one thing that reliably leads away, and it is not
--- the vehicle's own axis: a driver standing on a collection route is ALIGNED with that route,
--- having just driven it, so a target on its own axis is a target further along the very route it is
--- blocking. It would clear the spot and land on the next one, eighteen metres down.
---
--- driveToPoint steers towards the target, so a target off to the side is reached on a curve. Which
--- is the whole manoeuvre: leave the line, do not travel along it.
function WaitForCallTask:startMakingWay()
    local request = AutoDrive.getMakeWayRequest(self.vehicle)
    if request == nil or request.awayFromX == nil or request.awayFromZ == nil then
        AutoDrive.clearMakeWayRequest(self.vehicle)
        return
    end

    local sx, sy, sz = getWorldTranslation(self.vehicle.components[1].node)
    local dx, dz = sx - request.awayFromX, sz - request.awayFromZ
    local length = MathUtil.vector2Length(dx, dz)
    if length < 1 then
        -- Standing on the very thing we are clearing, so the line through it has no direction in it.
        -- Our own right is at least across whatever we are lined up with, which is what is wanted.
        local rx, _, rz = AutoDrive.localDirectionToWorld(self.vehicle, 1, 0, 0)
        dx, dz = rx, rz
        length = MathUtil.vector2Length(dx, dz)
    end
    if length < 0.001 then
        AutoDrive.clearMakeWayRequest(self.vehicle)
        return
    end

    local tx = sx + (dx / length) * AutoDrive.MAKE_WAY_DISTANCE
    local tz = sz + (dz / length) * AutoDrive.MAKE_WAY_DISTANCE
    local ty = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, tx, 300, tz)
    self.makeWayTarget = { x = tx, y = ty, z = tz }

    -- Reverse only for a target genuinely astern of the whole train, and near enough dead astern
    -- that the reverse controller will actually drive to it rather than declare it reached. A train
    -- longer than the step-aside distance therefore never reverses, which is the safe way round:
    -- forward steers towards anything. Computed against the target's real height, because passing 0
    -- for y on a map whose ground sits at fifty to a hundred and fifty metres lets the vehicle's own
    -- pitch decide the direction instead of the geometry.
    local targetLocalX, _, targetLocalZ = AutoDrive.worldToLocal(self.vehicle, tx, ty, tz)
    local trainLength = AutoDrive.getTractorTrainLength ~= nil
        and AutoDrive.getTractorTrainLength(self.vehicle, true, false) or 0
    self.makeWayReverse = targetLocalZ < -(trainLength + WaitForCallTask.REVERSE_ASTERN_MARGIN)
        and math.abs(targetLocalX) < math.abs(targetLocalZ) * WaitForCallTask.REVERSE_CONE

    self.makeWayStart = { x = sx, y = sy, z = sz }

    self.makeWayTimer:timer(false)
    self.state = WaitForCallTask.STATE_MAKING_WAY
    self.vehicle.ad.specialDrivingModule:releaseVehicle()
end

function WaitForCallTask:updateMakingWay(dt)
    local timedOut = self.makeWayTimer:timer(true, WaitForCallTask.MAKE_WAY_TIMEOUT, dt)
    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    local moved = MathUtil.vector2Length(x - self.makeWayStart.x, z - self.makeWayStart.z)

    -- Far enough counts as done well before the target is reached: the spot is free once we are
    -- out of it, and insisting on the full distance would walk the vehicle into the next problem.
    if timedOut or moved >= (AutoDrive.MAKE_WAY_DISTANCE * 0.75) then
        self:stopMakingWay()
        return
    end

    if self.makeWayReverse then
        -- A true return means the controller considers itself there - which it also does when the
        -- target is too far off its own rear axis to drive to. Either way there is nothing more it
        -- will do, and it commands no brake on that path, so the manoeuvre has to end rather than
        -- leave the vehicle released and rolling for the rest of the timeout.
        if self.vehicle.ad.specialDrivingModule:reverseToTargetLocation(dt, self.makeWayTarget, 6) then
            self:stopMakingWay()
        end
    else
        self.vehicle.ad.specialDrivingModule:driveToPoint(dt, self.makeWayTarget, 8, true, 0.5, 8)
    end
end

function WaitForCallTask:stopMakingWay()
    AutoDrive.clearMakeWayRequest(self.vehicle)
    self.makeWayTarget = nil
    self.makeWayStart = nil
    self.makeWayTimer:timer(false)
    self.state = WaitForCallTask.STATE_WAITING
    self.vehicle.ad.specialDrivingModule:stopVehicle()
end

function WaitForCallTask:abort()
    AutoDrive.clearMakeWayRequest(self.vehicle)
end

function WaitForCallTask:finished()
    AutoDrive.clearMakeWayRequest(self.vehicle)
    self.vehicle.ad.taskModule:setCurrentTaskFinished(self.propagate)
end

function WaitForCallTask:getInfoText()
    if self.state == WaitForCallTask.STATE_MAKING_WAY then
        return g_i18n:getText("AD_task_making_way")
    end
    return g_i18n:getText("AD_task_wait_for_call")
end

function WaitForCallTask:getI18nInfo()
    if self.state == WaitForCallTask.STATE_MAKING_WAY then
        return "$l10n_AD_task_making_way;"
    end
    return "$l10n_AD_task_wait_for_call;"
end
