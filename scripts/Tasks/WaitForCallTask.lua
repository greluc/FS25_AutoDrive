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

--- Stop stepping aside after this long even if the target was not reached; the point is to free the
--- spot, not to complete a manoeuvre.
WaitForCallTask.MAKE_WAY_TIMEOUT = 20000

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
        self:updateMakingWay(dt)
    else
        self.vehicle.ad.specialDrivingModule:stopVehicle()
    end

    self.vehicle.ad.specialDrivingModule:update(dt)
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

    local x, _, z = getWorldTranslation(self.vehicle.components[1].node)
    local wayPoint = ADGraphManager:getNearestWayPointWithin({x = x, z = z}, AutoDrive.WAITING_NETWORK_CLEARANCE)
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

--- Pick a direction and a target. Away from whatever we were told to clear: if it is ahead of us we
--- back off, if it is behind or beside us we pull forward. Straight along our own axis rather than
--- towards a computed free spot - this runs without a path search, and a short move along the axis
--- the vehicle is already lined up with is the one manoeuvre that needs none.
function WaitForCallTask:startMakingWay()
    local request = AutoDrive.getMakeWayRequest(self.vehicle)
    if request == nil or request.awayFromX == nil or request.awayFromZ == nil then
        AutoDrive.clearMakeWayRequest(self.vehicle)
        return
    end

    local _, _, awayFromZ = AutoDrive.worldToLocal(self.vehicle, request.awayFromX, 0, request.awayFromZ)
    self.makeWayReverse = awayFromZ > 0

    local offset = AutoDrive.MAKE_WAY_DISTANCE
    if self.makeWayReverse then
        offset = -offset
    end
    local tx, ty, tz = AutoDrive.localToWorld(self.vehicle, 0, 0, offset)
    self.makeWayTarget = { x = tx, y = ty, z = tz }

    local sx, sy, sz = getWorldTranslation(self.vehicle.components[1].node)
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
        self.vehicle.ad.specialDrivingModule:reverseToTargetLocation(dt, self.makeWayTarget, 6)
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
