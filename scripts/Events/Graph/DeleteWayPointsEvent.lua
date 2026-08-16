AutoDriveDeleteWayPointsEvent = {}
AutoDriveDeleteWayPointsEvent_mt = Class(AutoDriveDeleteWayPointsEvent, Event)

InitEventClass(AutoDriveDeleteWayPointsEvent, "AutoDriveDeleteWayPointsEvent")

function AutoDriveDeleteWayPointsEvent.emptyNew()
	local self = Event.new(AutoDriveDeleteWayPointsEvent_mt)
	return self
end

function AutoDriveDeleteWayPointsEvent.new(wayPointIDs)
    local self = AutoDriveDeleteWayPointsEvent.emptyNew()
	self.wayPointIDs = wayPointIDs or {}
	return self
end

function AutoDriveDeleteWayPointsEvent:writeStream(streamId, connection)
    local wayPointsCount = #self.wayPointIDs
    streamWriteUInt32(streamId, wayPointsCount)
    if wayPointsCount > 0 then
        for _, wayPointIDs in pairs(self.wayPointIDs) do
            streamWriteUIntN(streamId, wayPointIDs, 20)
        end
    end
end

function AutoDriveDeleteWayPointsEvent:readStream(streamId, connection)
    self.wayPointIDs = {}
    local wayPointsCount = streamReadUInt32(streamId)
    if wayPointsCount > 0 then
        for i = 1, wayPointsCount do
    	    self.wayPointIDs[i] = streamReadUIntN(streamId, 20)
        end
    end
	self:run(connection)
end

function AutoDriveDeleteWayPointsEvent:run(connection)
	if g_server ~= nil and connection:getIsServer() == false then
		-- If the event is coming from a client, server have only to broadcast
		AutoDriveDeleteWayPointsEvent.sendEvent(self.wayPointIDs)
	else
		-- If the event is coming from the server, both clients and server have to delete the way points.
        --
        -- As a SET, in one call. Looping removeWayPoint per id renumbered the whole graph once per
        -- deleted point, which is exactly the M*N cost removeWayPoints was written to remove - and
        -- this loop was the only way the game ever reached that code, so the batched version was
        -- unreachable and the comment describing the fix described something nothing used. Measured
        -- on the network sizes that comment cites: 200 points out of 55.595 took 13.5 s through here
        -- against 0.07 s through the batched call, on the server and on every client, because the
        -- event is broadcast to all of them.
        --
        -- It also removes a trap. removeWayPoints is order independent by construction; this loop
        -- was not, because each removal renumbers and every later id then means a different point.
        -- The two branches of one function disagreed for the same argument: {3,5,8} deleted the
        -- points at 30, 50 and 80 in place and the points at 30, 60 and 100 over the wire.
        if #self.wayPointIDs > 0 then
            ADGraphManager:removeWayPoints(self.wayPointIDs, false)
        end
	end
end

function AutoDriveDeleteWayPointsEvent.sendEvent(wayPointIDs)
	local event = AutoDriveDeleteWayPointsEvent.new(wayPointIDs)
	if g_server ~= nil then
		-- Server have to broadcast to all clients and himself
		g_server:broadcastEvent(event, true)
	else
		-- Client have to send to server
		g_client:getServerConnection():sendEvent(event)
	end
end
