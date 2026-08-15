ADScheduler = {}
ADScheduler.UPDATE_TIME = 3000 -- time interval for calculation/check
ADScheduler.FRAMES_TO_CHECK_FOR_ACTUAL_FPS = 10 -- number of frames to calculate actual FPS
ADScheduler.FRAMES_TO_CHECK_FOR_AVERAGE_FPS = 60 * 60 -- number of frames to calculate average FPS
ADScheduler.MIN_STEPS_PER_FRAME = 2 -- min steps for pathfinder per frame
ADScheduler.MAX_STEPS_PER_FRAME = 8 -- max steps for pathfinder per frame
ADScheduler.MAX_CONCURRENT_PATHFINDERS = 3 -- max vehicles searching for a path at the same time
ADScheduler.FPS_DIFFERENCE = 0.1   -- 10 % difference to average for calculation/check
ADScheduler.MIN_FPS = 20 -- min FPS where stepsPerFrame will always be decreased
ADScheduler.MAX_FPS = 60 -- max FPS for some calculations

function ADScheduler:load()
    self.actual_values = {}
    self.actual_counter = 0
    self.actual_fps = 0

    self.average_values = {}
    self.average_counter = 0
    self.average_fps = 0
    -- Running sum over average_values, maintained in update(), see updateAverageFPS.
    self.average_sum = 0
    self.average_filled = 0

    self.stepsPerFrame = ADScheduler.MAX_STEPS_PER_FRAME
    self.pathFinderVehicles = {}
    self.activeCount = 1
    self.activePathFinderVehicle = nil
    self.updateTimer = AutoDriveTON:new()
end

function ADScheduler:update(dt)
    self.updateTimer:timer(true, ADScheduler.UPDATE_TIME, dt)

    self.actual_counter = self.actual_counter + 1
    if self.actual_counter > ADScheduler.FRAMES_TO_CHECK_FOR_ACTUAL_FPS then
        self.actual_counter = 1
    end
    self.actual_values[self.actual_counter] = dt

    self.average_counter = self.average_counter + 1
    if self.average_counter > ADScheduler.FRAMES_TO_CHECK_FOR_AVERAGE_FPS then
        self.average_counter = 1
    end
    -- Keep the sum up to date while the slot is overwritten, so updateAverageFPS does not have to
    -- add up all 3600 values again every UPDATE_TIME. average_filled counts the occupied slots,
    -- which is what the average has to be divided by while the ring is still filling up.
    local previousValue = self.average_values[self.average_counter]
    if previousValue == nil then
        self.average_filled = self.average_filled + 1
    else
        self.average_sum = self.average_sum - previousValue
    end
    self.average_values[self.average_counter] = dt
    self.average_sum = self.average_sum + dt
    if (self.average_counter == ADScheduler.FRAMES_TO_CHECK_FOR_AVERAGE_FPS) and (self.average_fps > (ADScheduler.MAX_FPS - ADScheduler.MIN_FPS) / 2 + ADScheduler.MIN_FPS) then
        -- if average FPS is high we can increase in longer periods to reach the max stepsPerFrame somehow if feasible
        self.stepsPerFrame = math.min(ADScheduler.MAX_STEPS_PER_FRAME, self.stepsPerFrame + 1)
    end

    if self.updateTimer:done() then
        self.updateTimer:timer(false)

        if self.average_values[ADScheduler.FRAMES_TO_CHECK_FOR_AVERAGE_FPS] ~= nil and self.average_values[ADScheduler.FRAMES_TO_CHECK_FOR_AVERAGE_FPS] ~= 0 then
            -- average table filled - we can start the update
            self:updateAverageFPS()
        end

        if self.actual_values[ADScheduler.FRAMES_TO_CHECK_FOR_ACTUAL_FPS] ~= nil and self.actual_values[ADScheduler.FRAMES_TO_CHECK_FOR_ACTUAL_FPS] ~= 0 then
            -- actual table filled - we can start
            self:updateActualFPS()

            local averagefps = 0
            if self.average_fps == 0 then
                -- on start of game we set the average somehow until the average table is filled
                averagefps = (ADScheduler.MAX_FPS - ADScheduler.MIN_FPS) / 2 + ADScheduler.MIN_FPS
            else
                -- the average is calculated, so use it from now onwards
                averagefps = self.average_fps
            end
            if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
                AutoDrive.debugPrint(nil, AutoDrive.DC_PATHINFO, "Scheduler update self.actual_fps %.1f averagefps %.1f self.stepsPerFrame %s", self.actual_fps, averagefps, tostring(self.stepsPerFrame))
            end

            if self.actual_fps > averagefps * (1 + ADScheduler.FPS_DIFFERENCE) then
                -- actual FPS is higher than average, so we can increase stepsPerFrame
                self.stepsPerFrame = math.min(ADScheduler.MAX_STEPS_PER_FRAME, self.stepsPerFrame + 1)
            elseif self.actual_fps < averagefps * (1 - ADScheduler.FPS_DIFFERENCE) then
                -- actual FPS is lower than average, so we have to decrease stepsPerFrame
                self.stepsPerFrame = math.max(ADScheduler.MIN_STEPS_PER_FRAME, self.stepsPerFrame - 1)
            elseif self.actual_fps < ADScheduler.MIN_FPS then
                -- if FPS drift sligthly below MIN_FPS, reduce the stepsPerFrame
                self.stepsPerFrame = math.max(ADScheduler.MIN_STEPS_PER_FRAME, self.stepsPerFrame - 1)
            end
        end
        self:updateActiveVehicle() -- activate the next vehicle in queue
    end
end

function ADScheduler:updateActualFPS()
    -- calculate actual FPS by using some frame values to avoid short time drops to have to much influence
    local dt_sum = 0
    for i = 1, ADScheduler.FRAMES_TO_CHECK_FOR_ACTUAL_FPS, 1 do
        dt_sum = dt_sum + self.actual_values[i]
    end
    self.actual_fps = 1000 / (dt_sum / ADScheduler.FRAMES_TO_CHECK_FOR_ACTUAL_FPS)
end

function ADScheduler:updateAverageFPS()
    -- calculate the average FPS with huge number of values - the sum is maintained incrementally in
    -- update(), so this runs in constant time instead of walking 3600 slots every UPDATE_TIME
    if self.average_filled <= 0 or self.average_sum <= 0 then
        return
    end
    self.average_fps = 1000 / (self.average_sum / self.average_filled)
end

function ADScheduler:addPathfinderVehicle(vehicle)
    -- add a vehicle in the queue
    if not ADTable.contains(self.pathFinderVehicles, vehicle) then
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_PATHINFO, "Scheduler addPathfinderVehicle ")
        table.insert(self.pathFinderVehicles, vehicle)
        -- set new vehicle a delay
        if ADTable.count(self.pathFinderVehicles) > 1 then
            AutoDrive.debugPrint(vehicle, AutoDrive.DC_PATHINFO, "Scheduler addPathfinderVehicle addDelayTimer 5000")
            -- if already vehicle in table, pause this new one
            vehicle.ad.pathFinderModule:addDelayTimer(5000)
        end
    end
end

function ADScheduler:removePathfinderVehicle(vehicle)
    -- remove a vehicle from the queue
    if ADTable.contains(self.pathFinderVehicles, vehicle) then
        AutoDrive.debugPrint(vehicle, AutoDrive.DC_PATHINFO, "Scheduler removePathfinderVehicle")
        ADTable.removeValue(self.pathFinderVehicles, vehicle)
        if self.activePathFinderVehicle == vehicle then
            self.updateTimer:timer(true, ADScheduler.UPDATE_TIME, ADScheduler.UPDATE_TIME) -- shoutcut timer
        end
    end
end

-- How many vehicles may search for a path at the same time.
--
-- This used to be hard wired to one: every other waiting vehicle got a 5 second delay timer and the
-- active slot rotated every 3 seconds, so with five unloaders the last one could wait 15 seconds
-- before its search even started - visible as drivers standing at the field edge while the
-- harvester fills up.
--
-- The scheduler already measures the actual frame rate to decide how many search steps fit into a
-- frame; that same measurement decides how many searchers fit. The total step budget is split
-- between them, so running two searchers costs the same per frame as running one - it just finishes
-- both sooner instead of one after the other.
function ADScheduler:getMaxConcurrentPathfinders()
    local fps = self.actual_fps or 0
    if fps <= 0 then
        return 1 -- nothing measured yet, behave as before
    end
    if fps >= ADScheduler.MAX_FPS then
        return ADScheduler.MAX_CONCURRENT_PATHFINDERS
    end
    if fps >= (ADScheduler.MAX_FPS + ADScheduler.MIN_FPS) / 2 then
        return math.min(2, ADScheduler.MAX_CONCURRENT_PATHFINDERS)
    end
    return 1
end

-- Steps per frame each active searcher may spend. The sum stays within the budget the frame rate
-- allows, which is what keeps adding searchers from costing frame time.
function ADScheduler:getStepsPerFrame()
    local concurrent = math.max(1, self.activeCount or 1)
    return math.max(ADScheduler.MIN_STEPS_PER_FRAME, math.floor(self.stepsPerFrame / concurrent))
end

function ADScheduler:updateActiveVehicle()
    -- pause all vehicles
    for _, vehicle in pairs(self.pathFinderVehicles) do
        if vehicle and vehicle.ad and vehicle.ad.pathFinderModule then
            vehicle.ad.pathFinderModule:addDelayTimer(5000)
        end
    end
    if self.activePathFinderVehicle and ADTable.contains(self.pathFinderVehicles, self.activePathFinderVehicle) then
        -- add this vehicle to end of queue
        local vehicle = self.activePathFinderVehicle
        self.activePathFinderVehicle = nil
        self:removePathfinderVehicle(vehicle)
        self:addPathfinderVehicle(vehicle)
    end

    -- activate as many vehicles from the front of the queue as the frame rate allows
    local maxActive = self:getMaxConcurrentPathfinders()
    local activated = 0
    for i = 1, math.min(maxActive, #self.pathFinderVehicles) do
        local nextVehicle = self.pathFinderVehicles[i]
        if nextVehicle ~= nil then
            if AutoDrive.getDebugChannelIsSet(AutoDrive.DC_PATHINFO) then
                AutoDrive.debugPrint(nextVehicle, AutoDrive.DC_PATHINFO, "Scheduler updateActiveVehicle activate vehicle %d of %d", i, maxActive)
            end
            if nextVehicle.ad and nextVehicle.ad.pathFinderModule then
                nextVehicle.ad.pathFinderModule:addDelayTimer(0)
            end
            activated = activated + 1
            -- the first one keeps the "active" slot, which is what the rotation above cycles
            if i == 1 then
                self.activePathFinderVehicle = nextVehicle
            end
        end
    end
    self.activeCount = math.max(1, activated)
end
