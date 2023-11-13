classdef SimulationVehicleData
    %SIMULATIONVEHICLEDATA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        TABLE_INDEX_TIMESTAMP  = 1;
        TABLE_INDEX_VEHICLE_ID = 2;
        TABLE_INDEX_POS_X      = 3; % meters
        TABLE_INDEX_POS_Y      = 4; % meters
        TABLE_INDEX_V_X        = 5; % m/s
        TABLE_INDEX_V_Y        = 6; % m/s
        TABLE_INDEX_ACC_X      = 7; % m/s2
        TABLE_INDEX_ACC_Y      = 8; % m/s2
        TABLE_INDEX_H          = 9; % degree
        TABLE_INDEX_YAW_RATE   =10; % degree / s
        TABLE_INDEX_DIST       =11;
        TABLE_INDEX_TOTAL      =11;
        
        MAX_VEHICLE_COUNT = 10;
    end
    
    methods (Static)
        
        function [vehicleDataTable, oldDataTable, updateTable] = generateVehicleDataTable(egoData, otherData, pastTable, systemClock)
          
            oldDataTable = zeros(SimulationVehicleData.MAX_VEHICLE_COUNT, SimulationVehicleData.TABLE_INDEX_TOTAL);
            
            updateTable = 0;
            
            if size(pastTable,1) == 1
                %%%% Treat the data differently if we are dealing with the
                %%%% first batch of the data
                pastTable = zeros(SimulationVehicleData.MAX_VEHICLE_COUNT, SimulationVehicleData.TABLE_INDEX_TOTAL);
                updateTable = 1;
            end
                
            if pastTable(1,1) < systemClock
                updateTable = 1;
            end
            
            if updateTable == 1
                %%%% If time progresses, update vehicle table
                vehicleDataTable = zeros(SimulationVehicleData.MAX_VEHICLE_COUNT, SimulationVehicleData.TABLE_INDEX_TOTAL);
                egoPos = egoData.Position(1:2);
                egoVel = egoData.Velocity(1:2);
                egoYawRad = deg2rad(egoData.Yaw);
                egoYaw = egoData.Yaw;
                egoXUnitVector = [cos(egoYawRad), sin(egoYawRad)];
                egoYUnitVector = [cos(egoYawRad + pi/2), sin(egoYawRad+pi/2)];
                
                vehicleDataTable(egoData.ActorID,:) = SimulationVehicleData.formatActorToRowData(egoData, pastTable, systemClock, systemClock - pastTable(1,1), [0 0], [1 0], [0 1], [0 0], 0);
                %vehicleDataTable(egoData.ActorID,SimulationVehicleData.TABLE_INDEX_YAW_RATE) = egoData.AngularVelocity(3);
                vehicleDataTable(egoData.ActorID,SimulationVehicleData.TABLE_INDEX_DIST ) = 0;
                
                for vIndex = 1 : otherData.NumActors
                    tempActor = otherData.Actors(vIndex);
                    vehicleDataTable(tempActor.ActorID,:) = SimulationVehicleData.formatActorToRowData(tempActor, pastTable, systemClock, systemClock - pastTable(1,1), egoPos, egoXUnitVector, egoYUnitVector, egoVel, egoYaw);
                end
                
                %vehicleDataTable
                %fprintf('%f \n', vehicleDataTable(1,1));
                
                oldDataTable = pastTable;
                oldDataTable(:, SimulationVehicleData.TABLE_INDEX_ACC_X:SimulationVehicleData.TABLE_INDEX_ACC_Y) = vehicleDataTable(:, SimulationVehicleData.TABLE_INDEX_ACC_X:SimulationVehicleData.TABLE_INDEX_ACC_Y);
                oldDataTable(:, SimulationVehicleData.TABLE_INDEX_YAW_RATE) = vehicleDataTable(:, SimulationVehicleData.TABLE_INDEX_YAW_RATE);
                %oldDataTable
            else
                %%%% If timestamp is the same, do nothing
                vehicleDataTable = pastTable;
            end            
        end
        
        function dataRow = formatActorToRowData(actor, pastTable, timestamp, timeStep, egoPos, egoXUnitVector, egoYUnitVector, egoVel, egoYaw)
            
            %pos = egoXUnitVector * actor.Position(1) + egoYUnitVector * actor.Position(2); % + egoPos;
            %vel = egoXUnitVector * actor.Velocity(1) + egoYUnitVector * actor.Velocity(2); % + egoVel;
            
            pos = actor.Position;
            vel = actor.Velocity;
            
            heading = actor.Yaw; % + egoYaw;
            
            dataRow = [ timestamp, actor.ActorID,... 
                pos(1), pos(2),...
                vel(1), vel(2),...
                (vel(1) - pastTable(actor.ActorID, SimulationVehicleData.TABLE_INDEX_V_X)) / timeStep, (vel(2) - pastTable(actor.ActorID, SimulationVehicleData.TABLE_INDEX_V_Y)) / timeStep,...
                heading, (deg2rad(heading - pastTable(actor.ActorID, SimulationVehicleData.TABLE_INDEX_H))) / timeStep, norm(egoPos - pos(1:2))];
            
        end
        
        function displayDataTable(dataTable, updatedTable)
            
            conciseTable = SimulationVehicleData.getConciseTable(dataTable, updatedTable);
            
            for timeIndex = 1 : size(conciseTable,3)
                conciseTable(:,:,timeIndex)
            end
            
            
        end
        
        function conciseTable = getConciseTable(dataTable, updatedTable)            
            conciseTable = dataTable(:,:,updatedTable==1);            
        end
        
        function conciseTableNew = recomputeYawRate(conciseTable)
            
            conciseTableNew = conciseTable;
            
            for rIndex = 2 : size(conciseTable,1)
                
                for tIndex = 2 : size(conciseTable,3)
                    
                    headingOldDeg = conciseTable(rIndex, SimulationVehicleData.TABLE_INDEX_H, tIndex-1);
                    headingNewDeg = conciseTable(rIndex, SimulationVehicleData.TABLE_INDEX_H, tIndex);
                    timeStep = conciseTable(rIndex, SimulationVehicleData.TABLE_INDEX_TIMESTAMP, tIndex) - conciseTable(rIndex, SimulationVehicleData.TABLE_INDEX_TIMESTAMP, tIndex-1);
                    
                    if timeStep > 0
                        if abs(headingNewDeg - headingOldDeg) < 180
                            conciseTableNew(rIndex, SimulationVehicleData.TABLE_INDEX_YAW_RATE, tIndex-1) = wrapTo180(headingNewDeg - headingOldDeg) / timeStep;
                        else
                            conciseTableNew(rIndex, SimulationVehicleData.TABLE_INDEX_YAW_RATE, tIndex-1) = wrapTo180(wrapTo360(headingNewDeg) - wrapTo360(headingOldDeg)) / timeStep;
                        end
                    end
                    
                end
                
            end
            
        end
        
    end
end

