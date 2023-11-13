classdef TestCaseGenerator < matlab.mixin.Copyable
    %TESTCASEGENERATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        timeStep = 0.1;  
        % seconds
        
        numOther = 2; 
        % Exclude ego vehicle
        
        offsetOther = [5 5; 10 10]; 
        % Relative locations of other vehicles
        
        statusInitEgo = [10 0];
        % [Speed heading]
        
        patternEgo = [0, 0, 0];
        % [Time, Acc (m/s2), Steering (rad);
        %  Time, Acc (m/s2), Steering (rad);
        %  ...
        % ]
        
        statusInitOther = [10 0; 10 0];
        
        patternOther = {[0, 0, 0]; [0, 0, 0]};
        
        simTime = 10; % seconds
        
        vehicleEgo = Vehicle([]);
        
        vehicleOther = {Vehicle([]); Vehicle([])};
        
    end
    
    properties (Constant)
        
        PATTERN_TIME     = 1;
        PATTERN_ACC      = 2;
        PATTERN_STEERING = 3;
        
        INIT_SPEED   = 1;
        INIT_HEADING = 2;
        
    end
    
    methods 
        
        function traceCell = performTraceGeneration(obj)
            
            traceCell = cell(1 + obj.numOther,1);
            
            traceCell{1} = TestCaseGenerator.performIndividualTraceGeneration(obj.vehicleEgo, [0 0], obj.statusInitEgo, obj.patternEgo, obj.timeStep, obj.simTime);
            
            for vIndex = 1 : obj.numOther
                traceCell{1+vIndex} = TestCaseGenerator.performIndividualTraceGeneration(obj.vehicleOther{vIndex}, obj.offsetOther(vIndex,:), obj.statusInitOther(vIndex,:), obj.patternOther{vIndex}, obj.timeStep, obj.simTime);
            end
            
        end
        
    end
    
    methods (Static)
        
        function traceArr = performIndividualTraceGeneration(vehicle, offset, statusInit, pattern, timeStep, simTime)
            
            timeArr = 0:timeStep:(ceil(simTime / timeStep) * timeStep);
            
            rowCount = length(timeArr);
            traceArr = zeros(rowCount, vehicle.STATUS_TOTAL);
            
            traceArr(:, vehicle.STATUS_TIME) = timeArr;
            
            patternCount = size(pattern,1);
            pIndex = 1;
            
            traceArr(1, vehicle.STATUS_LOC_X:vehicle.STATUS_LOC_Y) = offset;
            traceArr(1, vehicle.STATUS_V) = statusInit(TestCaseGenerator.INIT_SPEED);
            traceArr(1, vehicle.STATUS_H) = statusInit(TestCaseGenerator.INIT_HEADING);
            
            traceArr(1, vehicle.STATUS_A) = pattern(1, TestCaseGenerator.PATTERN_ACC);
            traceArr(1, vehicle.STATUS_W) = vehicle.computeYawRateFromSteering(vehicle, traceArr(1, vehicle.STATUS_V), pattern(1,TestCaseGenerator.PATTERN_STEERING));
            
            
            for rIndex = 2 : rowCount
                
                oldRow = traceArr(rIndex-1,:);
                tempRow = traceArr(rIndex,:);
                
                while(pIndex < patternCount)                
                    if(tempRow(1, vehicle.STATUS_TIME) >= pattern(pIndex+1, TestCaseGenerator.PATTERN_TIME))
                        pIndex = pIndex + 1;
                    else 
                        break;
                    end
                end
                
                % Acc according to the pattern
                tempRow(1, vehicle.STATUS_A) = pattern(pIndex, TestCaseGenerator.PATTERN_ACC);
            
                % Speed
                tempRow(1, vehicle.STATUS_V) = oldRow(1, vehicle.STATUS_V) + timeStep * tempRow(1, vehicle.STATUS_A);

                % Yaw Rate
                tempRow(1, vehicle.STATUS_W) = vehicle.computeYawRateFromSteering(vehicle, tempRow(1, vehicle.STATUS_V), pattern(pIndex,TestCaseGenerator.PATTERN_STEERING));

                % Heading
                tempRow(1, vehicle.STATUS_H) = oldRow(1, vehicle.STATUS_H) + timeStep * tempRow(1, vehicle.STATUS_W);
                
                % Location
                tempRow(1, vehicle.STATUS_LOC_X:vehicle.STATUS_LOC_Y) = oldRow(1, vehicle.STATUS_LOC_X:vehicle.STATUS_LOC_Y) + TestCaseGenerator.computeDisplacement(oldRow, timeStep);
                
                
                traceArr(rIndex,:) = tempRow;
                
            end
            
        end
        
    end
    
    methods (Static)
        %%%% Helper Function
        
        function displacement = computeDisplacement(status, timeStep)                        
                        
            v = status(1, Vehicle.STATUS_V);
            a = status(1, Vehicle.STATUS_A);
            w = status(1, Vehicle.STATUS_W);
            h = status(1, Vehicle.STATUS_H);
            t = timeStep;
            
            if w == 0
                
                dX = v * sin(h) * t + 0.5 * a * t ^ 2 * sin(h);
                dY = v * cos(h) * t + 0.5 * a * t ^ 2 * cos(h);
                
            else
                
                dX = (- (v/w + a/w * t) * cos(h + w * t) + (a/(w^2)) * sin(h + w * t)) - (- (v/w) * cos(h) + (a/(w^2)) * sin(h));
                dY = ((v/w + a/w * t) * sin(h + w * t) + (a/(w^2)) * cos(h + w * t)) - ((v/w) * sin(h) + (a/(w^2)) * cos(h));
            end
            
            displacement = [dX, dY];
            
        end
        
        function plotLocTrace(traceCell)
            
            traceCount = size(traceCell,1);
            
            figure
            hold on
            
            for tIndex = 1 : traceCount
                plot(traceCell{tIndex}(:,Vehicle.STATUS_LOC_X), traceCell{tIndex}(:,Vehicle.STATUS_LOC_Y));
            end
            
            hold off
            grid on
            axis equal
        end
        
    end
    
end

