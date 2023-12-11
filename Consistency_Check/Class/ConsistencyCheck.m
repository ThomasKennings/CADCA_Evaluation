classdef ConsistencyCheck
        
    
    properties
        thresholds = [...
            3;    % DS1 (m)
             5;    % DS2 (m/s)
             5;    % DS3 (m/s2)
            0.5;   % DS4 (rad/s)
            pi/16; % DS5 (rad)
            1;     % DS6 (m)
            ];
    end
    
    properties (Constant)
        
    end
    
    methods 
        
        function lambda = rtcl_wrapToPi(obj, lambda)
            q = (lambda < -pi) | (pi < lambda);
            lambda(q) = obj.rtcl_wrapTo2Pi(lambda(q) + pi) - pi;
        end
        
        function lambda = rtcl_wrapTo2Pi(obj, lambda)
            positiveInput = (lambda > 0);
            lambda = mod(lambda, 2*pi);
            lambda((lambda == 0) & positiveInput) = 2*pi;
        end
                
        function [eqSys, eqVal, maxVehicle] = performConsistencyCheck(obj, currentStateTable, oldStateTable, interval)
                     
            % Should be consistent with AnomalyEquationSolver
            
            %%%% Determine the number of vehicles by finding the first
            %%%% entry with vehicle-ID = 0
            
            [maxVehicle] = find(currentStateTable(:,SimulationVehicleData.TABLE_INDEX_VEHICLE_ID) == 0, 1) - 1;
            if isempty(maxVehicle)
                maxVehicle = 10;
            end
            maxVehicle = maxVehicle(1);
            
            eqSys = AnomalyEquationSolver.genEqSys(maxVehicle-1);
            
            %%%% Perform actual consistency check to generate eqVal
            eqVal = true(size(eqSys,1));
            
            otherBlockSize = size(AnomalyEquationSolver.DEFAULT_EQ_SYS_OTHER_BLOCK,1);
            
            tempDSResult = true(otherBlockSize,1);
            
            locEgoNew = currentStateTable(1, SimulationVehicleData.TABLE_INDEX_POS_X:SimulationVehicleData.TABLE_INDEX_POS_Y);
            for vIndex = 1 : maxVehicle
                
                %%%% Retrieve data from state tables
                
                locOld = oldStateTable(vIndex, SimulationVehicleData.TABLE_INDEX_POS_X:SimulationVehicleData.TABLE_INDEX_POS_Y);
                speedOld = norm(oldStateTable(vIndex, SimulationVehicleData.TABLE_INDEX_V_X:SimulationVehicleData.TABLE_INDEX_V_Y));
                accOld = norm(oldStateTable(vIndex, SimulationVehicleData.TABLE_INDEX_ACC_X:SimulationVehicleData.TABLE_INDEX_ACC_Y));
                yawRateOld = deg2rad(oldStateTable(vIndex, 1));
                headingOld = deg2rad(oldStateTable(vIndex, 1));
                
                locNew = currentStateTable(vIndex, SimulationVehicleData.TABLE_INDEX_POS_X:SimulationVehicleData.TABLE_INDEX_POS_Y);
                speedNew = norm(currentStateTable(vIndex, SimulationVehicleData.TABLE_INDEX_V_X:SimulationVehicleData.TABLE_INDEX_V_Y));
                %accNew = norm(currentStateTable(vIndex, SimulationVehicleData.TABLE_INDEX_ACC_X:SimulationVehicleData.TABLE_INDEX_ACC_Y));
                %yawRateNew = deg2rad(currentStateTable(vIndex, SimulationVehicleData.TABLE_INDEX_YAW_RATE));
                headingNew = deg2rad(currentStateTable(vIndex, SimulationVehicleData.TABLE_INDEX_H));
                distNew = currentStateTable(vIndex, SimulationVehicleData.TABLE_INDEX_DIST);
                
                %%%% Perform consistency check for common vehicle data

                tempDSResult(1) = ~obj.checkLocConsistency(locOld, locNew, speedOld, accOld, yawRateOld, headingOld, interval);
                tempDSResult(2) = ~obj.checkLocSpeed(locOld, locNew, speedOld, interval);
                tempDSResult(3) = ~obj.checkSpeedAcc(speedOld, speedNew, accOld, interval);
                tempDSResult(4) = ~obj.checkYawRateHeading(headingOld, headingNew, yawRateOld, interval);
                tempDSResult(5) = ~obj.checkLocHeading(locOld, locNew, headingOld);
                
                %%%% Perform consistency check for distant measurement for
                %%%% non-ego vehicles

                if vIndex > 1
                    tempDSResult(6) = ~obj.checkRelativeDistance(locEgoNew, locNew, distNew);
                    indexOffet = (vIndex - 1) * otherBlockSize - 1;
                    eqVal(indexOffet+6) = tempDSResult(6);
                else
                    indexOffet = 0;
                end
                
                if length(eqVal) < indexOffet+5
                    for i=length(eqVal):indexOffet+5
                        eqVal(i+1) = true;
                    end
                end

                eqVal(indexOffet+1:indexOffet+5) = tempDSResult(1:5);
                
%                 if sum(eqVal == 0) > 0
%                     eqVal
%                 end
            
            end
            
%             if sum(eqVal == 0) > 0
%                 eqVal
%             end
            
            
        end
        
        function isAnomalous = checkLocConsistency(obj, locOld, locNew, speedOld, accOld, yawRateOld, headingOld, interval)
            % DS1
            locNewEst= RiskAssessment.predictLoc(locOld, speedOld, accOld, headingOld, yawRateOld, interval, 1);
            isAnomalous = (norm(locNewEst-locNew) > obj.thresholds(1));
        end
        
        function isAnomalous = checkLocSpeed(obj, locOld, locNew, speedOld, interval)
            % DS2            
            speedEst = norm(locNew - locOld) / interval;            
            isAnomalous = (abs(speedEst - speedOld) > obj.thresholds(2));
            
        end
        
        function isAnomalous = checkSpeedAcc(obj, speedOld, speedNew, accOld, interval)
            % DS3
            accEst = norm(speedNew - speedOld) / interval;            
            isAnomalous = (abs(accEst - accOld) > obj.thresholds(3));
        end
        
        function isAnomalous = checkYawRateHeading(obj, headingOld, headingNew, yawRateOld, interval)
            % DS4            
            if abs(headingNew - headingOld) < pi
                yawRateEst = obj.rtcl_wrapToPi(headingNew - headingOld) / interval;            
            else
                yawRateEst = obj.rtcl_wrapToPi(obj.rtcl_wrapTo2Pi(headingNew) - obj.rtcl_wrapTo2Pi(headingOld)) / interval;
            end
            isAnomalous = (abs(yawRateEst - yawRateOld) > obj.thresholds(4));            
        end
        
        function isAnomalous = checkLocHeading(obj, locOld, locNew, headingOld)
            % DS5            
            displaceVec = locNew - locOld;
            headingEst = atan2(displaceVec(2), displaceVec(1));
            if abs(headingEst - headingOld) > pi
                headingEst = obj.rtcl_wrapTo2Pi(headingEst);
                headingOld = obj.rtcl_wrapTo2Pi(headingOld);
            end
            
            isAnomalous = (abs(headingEst - headingOld) > obj.thresholds(5));            
        end
        
        function isAnomalous = checkRelativeDistance(obj, locEgo, locOther, dist)
            % DS6            
            distEst = norm(locEgo - locOther);
            isAnomalous = (abs(distEst - dist) > obj.thresholds(6));
        end
        
    end
end

