classdef LocalViewConstructor < matlab.mixin.Copyable
    %LOCALVIEWCONSTRUCTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Static)
        
        function localViews = constructLocalViews(solutionCell, stateTable, stateTableOld)
            
            % (Output)localViews: a row cell that each column represents a
            % local view for each algorithm result.
            
            solutionCount = size(solutionCell,1);
            
            localViews = cell(1, solutionCount);
            
            %%%% Skip solution space algorithm
            localViews{1,1} = nan;
            vehicleMaxCount = size(stateTable,1);
            
            for sIndex = 2 : solutionCount
                
                localViews{sIndex} = stateTable;
                
                for vIndex = 1 : vehicleMaxCount                
                    tempStateRow = stateTable(vIndex,:);
                    
                    if tempStateRow(1, SimulationVehicleData.TABLE_INDEX_VEHICLE_ID) == 0
                        continue
                    end
                    
                    tempStateRowOld = stateTableOld(vIndex,:);
                    if ~isnan(solutionCell{sIndex})
                        normalFlag = solutionCell{sIndex}((vIndex-1) * AnomalyEquationSolver.INDEX_ALL_TYPES + 1: vIndex * AnomalyEquationSolver.INDEX_ALL_TYPES);
                        localViews{sIndex}(vIndex,:) = LocalViewConstructor.restoreAnomalousDataSingleRow(tempStateRow, tempStateRowOld, normalFlag);                    
                    end
                    
                end
                
            end
            
            
            
        end
        
        function restoredStateRow = restoreAnomalousDataSingleRow(tempStateRow, tempStateRowOld, normalFlag)
            
            % (Input) tempStateRow, tempStateRowOld: should be both
            % corrected version.
            
            % (Output) only correct P, V, and H
            
            restoredStateRow = tempStateRow;
            
            posOld = tempStateRowOld(SimulationVehicleData.TABLE_INDEX_POS_X:SimulationVehicleData.TABLE_INDEX_POS_Y);
            velOld1D = norm(tempStateRowOld(SimulationVehicleData.TABLE_INDEX_V_X:SimulationVehicleData.TABLE_INDEX_V_Y));
            accOld1D = norm(tempStateRowOld(SimulationVehicleData.TABLE_INDEX_ACC_X:SimulationVehicleData.TABLE_INDEX_ACC_Y));
            headingOldRad = deg2rad(tempStateRowOld(SimulationVehicleData.TABLE_INDEX_H));
            yawRateOldRad = deg2rad(tempStateRowOld(SimulationVehicleData.TABLE_INDEX_YAW_RATE));
            timeInterval = tempStateRow(SimulationVehicleData.TABLE_INDEX_TIMESTAMP) - tempStateRowOld(SimulationVehicleData.TABLE_INDEX_TIMESTAMP);
            stepTotalCount = 1;
            
            if normalFlag(AnomalyEquationSolver.INDEX_P) == false
                % Latest is incorrect
                
                if (normalFlag(AnomalyEquationSolver.INDEX_A) == true) && (normalFlag(AnomalyEquationSolver.INDEX_W) == true)
                    restoredStateRow(SimulationVehicleData.TABLE_INDEX_POS_X:SimulationVehicleData.TABLE_INDEX_POS_Y)... 
                        = RiskAssessment.predictLoc(posOld, velOld1D, accOld1D, headingOldRad, yawRateOldRad, timeInterval, stepTotalCount);
                else
                    restoredStateRow(SimulationVehicleData.TABLE_INDEX_POS_X:SimulationVehicleData.TABLE_INDEX_POS_Y)... 
                        = RiskAssessment.predictLoc(posOld, velOld1D, 0, headingOldRad, 0, timeInterval, stepTotalCount);
                end
                
            end
            
            if normalFlag(AnomalyEquationSolver.INDEX_V) == false
                % Latest is incorrect
                if normalFlag(AnomalyEquationSolver.INDEX_P) == true
                    posNow = tempStateRow(SimulationVehicleData.TABLE_INDEX_POS_X:SimulationVehicleData.TABLE_INDEX_POS_Y);
                    restoredStateRow(SimulationVehicleData.TABLE_INDEX_V_X:SimulationVehicleData.TABLE_INDEX_V_Y)...
                        = norm(posOld - posNow) / timeInterval * [cos(headingOldRad), sin(headingOldRad)];
                elseif normalFlag(AnomalyEquationSolver.INDEX_A) == true
                    restoredStateRow(SimulationVehicleData.TABLE_INDEX_V_X:SimulationVehicleData.TABLE_INDEX_V_Y)...
                        = tempStateRowOld(SimulationVehicleData.TABLE_INDEX_V_X:SimulationVehicleData.TABLE_INDEX_V_Y)...
                        + tempStateRowOld(SimulationVehicleData.TABLE_INDEX_ACC_X:SimulationVehicleData.TABLE_INDEX_ACC_Y) * timeInterval;
                else
                    restoredStateRow(SimulationVehicleData.TABLE_INDEX_V_X:SimulationVehicleData.TABLE_INDEX_V_Y)...
                        = tempStateRowOld(SimulationVehicleData.TABLE_INDEX_V_X:SimulationVehicleData.TABLE_INDEX_V_Y);
                end
            end
              
            if normalFlag(AnomalyEquationSolver.INDEX_H) == false
                % Latest is incorrect
                
                if normalFlag(AnomalyEquationSolver.INDEX_P) == true
                    posNow = tempStateRow(SimulationVehicleData.TABLE_INDEX_POS_X:SimulationVehicleData.TABLE_INDEX_POS_Y);                    
                    posDiff = posNow - posOld;
                    restoredStateRow(SimulationVehicleData.TABLE_INDEX_H) = atan2d(posDiff(2), posDiff(1));
                elseif normalFlag(AnomalyEquationSolver.INDEX_W) == true
                    restoredStateRow(SimulationVehicleData.TABLE_INDEX_H) = tempStateRowOld(SimulationVehicleData.TABLE_INDEX_H) + tempStateRowOld(SimulationVehicleData.TABLE_INDEX_YAW_RATE) * timeInterval;
                else
                    restoredStateRow(SimulationVehicleData.TABLE_INDEX_H) = tempStateRowOld(SimulationVehicleData.TABLE_INDEX_H);
                end
                
            end
                        
            
        end
        
    end
    
end

