classdef AnomalousVehicleHistory %< matlab.mixin.Copyable
    %ANOMALOUSVEHICLEHISTORY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        maxVehicleCount;
        history; % Row: vehicles, Col: timestep; 1: anomalous, 0: normal
        entryCount = 0;
    end
    
    properties (Constant)
        NW = 10;
        NC = 5;
        
        DEFAULT_HISTORY_ENTRY_COUNT = 500;
    end
    
    methods
        
        function obj = AnomalousVehicleHistory(maxVehicleCount)    
            obj.maxVehicleCount = maxVehicleCount;
            obj.history = true(maxVehicleCount, obj.DEFAULT_HISTORY_ENTRY_COUNT);            
        end
        
        function obj_out = copy(obj)
            obj_out = AnomalousVehicleHistory(obj.maxVehicleCount);
            obj_out.history = obj.history;
            obj_out.entryCount = obj.entryCount;
        end
        
        function [] = updateTable(obj, anomalousFlagArr)
            
            % (Input) anomalousFlagArr: an array whose rows are vehicles
            % and columns are algorithms. 
            % 1   <- anomalous, 
            % 0   <- normal,
            % nan <- no solution
            
            summarizedFlagArr = nan(size(obj.history,1),1);
            
            %%%% Consider a non-ego vehicle is anomalous if any of the
            %%%% algorithms (except solution-space) determines it to be
            %%%% anomalous.
            
            summarizedFlagArr(1:size(anomalousFlagArr,1),1) = ((sum(anomalousFlagArr,2) - anomalousFlagArr(:,AnomalyEquationSolver.ALG_SOLUTION_SPACE)) >= 1);
            
            %%%% For ego vehicle
            %%%% Anomalous <- Trust-Ego cannot find a solution
            %%%% Normal    <- AEnO cannot find a solution
            %%%% Otherwise <- Determined by Greedy solution
            
            if isnan(anomalousFlagArr(1, AnomalyEquationSolver.ALG_TRUST_EGO))
                summarizedFlagArr(1,1) = 1;
            elseif isnan(anomalousFlagArr(1, AnomalyEquationSolver.ALG_ANOMALOUS_EGO_OTHERS))
                summarizedFlagArr(1,1) = 0;
            else
                summarizedFlagArr(1,1) = anomalousFlagArr(1, AnomalyEquationSolver.ALG_GREEDY);
            end
            
            obj.entryCount = obj.entryCount + 1;
            obj.history(:,obj.entryCount) = (summarizedFlagArr == 1);
            
        end

        function wArr = getLatestWArr(obj)
            
            vehicleCount = size(obj.history,1);

            tempWArr = zeros(vehicleCount,size(obj.history,2));

            for vIndex = 1 : vehicleCount
                tempWArr(vIndex,:) = obj.computeWeightedAnomalyCount(obj.history(vIndex,:)');
            end

            wArr = tempWArr(:,end);

        end
        
    end
    
    methods (Static)
        
        function value = computeWeightedAnomalyCount(anomalousFlagColumn)
            
            P = AnomalousVehicleHistory.computeAnomalyProbability(anomalousFlagColumn);
            Q = AnomalousVehicleHistory.computeWeightedLikelyhood(anomalousFlagColumn);
            
            value = P + Q;
            value = value .* (value < 1) + (value >= 1);
            
        end
        
        function value = computeAnomalyProbability(anoamlousFlagArr)
            
            dataSize = size(anoamlousFlagArr,1);
            
            value = zeros(dataSize, size(anoamlousFlagArr,2));
            
            for eIndex = 1 : dataSize
                if eIndex < AnomalousVehicleHistory.NC
                    value(eIndex,:) = (AnomalousVehicleHistory.NC - eIndex + sum(anoamlousFlagArr(1:eIndex,:))) / AnomalousVehicleHistory.NC;
                else
                    value(eIndex,:) = sum(anoamlousFlagArr(1:eIndex,:)) / eIndex;
                end
            end
            
        end
        
        function value = computeWeightedLikelyhood(anomalousDataColumn)
                      
            dataSize = size(anomalousDataColumn,1);
            
            value = zeros(dataSize,1);
                 
            % Count-based computation
            for eIndex =  1 : dataSize
                
                indexOffset = max([eIndex - AnomalousVehicleHistory.NW, 1]);
                for cIndex = 1 : size(anomalousDataColumn,2)
                    value(eIndex,cIndex) = sum(AnomalousVehicleHistory.correlationTemplate(1:sum(anomalousDataColumn(indexOffset:eIndex,cIndex))));
                end
                
                
            end
            
            % Correlation-based calculation
            % for nIndex =  1 : dataSize                
            %     indexOffset = nIndex - AnomalousVehicleHistory.NW;                
            %     for iIndex = 1 : AnomalousVehicleHistory.NW                                        
            %         response = AnomalousVehicleHistory.correlationTemplate(iIndex);                     
            %         if iIndex + nIndex - AnomalousVehicleHistory.NW > 0                    
            %             value(nIndex,:) = value(nIndex,:) + data(iIndex + indexOffset,:) * response;
            %         end                    
            %     end
            % end
            
        end
        
        function value = correlationTemplate(steps)
            
            alpha = 0.9;
            
            
            nw = AnomalousVehicleHistory.NW;
            
            totalWeight = (alpha .^ nw - 1) / (alpha - 1); 
            %totalWeight = 1;
            
            %value = alpha ^ (nw - indices) / totalWeight;
            value = alpha .^ (steps-1) / totalWeight;
            
            value = value .* (steps <= nw);
            
        end
        
    end
    
end

