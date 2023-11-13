classdef AnomalyGenerator
    %ANOMALYGENERATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods
                
    end
    
    methods (Static)
        
        function [manipulatedTable, manipulatedFlag] = shiftData(conciseTable, vehicleID, dataIndex, offset, startEndIndex)
            
            manipulatedTable = conciseTable;
            
            if sum(offset) == 0
                return;
            end
            
            timeStepCount = size(conciseTable,3);
            
            manipulatedFlag = false(timeStepCount,1);
            
            for tIndex = startEndIndex(1) : min([timeStepCount, startEndIndex(2)])                
                manipulatedTable(vehicleID, dataIndex, tIndex) = manipulatedTable(vehicleID, dataIndex, tIndex) + offset;
                manipulatedFlag = true;
            end
            
            FLAG_PLOT_DATA = false;
            
            if FLAG_PLOT_DATA
                
                plotCount = length(dataIndex);
                
                figure
                for pIndex = 1 : plotCount                    
                    subplot(plotCount, 1, pIndex)
                    hold on
                    plot(squeeze(conciseTable(vehicleID, dataIndex(pIndex),:)), 'b', 'DisplayName', 'Ground Truth')
                    plot(squeeze(manipulatedTable(vehicleID, dataIndex(pIndex),:)), 'r', 'DisplayName', 'Manipulated')
                    legend('Location', 'best')
                    hold off                    
                end
                
                
            end
            
        end
        
    end
    
end

