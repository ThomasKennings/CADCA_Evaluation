classdef AnomalyEquationSolver
    %ANOMALYEQUARIONSOLVER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        INDEX_P = 1;
        INDEX_V = 2;
        INDEX_A = 3;
        INDEX_W = 4;
        INDEX_H = 5;
        
        INDEX_DIST_DIR = 6;
        
        INDEX_ALL_TYPES = 6;
        
        ID_EGO = 1;
        ID_OTHER = 2;
        
        ELE_INDEX_ID = 1;
        ELE_INDEX_TYPE = 2;
        
        EQ_VAL_NORMAL = 1;
        EQ_VAL_ANOMALOUS = 0;
        
        ALG_SOLUTION_SPACE       = 1;
        ALG_GREEDY               = 2;
        ALG_ANOMALOUS_INDIVIDUAL = 3;
        ALG_TRUST_EGO            = 4;
        ALG_ANOMALOUS_EGO_ONLY   = 5;
        ALG_ANOMALOUS_EGO_OTHERS = 6;
        
    end
    
    
    properties (Constant)    
        DEFAULT_EQ_SYS_EGO_BLOCK =...
            {...
                {[AnomalyEquationSolver.ID_EGO AnomalyEquationSolver.INDEX_P], [AnomalyEquationSolver.ID_EGO AnomalyEquationSolver.INDEX_V], [AnomalyEquationSolver.ID_EGO AnomalyEquationSolver.INDEX_A], [AnomalyEquationSolver.ID_EGO AnomalyEquationSolver.INDEX_W], [AnomalyEquationSolver.ID_EGO AnomalyEquationSolver.INDEX_H]};
                {[AnomalyEquationSolver.ID_EGO AnomalyEquationSolver.INDEX_P], [AnomalyEquationSolver.ID_EGO AnomalyEquationSolver.INDEX_V]};
                {[AnomalyEquationSolver.ID_EGO AnomalyEquationSolver.INDEX_P], [AnomalyEquationSolver.ID_EGO AnomalyEquationSolver.INDEX_H]};
                {[AnomalyEquationSolver.ID_EGO AnomalyEquationSolver.INDEX_V], [AnomalyEquationSolver.ID_EGO AnomalyEquationSolver.INDEX_A]};
                {[AnomalyEquationSolver.ID_EGO AnomalyEquationSolver.INDEX_H], [AnomalyEquationSolver.ID_EGO AnomalyEquationSolver.INDEX_W]};
            };
        
        DEFAULT_EQ_SYS_OTHER_BLOCK =...
            {...
                {[AnomalyEquationSolver.ID_OTHER AnomalyEquationSolver.INDEX_P], [AnomalyEquationSolver.ID_OTHER AnomalyEquationSolver.INDEX_V], [AnomalyEquationSolver.ID_OTHER AnomalyEquationSolver.INDEX_A], [AnomalyEquationSolver.ID_OTHER AnomalyEquationSolver.INDEX_W], [AnomalyEquationSolver.ID_OTHER AnomalyEquationSolver.INDEX_H]};
                {[AnomalyEquationSolver.ID_OTHER AnomalyEquationSolver.INDEX_P], [AnomalyEquationSolver.ID_OTHER AnomalyEquationSolver.INDEX_V]};
                {[AnomalyEquationSolver.ID_OTHER AnomalyEquationSolver.INDEX_P], [AnomalyEquationSolver.ID_OTHER AnomalyEquationSolver.INDEX_H]};
                {[AnomalyEquationSolver.ID_OTHER AnomalyEquationSolver.INDEX_V], [AnomalyEquationSolver.ID_OTHER AnomalyEquationSolver.INDEX_A]};
                {[AnomalyEquationSolver.ID_OTHER AnomalyEquationSolver.INDEX_H], [AnomalyEquationSolver.ID_OTHER AnomalyEquationSolver.INDEX_W]};
                {[AnomalyEquationSolver.ID_EGO AnomalyEquationSolver.INDEX_P], [AnomalyEquationSolver.ID_OTHER AnomalyEquationSolver.INDEX_P], [AnomalyEquationSolver.ID_EGO AnomalyEquationSolver.INDEX_DIST_DIR]};
            };
        
    end
    
    methods (Static)
        
        function solutionCell = solveEquationSystem(eqSys, eqVal, maxVehicle)
            % eqSys: a cell that specifies the equation system, where each
            % row (cell) is one equation and each element is (ID, Data
            % index).
            % Example: {{[1, 1], [1, 2]}; {[1, 2], [1, 3]}}                         
            
            % Testing Script: AnomalyEquationSolver.solveEquationSystem({{[1, 1], [1, 2]}; {[1, 2], [1, 3]}}, [1 0], 2)
            
            % Note that in the solutionCell,  1 indicates normal, and 0
            % indicates anomalous.
            
            eqCount = size(eqSys,1);
            
            
            % Create table and mark data
            solTable = nan(eqCount, maxVehicle*AnomalyEquationSolver.INDEX_ALL_TYPES);
            
            for eqIndex = 1 : eqCount
                
                eleCount = length(eqSys{eqIndex});
                for eleIndex = 1 : eleCount
                    tempEle = eqSys{eqIndex}{eleIndex};
                    solTable(eqIndex, (tempEle(AnomalyEquationSolver.ELE_INDEX_ID)-1) * AnomalyEquationSolver.INDEX_ALL_TYPES + tempEle(AnomalyEquationSolver.ELE_INDEX_TYPE)) = eqVal(eqIndex);
                end                
                
            end                        
            
            if sum(eqVal == 1) ~= length(eqVal)
                solutionCell = {...
                    AnomalyEquationSolver.getSolutionSpace(solTable, eqVal);
                    AnomalyEquationSolver.getGreedySolutionWithSolutionSpaceFiltering(solTable, eqVal);
                    AnomalyEquationSolver.getFaultyIndividualSolution(solTable, eqVal);
                    AnomalyEquationSolver.getTrustEgoSolutionWithSolutionSpaceFiltering(solTable, eqVal);                    
                    AnomalyEquationSolver.getTrustOtherSolution(solTable, eqVal);
                    AnomalyEquationSolver.getAnomalousEgoAndOthersSolution(solTable, eqVal);
                };
            else
                allTrueArr = true(1,size(solTable,2));
                solutionCell = {...
                    allTrueArr;
                    allTrueArr;
                    allTrueArr;
                    allTrueArr;
                    allTrueArr;
                    allTrueArr;
                };
            end
            
            
            
        end
        
        function solution = getGreedySolution(solTable, eqVal)
            
            % solTable Example 1: solTable = [0 0 nan; nan 1 1];
            % eqVal Example 1: eqVal = [0, 1]';
            
            % solTable Example 2: solTable = [nan nan 0 nan nan nan 0;  nan nan nan 0 0  nan nan; 0  nan nan nan 0  nan nan;  nan nan nan 0 0 0 nan; 0  nan nan nan  nan nan 0];
            % eqVal Example 2: eqVal = [0, 0, 0, 0, 0]';
            
            solution = true(1,size(solTable,2));
            
            eqCount = size(solTable,1);
            
            eqCovered = false(eqCount, 1);
            % Set the normal equations to true
            for eqIndex = 1 : eqCount
                if eqVal(eqIndex) == AnomalyEquationSolver.EQ_VAL_NORMAL
                    eqCovered(eqIndex) = true;
                end
            end
            
            % Create an array for zero count and replace nan with -1
            zeroCount = sum(solTable == 0);
            zeroCount(isnan(zeroCount)) = -1;
            
            tempTable = solTable;
            
            while true
                
                % Find the one with most zeros and mark the data
                [maxValue, maxIndex] = max(zeroCount);
                if maxValue <= 0
                    solution = nan;
                    return;
                end
                
                solution(1,maxIndex) = false;
                indexToChange = (tempTable(:,maxIndex) == 0);
                tempTable(indexToChange,:) = nan;
                eqCovered(indexToChange,1) = true;
                
                % Update zeroCount
                zeroCount = sum(tempTable == 0);
                zeroCount(isnan(zeroCount)) = -1;
                
                if sum(eqCovered) >= eqCount
                    % All equations covered
                    break;
                end
            end
            
            % Sanity Check
            for eqIndex = 1 : eqCount
                if eqVal(eqIndex) == AnomalyEquationSolver.EQ_VAL_NORMAL
                    if sum(solution(~isnan(solTable(eqIndex,:))) == false) > 0
                        solution(~isnan(solTable(eqIndex,:))) = false;
                    end
                end
            end
            
        end
        
        function solution = getGreedySolutionWithSolutionSpaceFiltering(solTable, eqVal)
            
            % solTable Example 1: solTable = [0 0 nan; nan 1 1];
            % eqVal Example 1: eqVal = [0, 1]';
            
            % solTable Example 2: solTable = [nan nan 0 nan nan nan 0; nan nan nan 0 0 nan nan; 0 nan nan nan 0 nan nan; nan nan nan 0 0 0 nan; 0 nan nan nan  nan nan 0];
            % eqVal Example 2: eqVal = [0, 0, 0, 0, 0]';
            
            % solTable Example 3: solTable = [nan nan 0 nan nan nan 0; nan nan nan 0 0 nan nan; 0 nan nan nan 0 nan nan; nan nan nan 0 0 0 nan; 0 nan nan nan nan nan 0; 1 1 1 1 1 1 1];
            % eqVal Example 3: eqVal = [0, 0, 0, 0, 0, 1]';
            
            solution = true(1,size(solTable,2));
            performUnconstrainedGreedySolution = false;
            
            eqCount = size(solTable,1);
            
            eqCovered = false(eqCount, 1);
            % Set the normal equations to true
            for eqIndex = 1 : eqCount
                if eqVal(eqIndex) == AnomalyEquationSolver.EQ_VAL_NORMAL
                    eqCovered(eqIndex) = true;
                end
            end
            
            % Create an array for zero count and replace nan with -1
            zeroCount = sum(solTable == 0);
            zeroCount(isnan(zeroCount)) = -1;
            zeroCount(sum(solTable == 1) > 0) = 0;
            
            tempTable = solTable;
            
            while true
                
                % Find the one with most zeros and mark the data
                [maxValue, maxIndex] = max(zeroCount);
                if maxValue <= 0
                    performUnconstrainedGreedySolution = true;
                    break;
                end
                
                solution(1,maxIndex) = false;
                indexToChange = (tempTable(:,maxIndex) == 0);
                tempTable(indexToChange,:) = nan;
                eqCovered(indexToChange,1) = true;
                
                % Update zeroCount
                zeroCount = sum(tempTable == 0);
                zeroCount(isnan(zeroCount)) = -1;
                zeroCount(sum(tempTable == 1) > 0) = 0;
                
                if sum(eqCovered) >= eqCount
                    % All equations covered
                    break;
                end
            end
            
            if performUnconstrainedGreedySolution == false
                % Sanity Check
                for eqIndex = 1 : eqCount
                    if eqVal(eqIndex) == AnomalyEquationSolver.EQ_VAL_NORMAL
                        if sum(solution(~isnan(solTable(eqIndex,:))) == false) > 0
                            solution(~isnan(solTable(eqIndex,:))) = false;
                        end
                    end
                end
            else
                solution = AnomalyEquationSolver.getGreedySolution(solTable, eqVal);
            end
            
        end
        
        function solution = getTrustEgoSolution(solTable, eqVal)
            % solTable Example 1: solTable = [0 0 nan; nan 1 1];
            % eqVal Example 1: eqVal = [0, 1]';
            
            % solTable Example 2: solTable = [nan nan 0 nan nan nan 0;  nan nan nan 0 0  nan nan; 0  nan nan nan 0  nan nan;  nan nan nan 0 0 0 nan; 0  nan nan nan  nan nan 0];
            % eqVal Example 2: eqVal = [0, 0, 0, 0, 0]';
            
            solution = true(1,size(solTable,2));
            
            eqCount = size(solTable,1);
            
            eqCovered = false(eqCount, 1);
            % Set the normal equations to true
            for eqIndex = 1 : eqCount
                if eqVal(eqIndex) == AnomalyEquationSolver.EQ_VAL_NORMAL
                    eqCovered(eqIndex) = true;
                end
            end
            
            % Create an array for zero count and replace nan with -1
            zeroCount = sum(solTable == 0);
            zeroCount(isnan(zeroCount)) = -1;
            zeroCount(:,1:AnomalyEquationSolver.INDEX_ALL_TYPES) = -1;
            
            tempTable = solTable;
            
            while true
                
                % Find the one with most zeros and mark the data
                [maxValue, maxIndex] = max(zeroCount);
                
                if maxValue <= 0
                    solution = nan;
                    return;
                end
                
                solution(1,maxIndex) = false;
                indexToChange = (tempTable(:,maxIndex) == 0);
                tempTable(indexToChange,:) = nan;
                eqCovered(indexToChange,1) = true;
                
                % Update zeroCount
                zeroCount = sum(tempTable == 0);
                zeroCount(isnan(zeroCount)) = -1;
                zeroCount(:,1:AnomalyEquationSolver.INDEX_ALL_TYPES) = -1;
                
                if sum(eqCovered) >= eqCount
                    % All equations covered
                    break;
                end
            end
            
            % Sanity Check
            for eqIndex = 1 : eqCount
                if eqVal(eqIndex) == AnomalyEquationSolver.EQ_VAL_NORMAL
                    if sum(solution(~isnan(solTable(eqIndex,:))) == false) > 0
                        solution(~isnan(solTable(eqIndex,:))) = false;
                    end
                end
            end
        end
        
        function solution = getTrustEgoSolutionWithSolutionSpaceFiltering(solTable, eqVal)
            % solTable Example 1: solTable = [0 0 nan; nan 1 1];
            % eqVal Example 1: eqVal = [0, 1]';
            
            % solTable Example 2: solTable = [nan nan 0 nan nan nan 0;  nan nan nan 0 0  nan nan; 0  nan nan nan 0  nan nan;  nan nan nan 0 0 0 nan; 0  nan nan nan  nan nan 0];
            % eqVal Example 2: eqVal = [0, 0, 0, 0, 0]';
            
            solution = true(1,size(solTable,2));
            performUnconstrainedGreedySolution = false;
            
            eqCount = size(solTable,1);
            
            eqCovered = false(eqCount, 1);
            % Set the normal equations to true
            for eqIndex = 1 : eqCount
                if eqVal(eqIndex) == AnomalyEquationSolver.EQ_VAL_NORMAL
                    eqCovered(eqIndex) = true;
                end
            end
            
            % Create an array for zero count and replace nan with -1
            zeroCount = sum(solTable == 0);
            zeroCount(isnan(zeroCount)) = -1;
            zeroCount(:,1:AnomalyEquationSolver.INDEX_ALL_TYPES) = -1;
            zeroCount(sum(solTable == 1) > 0) = 0;
            
            tempTable = solTable;
            
            while true
                
                % Find the one with most zeros and mark the data
                [maxValue, maxIndex] = max(zeroCount);
                
                if maxValue <= 0
                    %performUnconstrainedGreedySolution = true;
                    solution = nan;
                    return;
                end
                
                solution(1,maxIndex) = false;
                indexToChange = (tempTable(:,maxIndex) == 0);
                tempTable(indexToChange,:) = nan;
                eqCovered(indexToChange,1) = true;
                
                % Update zeroCount
                zeroCount = sum(tempTable == 0);
                zeroCount(isnan(zeroCount)) = -1;
                zeroCount(:,1:AnomalyEquationSolver.INDEX_ALL_TYPES) = -1;
                zeroCount(sum(tempTable == 1) > 0) = 0;
                
                if sum(eqCovered) >= eqCount
                    % All equations covered
                    break;
                end
            end
            
            if performUnconstrainedGreedySolution == false
                % Sanity Check
                for eqIndex = 1 : eqCount
                    if eqVal(eqIndex) == AnomalyEquationSolver.EQ_VAL_NORMAL
                        if sum(solution(~isnan(solTable(eqIndex,:))) == false) > 0
                            solution(~isnan(solTable(eqIndex,:))) = false;
                        end
                    end
                end
            else
                solution = AnomalyEquationSolver.getTrustEgoSolution(solTable, eqVal);
            end
        end
        
        function solution = getTrustOtherSolution(solTable, eqVal)
            % solTable Example 1: solTable = [0 0 nan; nan 1 1];
            % eqVal Example 1: eqVal = [0, 1]';
            
            % solTable Example 2: solTable = [nan nan 0 nan nan nan 0;  nan nan nan 0 0  nan nan; 0  nan nan nan 0  nan nan;  nan nan nan 0 0 0 nan; 0  nan nan nan  nan nan 0];
            % eqVal Example 2: eqVal = [0, 0, 0, 0, 0]';
            
            solution = true(1,size(solTable,2));
            
            eqCount = size(solTable,1);
            
            eqCovered = false(eqCount, 1);
            % Set the normal equations to true
            for eqIndex = 1 : eqCount
                if eqVal(eqIndex) == AnomalyEquationSolver.EQ_VAL_NORMAL
                    eqCovered(eqIndex) = true;
                end
            end
            
            % Create an array for zero count and replace nan with -1
            zeroCount = sum(solTable == 0);
            zeroCount(isnan(zeroCount)) = -1;
            zeroCount(:,AnomalyEquationSolver.INDEX_ALL_TYPES+1:end) = -1;
            
            tempTable = solTable;
            
            while true
                
                % Find the one with most zeros and mark the data
                [maxValue, maxIndex] = max(zeroCount);
                
                if maxValue <= 0
                    solution = nan;
                    return;
                end
                
                solution(1,maxIndex) = false;
                indexToChange = (tempTable(:,maxIndex) == 0);
                tempTable(indexToChange,:) = nan;
                eqCovered(indexToChange,1) = true;
                
                % Update zeroCount
                zeroCount = sum(tempTable == 0);
                zeroCount(isnan(zeroCount)) = -1;
                zeroCount(:,AnomalyEquationSolver.INDEX_ALL_TYPES+1:end) = -1;
                
                if sum(eqCovered) >= eqCount
                    % All equations covered
                    break;
                end
            end
            
            % Sanity Check
            for eqIndex = 1 : eqCount
                if eqVal(eqIndex) == AnomalyEquationSolver.EQ_VAL_NORMAL
                    if sum(solution(~isnan(solTable(eqIndex,:))) == false) > 0
                        solution(~isnan(solTable(eqIndex,:))) = false;
                    end
                end
            end
        end
        
        function solution = getAnomalousEgoAndOthersSolution(solTable, eqVal)
            % solTable Example 1: solTable = [0 0 nan; nan 1 1];
            % eqVal Example 1: eqVal = [0, 1]';
            
            % solTable Example 2: solTable = [nan nan 0 nan nan nan 0;  nan nan nan 0 0  nan nan; 0  nan nan nan 0  nan nan;  nan nan nan 0 0 0 nan; 0  nan nan nan  nan nan 0];
            % eqVal Example 2: eqVal = [0, 0, 0, 0, 0]';
            
            solution = true(1,size(solTable,2));
            
            eqCount = size(solTable,1);
            
            eqCovered = false(eqCount, 1);
            % Set the normal equations to true
            for eqIndex = 1 : eqCount
                if eqVal(eqIndex) == AnomalyEquationSolver.EQ_VAL_NORMAL
                    eqCovered(eqIndex) = true;
                end
            end
            
            % Create an array for zero count and replace nan with -1
            zeroCount = sum(solTable == 0);
            zeroCount(isnan(zeroCount)) = -1;
            zeroCount(:,AnomalyEquationSolver.INDEX_ALL_TYPES+1:end) = -1;
            
            tempTable = solTable;
            
            while true
                
                % Find the one with most zeros and mark the data
                [maxValue, maxIndex] = max(zeroCount);
                
                if maxValue <= 0
                    break;
                end
                
                solution(1,maxIndex) = false;
                indexToChange = (tempTable(:,maxIndex) == 0);
                tempTable(indexToChange,:) = nan;
                eqCovered(indexToChange,1) = true;
                
                % Update zeroCount
                zeroCount = sum(tempTable == 0);
                zeroCount(isnan(zeroCount)) = -1;
                zeroCount(:,AnomalyEquationSolver.INDEX_ALL_TYPES+1:end) = -1;
                
                if sum(eqCovered) >= eqCount
                    % All equations covered
                    break;
                end
            end
            
            % Update zeroCount
            zeroCount = sum(tempTable == 0);
            zeroCount(isnan(zeroCount)) = -1;
            
            if sum(solution(1,1:AnomalyEquationSolver.INDEX_ALL_TYPES) == 0) == 0
                solution = nan;
                return
            end
            
            while true
                
                % Find the one with most zeros and mark the data
                [maxValue, maxIndex] = max(zeroCount);
                
                if maxValue <= 0
                    break;
                end
                
                solution(1,maxIndex) = false;
                indexToChange = (tempTable(:,maxIndex) == 0);
                tempTable(indexToChange,:) = nan;
                eqCovered(indexToChange,1) = true;
                
                % Update zeroCount
                zeroCount = sum(tempTable == 0);
                zeroCount(isnan(zeroCount)) = -1;
                %zeroCount(:,AnomalyEquationSolver.INDEX_ALL_TYPES+1:end) = -1;
                
                if sum(eqCovered) >= eqCount
                    % All equations covered
                    break;
                end
            end
            
            % Sanity Check
            for eqIndex = 1 : eqCount
                if eqVal(eqIndex) == AnomalyEquationSolver.EQ_VAL_NORMAL
                    if sum(solution(~isnan(solTable(eqIndex,:))) == false) > 0
                        solution(~isnan(solTable(eqIndex,:))) = false;
                    end
                end
            end
        end
        
        function solution = getFaultyIndividualSolution(solTable, eqVal)
            % TODO
            solution = true(1,size(solTable,2));
            
            eqCount = size(eqVal,1);
            
            % Rank the vehicle according to the anomalous equation
            egoBlockSize = size(AnomalyEquationSolver.DEFAULT_EQ_SYS_EGO_BLOCK,1);
            otherBlockSize = size(AnomalyEquationSolver.DEFAULT_EQ_SYS_OTHER_BLOCK,1);
            numOtherVehicle = ceil((eqCount - egoBlockSize) / otherBlockSize);
            
            anoEqCount = zeros(numOtherVehicle,1);
            
            for vIndex = 1 : numOtherVehicle                
                anoEqCount(vIndex) = sum(eqVal(egoBlockSize + (vIndex - 1) * otherBlockSize + 1 : egoBlockSize + vIndex * otherBlockSize,1) == 0);                
            end
            
            [anoVehicleEqCount, anoVehicleSortedIndex] = sort(anoEqCount, 'descend');
            
            % Perform greedy algorithm according to the anomalous count
                    
            eqCovered = false(eqCount, 1);
            % Set the normal equations to true
            for eqIndex = 1 : eqCount
                if eqVal(eqIndex) == AnomalyEquationSolver.EQ_VAL_NORMAL
                    eqCovered(eqIndex) = true;
                end
            end
            
            vehicleSearchIndex = 1;
            
            % Create an array for zero count and replace nan with -1
            %zeroCount = sum(solTable == 0);
            %zeroCount(isnan(zeroCount)) = -1;            
            % Set the data other than the vehicle with most covered equation to
            % 0
            zeroCount = zeros(1,size(solTable,2));
            indexToFocus = (anoVehicleSortedIndex(vehicleSearchIndex)) * AnomalyEquationSolver.INDEX_ALL_TYPES + 1: (anoVehicleSortedIndex(vehicleSearchIndex)+1) * AnomalyEquationSolver.INDEX_ALL_TYPES;
            zeroCount(1, indexToFocus) = sum(solTable(:, indexToFocus) == 0);
            zeroCount(sum(solTable == 1) > 0) = 0;
            
            tempTable = solTable;
            
            while true
                
                % Find the one with most zeros and mark the data
                [maxValue, maxIndex] = max(zeroCount);
                if maxValue <= 0
                    vehicleSearchIndex = vehicleSearchIndex + 1;
                else
                    solution(1,maxIndex) = false;
                    indexToChange = (tempTable(:,maxIndex) == 0);
                    tempTable(indexToChange,:) = nan;
                    eqCovered(indexToChange,1) = true;
                end                                
                
                if (sum(eqCovered) >= eqCount) || (vehicleSearchIndex > numOtherVehicle)                    
                    % All equations covered
                    break;
                end
                
                % Update zeroCount
                zeroCount = zeros(1,size(tempTable,2));
                indexToFocus = (anoVehicleSortedIndex(vehicleSearchIndex)) * AnomalyEquationSolver.INDEX_ALL_TYPES + 1: (anoVehicleSortedIndex(vehicleSearchIndex) + 1) * AnomalyEquationSolver.INDEX_ALL_TYPES;
                zeroCount(1, indexToFocus) = sum(tempTable(:, indexToFocus) == 0);
                zeroCount(sum(tempTable == 1) > 0) = 0;
                
                
            end
            
            % Perform greedy solution to ego vehicle
            zeroCount = sum(tempTable == 0);
            zeroCount(isnan(zeroCount)) = -1;
            
            while true
                
                % Find the one with most zeros and mark the data
                [maxValue, maxIndex] = max(zeroCount);
                if maxValue <= 0
                    break;
                end
                
                solution(1,maxIndex) = false;
                indexToChange = (tempTable(:,maxIndex) == 0);
                tempTable(indexToChange,:) = nan;
                eqCovered(indexToChange,1) = true;
                
                % Update zeroCount
                zeroCount = sum(tempTable == 0);
                zeroCount(isnan(zeroCount)) = -1;
                
                if sum(eqCovered) >= eqCount
                    % All equations covered
                    break;
                end
            end
            
            % Sanity Check
            for eqIndex = 1 : eqCount
                if eqVal(eqIndex) == AnomalyEquationSolver.EQ_VAL_NORMAL
                    if sum(solution(~isnan(solTable(eqIndex,:))) == false) > 0
                        solution(~isnan(solTable(eqIndex,:))) = false;
                    end
                end
            end
            
        end
        
        function solution = getSolutionSpace(solTable, eqVal)
            
            % solTable Example 1: solTable = [0 0 nan; nan 1 1];
            % eqVal Example 1: eqVal = [0, 1]';
            
            % solTable Example 2: solTable = [nan nan 0 nan nan nan 0; nan nan nan 0 0 nan nan; 0 nan nan nan 0 nan nan; nan nan nan 0 0 0 nan; 0 nan nan nan  nan nan 0];
            % eqVal Example 2: eqVal = [0, 0, 0, 0, 0]';
            
            % solTable Example 3: solTable = [nan nan 0 nan nan nan 0; nan nan nan 0 0 nan nan; 0 nan nan nan 0 nan nan; nan nan nan 0 0 0 nan; 0 nan nan nan nan nan 0; 1 1 1 1 1 1 1];
            % eqVal Example 3: eqVal = [0, 0, 0, 0, 0, 1]';
            
            solution = true(1,size(solTable,2));
            performUnconstrainedGreedySolution = false;
            
            eqCount = size(solTable,1);
            
            eqCovered = false(eqCount, 1);
            % Set the normal equations to true
            for eqIndex = 1 : eqCount
                if eqVal(eqIndex) == AnomalyEquationSolver.EQ_VAL_NORMAL
                    eqCovered(eqIndex) = true;
                end
            end
            
            % Create an array for zero count and replace nan with -1
            zeroCount = sum(solTable == 0);
            zeroCount(isnan(zeroCount)) = -1;
            zeroCount(sum(solTable == 1) > 0) = 0;
            
%            tempTable = solTable;

            solution(1,zeroCount>0) = false;
            
%             while true
%                 
%                 % Find the one with most zeros and mark the data
%                 [maxValue, maxIndex] = max(zeroCount);
%                 if maxValue <= 0
%                     performUnconstrainedGreedySolution = true;
%                     break;
%                 end
%                 
%                 solution(1,maxIndex) = false;
%                 indexToChange = (tempTable(:,maxIndex) == 0);
%                 tempTable(indexToChange,:) = nan;
%                 eqCovered(indexToChange,1) = true;
%                 
%                 % Update zeroCount
%                 zeroCount = sum(tempTable == 0);
%                 zeroCount(isnan(zeroCount)) = -1;
%                 zeroCount(sum(tempTable == 1) > 0) = 0;
%                 
%                 if sum(eqCovered) >= eqCount
%                     % All equations covered
%                     break;
%                 end
%             end
            
            if performUnconstrainedGreedySolution == false
                % Sanity Check
                for eqIndex = 1 : eqCount
                    if eqVal(eqIndex) == AnomalyEquationSolver.EQ_VAL_NORMAL
                        if sum(solution(~isnan(solTable(eqIndex,:))) == false) > 0
                            solution(~isnan(solTable(eqIndex,:))) = false;
                        end
                    end
                end
            else
                solution = AnomalyEquationSolver.getGreedySolution(solTable, eqVal);
            end
            
        end
       
        function eqSys = genEqSys(numOtherVehicle)
            
            otherEqCount = size(AnomalyEquationSolver.DEFAULT_EQ_SYS_OTHER_BLOCK,1);
            
            otherEqSys = cell(otherEqCount * numOtherVehicle, 1);
            
            if numOtherVehicle == 0
                eqSys = AnomalyEquationSolver.DEFAULT_EQ_SYS_EGO_BLOCK;
                return;
            end
            
            eqSys = cell(numOtherVehicle+1, 1);
            eqSys{1} = AnomalyEquationSolver.DEFAULT_EQ_SYS_EGO_BLOCK{:,1};

            for vIndex = 1 : numOtherVehicle                
                for eqIndex = 1 : otherEqCount                    
                    tempEq = AnomalyEquationSolver.DEFAULT_EQ_SYS_OTHER_BLOCK{eqIndex};                    
                    for eleIndex = 1 : length(tempEq)
                        if(tempEq{eleIndex}(1) == AnomalyEquationSolver.ID_OTHER)
                            tempEq{eleIndex}(1) = vIndex + AnomalyEquationSolver.ID_EGO;
                        end
                    end
                    eqSys{vIndex+1,eqIndex} = tempEq;
                    % otherEqSys{(vIndex-1) * otherEqCount + eqIndex,1} = tempEq;                    
                end
                % eqSys{vIndex+1} = otherEqSys{(vIndex-1) * otherEqCount+1:(vIndex-1) * otherEqCount+otherEqCount,1};
            end
            
            

            % eqSys = [ AnomalyEquationSolver.DEFAULT_EQ_SYS_EGO_BLOCK{:,1}; otherEqSys{:,1}];
            
        end
        
        function [solTable, eqVal, eqSys] = genTestCase(anoData, numOtherVehicle, cmdStr)
            
            % anoData: Array of anomalous data. Each row is the [vehicle
            % ID, data ID]
            % Example: anoData = [0 1; 0 2];
            
            eqSys = AnomalyEquationSolver.genEqSys(numOtherVehicle);
            
            % Create table and mark data
            eqCount = size(eqSys,1);
            solTable = nan(eqCount, (numOtherVehicle+1)*AnomalyEquationSolver.INDEX_ALL_TYPES);
            eqVal = ones(eqCount,1);
            
            for eqIndex = 1 : eqCount
                
                eleCount = length(eqSys{eqIndex});
                eqAnoCount = 0;
                for eleIndex = 1 : eleCount
                    tempEle = eqSys{eqIndex}{eleIndex};
                    solTable(eqIndex, tempEle(AnomalyEquationSolver.ELE_INDEX_ID) * AnomalyEquationSolver.INDEX_ALL_TYPES + tempEle(AnomalyEquationSolver.ELE_INDEX_TYPE)) = 1;
                    
                    if ismember(tempEle, anoData, 'rows')
                        eqVal(eqIndex) = 0;
                        if strcmpi(cmdStr, 'weak')
                            eqAnoCount = eqAnoCount + 1;
                            if eqAnoCount >= eleCount
                                eqVal(eqIndex) = 1;
                            end
                        end
                    end
                    
                end                
                solTable(eqIndex,:) = solTable(eqIndex,:) * eqVal(eqIndex);
            end
        end
        
        function anoDataCell = genAnoDataCombination(numOtherVehicle, anoCount)
            
            totalDataType = (numOtherVehicle + 1) * AnomalyEquationSolver.INDEX_ALL_TYPES;
            
            combArr = combnk(1:totalDataType, anoCount);
            
            combCount = size(combArr,1);
            
            anoDataCell = cell(combCount,1);
            
            for cIndex = 1 : combCount
                                
                tempCell = cell(1,anoCount);
                for aIndex = 1 : anoCount
                    tempCell{1,aIndex} = AnomalyEquationSolver.convertToIdType(combArr(cIndex,aIndex));
                end
                anoDataCell{cIndex} = tempCell;
            end
            
        end
        
        function dataIdTypeArr = convertToIdType(colIndex)
            
            dataCount = length(colIndex);
            
            dataIdTypeArr = zeros(dataCount,2);
            
            for dIndex = 1 : dataCount
                
                dataIdTypeArr(dIndex, :) = [floor((colIndex(dIndex)-1)/AnomalyEquationSolver.INDEX_ALL_TYPES), mod(colIndex(dIndex)-1, AnomalyEquationSolver.INDEX_ALL_TYPES) + 1];
                
            end
            
        end
        
        function colIndex = convertToColIndex(dataIdTypeArr)
            
            dataCount = size(dataIdTypeArr,1);
            colIndex = zeros(dataCount,1);
            
            for dIndex = 1 : dataCount
                
                colIndex(dIndex) = dataIdTypeArr(dIndex,1) * AnomalyEquationSolver.INDEX_ALL_TYPES + dataIdTypeArr(dIndex,2);
                
            end
            
        end
        
        function [dataSummary, sourceSymmary] = verifyResult(gtAnoIdType, solution)                                                
            
            solutionReversed = ~solution;
            gtColIndex = AnomalyEquationSolver.convertToColIndex(gtAnoIdType);
            
            gtArr = zeros(1,size(solutionReversed,2));
            gtArr(gtColIndex) = 1;
            
            % Source
            totalVehicle = size(solution,2) / AnomalyEquationSolver.INDEX_ALL_TYPES;
            idArr = unique(gtAnoIdType(:,1));
            idFlagArr = zeros(1,totalVehicle);
            idFlagArr(1,idArr + 1) = 1;
            solIdFlagArr = sum(reshape(solutionReversed, AnomalyEquationSolver.INDEX_ALL_TYPES, [])) > 0;
            
            sTP = sum((idFlagArr == 1) & (solIdFlagArr == idFlagArr));
            sTN = sum((idFlagArr == 0) & (solIdFlagArr == idFlagArr));
            
            sFP = sum((idFlagArr == 0) & (solIdFlagArr ~= idFlagArr));
            sFN = sum((idFlagArr == 1) & (solIdFlagArr ~= idFlagArr));
            
            sP = sum((idFlagArr == 1));
            sN = sum((idFlagArr == 0));
            
            sTPR = sTP / sP;
            sFPR = sFP / sN;
            
            if sTPR < 1
                disp('')
            end
            
            % Remove Dist_Dir from other vehicle
            skipCount = AnomalyEquationSolver.INDEX_DIST_DIR;
            skipIndices = skipCount*2:skipCount:size(solutionReversed,2);
            solutionReversed(skipIndices) = [];
            gtArr(skipIndices) = [];
            
            % Individual Data
            TP = sum((gtArr == 1) & (solutionReversed == gtArr));
            TN = sum((gtArr == 0) & (solutionReversed == gtArr));
            
            FP = sum((gtArr == 0) & (solutionReversed ~= gtArr));
            FN = sum((gtArr == 1) & (solutionReversed ~= gtArr));
            
            P = sum((gtArr == 1));
            N = sum((gtArr == 0));
            
            TPR = TP / P;
            FPR = FP / N;
            
            
            
            dataSummary = [TPR, FPR, TP, TN, FP, FN, P, N];
            sourceSymmary = [sTPR, sFPR, sTP, sTN, sFP, sFN, sP, sN];
        end
        
    end
    
end

