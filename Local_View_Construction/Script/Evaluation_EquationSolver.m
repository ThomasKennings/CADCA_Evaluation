%% Parameter Setup

numOtherVehicle = 1;

maxNumAnomaly = (1 + numOtherVehicle) * AnomalyEquationSolver.INDEX_ALL_TYPES;

anoCountStart = 1;
anoCountEnd   = floor(maxNumAnomaly / (numOtherVehicle+1));
%anoCountEnd  =  2; 

%testCaseCmd = '';
testCaseCmd = 'weak';

NO_EGO_FAULTY_FLAG = false;
NO_OTHER_FAULTY_FLAG = true;
MULTI_SOURCE_ONLY = false;

DEBUG_FLAG = true;

%%  

greedyResult = zeros(anoCountEnd,8);
greedySourceResult = zeros(anoCountEnd,8);
greedySuccCount = zeros(anoCountEnd,1);
greedyFailCount = zeros(anoCountEnd,1);

egoResult = zeros(anoCountEnd,8);
egoSourceResult = zeros(anoCountEnd,8);
egoSuccCount = zeros(anoCountEnd,1);
egoFailCount = zeros(anoCountEnd,1);

fIResult = zeros(anoCountEnd,8);
fISourceResult = zeros(anoCountEnd,8);
fISuccCount = zeros(anoCountEnd,1);
fIFailCount = zeros(anoCountEnd,1);

ssResult = zeros(anoCountEnd,8);
ssSourceResult = zeros(anoCountEnd,8);
ssSuccCount = zeros(anoCountEnd,1);
ssFailCount = zeros(anoCountEnd,1);

%%

for aIndex = anoCountStart : anoCountEnd 
    
    % Generate All Combinations
    anoDataCell = AnomalyEquationSolver.genAnoDataCombination(numOtherVehicle, aIndex);
    
    tempCaseCount = size(anoDataCell,1);
    
    for cIndex = 1 : tempCaseCount
        
        tempCase = anoDataCell{cIndex};
        
        % Remove unnecessary ones
        skip = false;
        for rIndex = 1 : aIndex            
            if(tempCase{rIndex}(AnomalyEquationSolver.ELE_INDEX_TYPE) == AnomalyEquationSolver.INDEX_DIST_DIR) && (tempCase{rIndex}(AnomalyEquationSolver.ELE_INDEX_ID) ~= AnomalyEquationSolver.ID_EGO)
                skip = true;
                break;
            end
            
            if NO_EGO_FAULTY_FLAG == true                
                if (tempCase{rIndex}(AnomalyEquationSolver.ELE_INDEX_ID) == AnomalyEquationSolver.ID_EGO)
                    skip = true;
                    break;
                end                
            end
            
            if NO_OTHER_FAULTY_FLAG == true                
                if (tempCase{rIndex}(AnomalyEquationSolver.ELE_INDEX_ID) ~= AnomalyEquationSolver.ID_EGO)
                    skip = true;
                    break;
                end                
            end
            
            if MULTI_SOURCE_ONLY == true
                tempCaseArr = cell2mat(tempCase');                
                if length(unique(tempCaseArr(:,AnomalyEquationSolver.ELE_INDEX_ID))) < 2
                    skip = true;
                    break;
                end
            end            
            
        end
        if skip == true
            continue
        end

        tempCase = cell2mat(tempCase');
        
        % Perform algorithms        
        [~, eqVal, eqSys] = AnomalyEquationSolver.genTestCase(tempCase, numOtherVehicle, testCaseCmd);
        solCell = AnomalyEquationSolver.solveEquationSystem(eqSys, eqVal, numOtherVehicle + 1);                

        % Record Results
        if ~isnan(solCell{1})
            [tempGreedyResult, tempGreedySourceResult] = AnomalyEquationSolver.verifyResult(tempCase, solCell{1});
            greedyResult(aIndex,:) = greedyResult(aIndex,:) + tempGreedyResult;
            greedySourceResult(aIndex,:) = greedySourceResult(aIndex,:) + tempGreedySourceResult;
            greedySuccCount(aIndex,:) = greedySuccCount(aIndex,:) + 1;
            
            % if (tempGreedyResult(1) ~= 1) && (DEBUG_FLAG == true)
            %     tempCase
            % end
        else
            greedyFailCount(aIndex,:) = greedyFailCount(aIndex,:) + 1;
        end
        
        if ~isnan(solCell{2})
            [tempEgoResult, tempEgoSourceResult] = AnomalyEquationSolver.verifyResult(tempCase, solCell{2});
            egoResult(aIndex,:) = egoResult(aIndex,:) + tempEgoResult;
            egoSourceResult(aIndex,:) = egoSourceResult(aIndex,:) + tempEgoSourceResult;
            egoSuccCount(aIndex,:) = egoSuccCount(aIndex,:) + 1;
        else
            egoFailCount(aIndex,:) = egoFailCount(aIndex,:) + 1;
        end
        
        if ~isnan(solCell{3})
            [tempFIResult, tempFISourceResult] = AnomalyEquationSolver.verifyResult(tempCase, solCell{3});
            fIResult(aIndex,:) = fIResult(aIndex,:) + tempFIResult;
            fISourceResult(aIndex,:) = fISourceResult(aIndex,:) + tempFISourceResult;
            fISuccCount(aIndex,:) = fISuccCount(aIndex,:) + 1;
        else
            fIFailCount(aIndex,:) = fIFailCount(aIndex,:) + 1;
        end
        
        if ~isnan(solCell{4})
            [tempSSResult, tempSSSourceResult] = AnomalyEquationSolver.verifyResult(tempCase, solCell{4});
            ssResult(aIndex,:) = ssResult(aIndex,:) + tempSSResult;
            ssSourceResult(aIndex,:) = ssSourceResult(aIndex,:) + tempSSSourceResult;
            ssSuccCount(aIndex,:) = ssSuccCount(aIndex,:) + 1;
            
            if tempSSSourceResult(1) < 1
                tempCase
            end
            
        else
            ssFailCount(aIndex,:) = ssFailCount(aIndex,:) + 1;
        end
                        
    end
    
end

%% Display Results

disp(' ')
disp('=============================================================')
disp(' ')
for aIndex = anoCountStart : anoCountEnd
    fprintf('Number of Anomalies: %d\n', aIndex)
    fprintf('Greedy Results           : TPR = %5.4f, FPR = %5.4f\n', greedyResult(aIndex,1) / greedySuccCount(aIndex,1), greedyResult(aIndex,2) / greedySuccCount(aIndex,1));
    fprintf('Ego Results              : TPR = %5.4f, FPR = %5.4f\n', egoResult(aIndex,1) / egoSuccCount(aIndex,1), egoResult(aIndex,2) / egoSuccCount(aIndex,1));
    fprintf('Faulty Individual Results: TPR = %5.4f, FPR = %5.4f\n', fIResult(aIndex,1) / fISuccCount(aIndex,1), fIResult(aIndex,2) / fISuccCount(aIndex,1));
    fprintf('Solution Space Results   : TPR = %5.4f, FPR = %5.4f\n\n', ssResult(aIndex,1) / ssSuccCount(aIndex,1), ssResult(aIndex,2) / ssSuccCount(aIndex,1));
end
disp('=============================================================')


%% Record Data

methodCount = 4;
rocMat = zeros(methodCount,anoCountEnd,5);

for aIndex = 1 : anoCountEnd    
    rocMat(1,aIndex,:) = [greedyResult(aIndex,1) / greedySuccCount(aIndex,1), greedyResult(aIndex,2) / greedySuccCount(aIndex,1), greedySuccCount(aIndex,1)/(greedySuccCount(aIndex,1)+greedyFailCount(aIndex,1)), greedySourceResult(aIndex,1) / greedySuccCount(aIndex,1), greedySourceResult(aIndex,2) / greedySuccCount(aIndex,1)];
    rocMat(2,aIndex,:) = [egoResult(aIndex,1) / egoSuccCount(aIndex,1), egoResult(aIndex,2) / egoSuccCount(aIndex,1), egoSuccCount(aIndex,1)/(egoSuccCount(aIndex,1) + egoFailCount(aIndex,1)), egoSourceResult(aIndex,1) / egoSuccCount(aIndex,1), egoSourceResult(aIndex,2) / egoSuccCount(aIndex,1)];
    rocMat(3,aIndex,:) = [fIResult(aIndex,1) / fISuccCount(aIndex,1), fIResult(aIndex,2) / fISuccCount(aIndex,1), fISuccCount(aIndex,1)/(fISuccCount(aIndex,1)+fIFailCount(aIndex,1)), fISourceResult(aIndex,1) / fISuccCount(aIndex,1), fISourceResult(aIndex,2) / fISuccCount(aIndex,1)];
    rocMat(4,aIndex,:) = [ssResult(aIndex,1) / ssSuccCount(aIndex,1), ssResult(aIndex,2) / ssSuccCount(aIndex,1), ssSuccCount(aIndex,1)/(ssSuccCount(aIndex,1)+ssFailCount(aIndex,1)), ssSourceResult(aIndex,1) / ssSuccCount(aIndex,1), ssSourceResult(aIndex,2) / ssSuccCount(aIndex,1)];    
end

%% Plot Data

% Plot TPR
figure

totalSubFig = 5;

subplot(1,totalSubFig,1)
hold on
bar(rocMat(:,:,1)')
hold off
ylim([0 1])
xlim([0 anoCountEnd+1])
title('TPR')
grid on
legend('Greedy', 'Trust Ego', 'Faulty Individual', 'Solution Space')

% Plot FPR
subplot(1,totalSubFig,2)
hold on
bar(rocMat(:,:,2)')
hold off
ylim([0 1])
xlim([0 anoCountEnd+1])
title('FPR')
grid on
legend('Greedy', 'Trust Ego', 'Faulty Individual', 'Solution Space')

% Plot Success Rate
% Plot FPR
subplot(1,totalSubFig,3)
hold on
bar(rocMat(:,:,3)')
hold off
ylim([0 1])
xlim([0 anoCountEnd+1])
title('Success Rate')
grid on
legend('Greedy', 'Trust Ego', 'Faulty Individual', 'Solution Space')

% Plot Source TPR
subplot(1,totalSubFig,4)
hold on
bar(rocMat(:,:,4)')
hold off
ylim([0 1])
xlim([0 anoCountEnd+1])
title('Source TPR')
grid on
legend('Greedy', 'Trust Ego', 'Faulty Individual', 'Solution Space')

% Plot Source FPR
subplot(1,totalSubFig,5)
hold on
bar(rocMat(:,:,5)')
hold off
ylim([0 1])
xlim([0 anoCountEnd+1])
title('Source FPR')
grid on
legend('Greedy', 'Trust Ego', 'Faulty Individual', 'Solution Space')