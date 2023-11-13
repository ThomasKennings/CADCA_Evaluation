classdef EvaluationCADCA < matlab.mixin.Copyable
        
    properties
        
    end
    
    methods
        
        function obj = EvaluationCADCA()
            
        end
        
        function result = performSingleTestCase(obj, scenario, manipulatedTable, controlInputs, controlInsertIndex, trueTable)
            
            totalStep = size(manipulatedTable,3);
            
            cadca = CADCA(scenario);
            result = zeros(totalStep,1);
            
            for tIndex = 3 : totalStep
                %oldStateTable = manipulatedTable(:,:,tIndex-1);
                oldStateTable = trueTable(:,:,tIndex-1);
                currentStateTable = manipulatedTable(:,:,tIndex);
                
                if tIndex >= controlInsertIndex
                    tempCI = controlInputs;
                else
                    tempCI = nan(2,2);
                end
                
                [~, result(tIndex,1)] = cadca.performCompleteProcess(currentStateTable, oldStateTable, tempCI);                
            end                        
            
        end
        
        function result = performSingleTestCaseWithEstimationUpdate(obj, scenario, manipulatedTable, controlInputs, controlInsertIndex, trueTable)
            
            totalStep = size(manipulatedTable,3);
            
            cadca = CADCA(scenario);
            result = zeros(totalStep,1);
            
            correctedTable = trueTable(:,:,2);
            for tIndex = 3 : totalStep
                %oldStateTable = manipulatedTable(:,:,tIndex-1);
                oldStateTable = correctedTable;
                currentStateTable = manipulatedTable(:,:,tIndex);
                
                if tIndex >= controlInsertIndex
                    tempCI = controlInputs;
                else
                    tempCI = nan(2,2);
                end
                
                [~, result(tIndex,1), correctedTable] = cadca.performCompleteProcess(currentStateTable, oldStateTable, tempCI);                
            end                        
            
        end
                
    end
    
    methods (Static)
        
        function [missCount, fpCount, delayCount, correctCount, totalCount, safeCount, unsafeCount] = summarizePerformance(gtResult, testResult, safeControlIndex)
            
            testResult = testResult(gtResult ~= 0);
            gtResult = gtResult(gtResult ~= 0);
            
            
            correctCount = sum(gtResult == testResult);
            missCount = sum(gtResult == safeControlIndex) - sum((testResult == safeControlIndex) & (gtResult == safeControlIndex));
            fpCount   = sum((gtResult ~= safeControlIndex) & (testResult == safeControlIndex));
            delayCount = find(testResult == safeControlIndex,1) - find(gtResult == safeControlIndex,1);
            if(delayCount < 0)
                delayCount = 0;
            elseif isempty(delayCount)
                delayCount = nan;
            end
            
            totalCount = length(gtResult);
            
            safeCount = sum(gtResult == safeControlIndex);
            unsafeCount = sum((gtResult ~= safeControlIndex) & (gtResult ~= 0));
            
        end
        
        function result = pureRiskAssessmentChoice(scenario, controlInput, conciseTable)
            
            ra = RiskAssessment(scenario);
            tccArr = ra.getGroundTruthTTCEstimation(controlInput, conciseTable);
            
            result = ones(1,size(tccArr,2));
            
            result(1, (tccArr(2,:) > tccArr(1,:)) | (isnan(tccArr(2,:)) & ~isnan(tccArr(1,:)))) = 2;
            
            result = result';
            
        end
        
    end
    
    
end

