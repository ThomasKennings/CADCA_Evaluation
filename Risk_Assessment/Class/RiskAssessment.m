classdef RiskAssessment
        
    properties
        
        vehicleDims; % Col-1: width, Col-2: Length; Row Index = vehicle ID - 1
        
        vehicleHistory;
        
        distThresholdMarginFactor = 1;
        
    end
    
    properties (Constant)
        
        T_REACT_SMOOTH = 4.5; % second
        T_REACT_URGENT = 2.6; % second
        
        ACCELERATION_MAX = 9.8; % m/s
        DECELERATION_MAX = 9.8; % m/s
        
        PREDICTION_TIME_STEP = 0.1; % second
        
        PARALLEL_THRESHOLD = pi / 180 * 10;
        
        DEBUG_FLAG_TTC = false;
        
        CONTROL_INDEX_ACC_X = 1;
        CONTROL_INDEX_ACC_Y = 2;
        %CONTROL_INDEX_STEERING = 3;
        CONTROL_INDEX_TOTAL = 2;
        
    end
    
    methods
        
        function obj = RiskAssessment(actors)
            obj.vehicleDims = obj.getVehicleDims(actors);
            obj.vehicleHistory = AnomalousVehicleHistory(max([SimulationVehicleData.MAX_VEHICLE_COUNT, length(actors)]));
        end
        
        function lambda = rtcl_wrapToPi(obj, lambda)
            q = (lambda < -pi) | (pi < lambda);
            lambda(q) = obj.rtcl_wrapTo2Pi(lambda(q) + pi) - pi;
        end
        
        function lambda = rtcl_wrapTo2Pi(obj, lambda)
            positiveInput = (lambda > 0);
            lambda = mod(lambda, 2*pi);
            lambda((lambda == 0) & positiveInput) = 2*pi;
        end
        
        function [controlIndex, mostLikelyScenario] = riskResultAggregation(obj, isSafeArr, ttcArr, anomalousFlagArr)
            
            % (Input) isSafeArr should be the output from
            % performRiskAssessment function.
            
            % (Input) anomalousFlagArr: where each column indicating the
            % anomalous vehicles output by each algorithm and the rows
            % indicate different vehicle
            
            %%%% Summarize the safety concern for each control input
            
            safetySummary = (sum(isSafeArr == 0 ,2) < 1);
                        
            %%%% Choose one control input if there is one without safety
            %%%% concern --> always put control input with higher priority
            %%%% first            
            
            controlIndex = find(safetySummary,1);
            
            %%%% Use history to determine the most possible scenario if
            %%%% all control inputs have safety concern
            
            %%%% Update Vehicle History
            obj.vehicleHistory.updateTable(anomalousFlagArr);
                               
            mostLikelyScenario = obj.getMostLikelyScenario(anomalousFlagArr);
            
            if isempty(controlIndex)                
                %%%% Find the safe control according to the most possible
                %%%% scenario.
                controlIndex = find(isSafeArr(:,mostLikelyScenario),1);
                
                %%%%%% If no safe option choose the one with largest TTC
                if isempty(controlIndex)
                    [~, controlIndex] = max(ttcArr(:,mostLikelyScenario));
                    
                end                
            end
            
        end
        
        function mostLikelyScenario = getMostLikelyScenario(obj, anomalousFlagArr)
            anomalousWeightArr = obj.vehicleHistory.getLatestWArr();
                
                %%%% Compute inner product for each solution to identify
                %%%% the most possible scenario.
                
                %%%%%% First, normalize the vector
                
                normalizedFlagArr = anomalousFlagArr;
                for aIndex = 1 : size(anomalousFlagArr,2)
                    if sum(normalizedFlagArr(:,aIndex) == 1) > 0
                        normalizedFlagArr(:,aIndex) = normalizedFlagArr(:,aIndex) / norm(double(normalizedFlagArr(:,aIndex)));
                    end
                end
                
                %%%%%% Second, compute the inner product
                
                tempArr = repmat(anomalousWeightArr(1:size(normalizedFlagArr,1),:), [1, size(anomalousFlagArr,2)]) .* normalizedFlagArr;
                innerProductArr = sum(tempArr);
                [~, mostLikelyScenario] = max(innerProductArr(2:end));
                mostLikelyScenario = mostLikelyScenario + 1;
        end
        
        function [isSafeArr, ttcArr] = performRiskAssessment(obj, controlInput, stateTableCell)
                 
            % (Input) controlInput: should be a column array, where each row
            % presents one control input --- the desired acceleration.
            
            % (Input) stateTableCell: should be a row cell, where each column
            % corresponds to one algorithm result.
            
            % (Output) isSafeArr: should be an array specifying whether the
            % controlxstate combination is safe or not. The rowxcolumn
            % presentation should be the same as stateTableCell.
            
            controlCount = size(controlInput,1);
            algCount = size(stateTableCell,2);
            
            isSafeArr = true(controlCount,algCount);
            
            %%%% Update state table with the desired control input and
            %%%% perform risk assessment
            
            stateTableWithControlCell = cell(controlCount,algCount);
            ttcArr = nan(controlCount,algCount);
            for cIndex = 1 : controlCount
                for aIndex = 1 : algCount  
                    if ~isnan(stateTableCell{1,aIndex})     
                        stateTableWithControlCell{cIndex, aIndex} = stateTableCell{1,aIndex};
                        if ~isnan(controlInput(cIndex,obj.CONTROL_INDEX_ACC_X))
                            stateTableWithControlCell{cIndex, aIndex}(1, SimulationVehicleData.TABLE_INDEX_ACC_X:SimulationVehicleData.TABLE_INDEX_ACC_Y) = controlInput(cIndex,obj.CONTROL_INDEX_ACC_X:obj.CONTROL_INDEX_ACC_Y);
                        end
                        [isSafeArr(cIndex, aIndex), ~, tempTTC] = obj.performRiskAssessmentSingleTable(stateTableWithControlCell{cIndex, aIndex});
                        ttcArr(cIndex, aIndex) = min(tempTTC);                                                
                        
                    else
                        stateTableWithControlCell{cIndex, aIndex} = nan;
                        ttcArr(cIndex, aIndex) = nan;
                    end
                end                
            end
            
        end
        
        function [ttcArr, isSafeArr] = getGroundTruthTTCEstimation(obj, controlInput, conciseTable)
                 
            % (Input) controlInput: should be a array, where each row
            % presents one control input --- the desired acceleration.
                      
            % (Input) conciseTable: should be the same as test cases
            
            % (Output) tccArr: row -> control input, column -> time
            
            controlCount = size(controlInput,1);
            timeCount = size(conciseTable,3);
            
            isSafeArr = true(controlCount,timeCount);
            
            %%%% Update state table with the desired control input and
            %%%% perform risk assessment
             
            stateTableWithControlCell = cell(controlCount,timeCount);
            ttcArr = nan(controlCount,timeCount);
            for cIndex = 1 : controlCount
                for tIndex = 1 : timeCount  
                         
                    stateTableWithControlCell{cIndex, tIndex} = conciseTable(:,:,tIndex);
                    if ~isnan(controlInput(cIndex,obj.CONTROL_INDEX_ACC_X))
                        stateTableWithControlCell{cIndex, tIndex}(1, SimulationVehicleData.TABLE_INDEX_ACC_X:SimulationVehicleData.TABLE_INDEX_ACC_Y) = controlInput(cIndex,obj.CONTROL_INDEX_ACC_X:obj.CONTROL_INDEX_ACC_Y);
                    end
                    [isSafeArr(cIndex, tIndex), ~, tempTTC] = obj.performRiskAssessmentSingleTable(stateTableWithControlCell{cIndex, tIndex});
                    ttcArr(cIndex, tIndex) = min(tempTTC);
                    
                end                
            end
            
        end
        
        function [isSafe, isSafeArr, ttcArr] = performRiskAssessmentSingleTable(obj, stateTable)
            
            maxNumVehicles = size(stateTable,1);
            
            isSafe = true;
            isSafeArr = true(maxNumVehicles,1);
            ttcArr = nan(maxNumVehicles,1);
            
            egoRadius = norm(obj.vehicleDims(1,:)) / 4;
            egoHeading = deg2rad(stateTable(1,SimulationVehicleData.TABLE_INDEX_H));
            egoLongUnitVec = [cos(egoHeading) sin(egoHeading)];
            egoLatUnitVec = [-sin(egoHeading) cos(egoHeading)];
            
            for vIndex = 2 : maxNumVehicles
                
                stateRowTemp = stateTable(vIndex,:);
                vIDTemp = stateRowTemp(1, SimulationVehicleData.TABLE_INDEX_VEHICLE_ID);
                
                if vIDTemp < 1
                    continue;
                end

                tempTargetRadius = norm(obj.vehicleDims(vIDTemp-1,:)) / 4;
                distThreshold = (tempTargetRadius + egoRadius) * obj.distThresholdMarginFactor;
                
                [ttcArr(vIDTemp), tu, egoPosArr, otherPosArr] = obj.computeTTCAndTu(stateTable(1,:), stateRowTemp, egoRadius, tempTargetRadius);
                
                if isnan(ttcArr(vIDTemp))
                    continue;
                end
                
                if ttcArr(vIDTemp) < max([obj.T_REACT_SMOOTH, tu + obj.T_REACT_URGENT])
                    isSafeArr(vIDTemp) = false;
                end
                
                %%%% Travelling Parallely
                tempVehicleHeading = deg2rad(stateRowTemp(1, SimulationVehicleData.TABLE_INDEX_H));
                headingDiff = abs(obj.rtcl_wrapToPi(tempVehicleHeading - egoHeading));
                
                if headingDiff > pi/2                    
                    headingDiff = pi - headingDiff;                    
                end
                
                if headingDiff < RiskAssessment.PARALLEL_THRESHOLD
                    posDiffArr = otherPosArr - egoPosArr;
                    
                    longThreshold = obj.vehicleDims(1,2) / 2 + obj.vehicleDims(vIDTemp,2) / 2;
                    latThreshold = obj.vehicleDims(1,1) / 2 + obj.vehicleDims(vIDTemp,1) / 2;
                    
                    longDiff = sum(posDiffArr .* repmat(egoLongUnitVec, [size(posDiffArr,1) 1]), 2);
                    latDiff = sum(posDiffArr .* repmat(egoLatUnitVec, [size(posDiffArr,1) 1]), 2);
                    
                    overlapFlag = sum(((longDiff <= longThreshold) + (latDiff < latThreshold)) >= 2);
                    
                    if overlapFlag == 0
                        isSafeArr(vIDTemp) = true;
                    end
                    
                end
                
            end
            
            
            if sum(isSafeArr == false) > 0
                isSafe = false;
            end                        
            
        end
        
    end
    
    methods (Static)
        
        function [ttc, tu, egoPosArr, otherPosArr] = computeTTCAndTu(egoState, otherState, egoRadius, otherRadius)
                                    
            tu = RiskAssessment.computeMinAvoidTime(egoState, otherState);
            
            maxPredictionTime = max([RiskAssessment.T_REACT_SMOOTH, tu + RiskAssessment.T_REACT_URGENT]);
            
            totalPredictionCount = ceil(maxPredictionTime / RiskAssessment.PREDICTION_TIME_STEP);
            
            egoVel = egoState(1, SimulationVehicleData.TABLE_INDEX_V_X:SimulationVehicleData.TABLE_INDEX_V_Y);            
            egoPos = egoState(1, SimulationVehicleData.TABLE_INDEX_POS_X:SimulationVehicleData.TABLE_INDEX_POS_Y);
            egoAcc = egoState(1, SimulationVehicleData.TABLE_INDEX_ACC_X:SimulationVehicleData.TABLE_INDEX_ACC_Y);
            
            egoHeadingRad = deg2rad(egoState(1, SimulationVehicleData.TABLE_INDEX_H));
            egoYawRateRad = deg2rad(egoState(1, SimulationVehicleData.TABLE_INDEX_YAW_RATE));
            
            egoheadingVec = [cos(egoHeadingRad), sin(egoHeadingRad)];
            egoVel1D = norm(egoVel) * sign(dot(egoVel, egoheadingVec));
            egoAcc1D = norm(egoAcc) * sign(dot(egoAcc, egoheadingVec));
            
            otherVel = otherState(1, SimulationVehicleData.TABLE_INDEX_V_X:SimulationVehicleData.TABLE_INDEX_V_Y);
            otherPos = otherState(1, SimulationVehicleData.TABLE_INDEX_POS_X:SimulationVehicleData.TABLE_INDEX_POS_Y);
            otherAcc = otherState(1, SimulationVehicleData.TABLE_INDEX_ACC_X:SimulationVehicleData.TABLE_INDEX_ACC_Y);
            otherHeadingRad = deg2rad(otherState(1, SimulationVehicleData.TABLE_INDEX_H));
            otherYawRateRad = deg2rad(otherState(1, SimulationVehicleData.TABLE_INDEX_YAW_RATE));
            
            otherheadingVec = [cos(otherHeadingRad), sin(otherHeadingRad)];
            otherVel1D = norm(otherVel) * sign(dot(otherVel, otherheadingVec));
            otherAcc1D = norm(otherAcc) * sign(dot(otherAcc, otherheadingVec));
            
            egoPosArr = zeros(totalPredictionCount+1,2);
            egoVelArr = zeros(totalPredictionCount+1,1);
            egoHeadingArr = zeros(totalPredictionCount+1,1);
            egoPosArr(1,:) = egoPos;
            egoVelArr(1,:) = egoVel1D;
            egoHeadingArr(1,:) = egoHeadingRad;
            
            otherPosArr = zeros(totalPredictionCount+1,2);
            otherVelArr = zeros(totalPredictionCount+1,1);
            otherHeadingArr = zeros(totalPredictionCount+1,1);
            otherPosArr(1,:) = otherPos;
            otherVelArr(1,:) = otherVel1D;
            otherHeadingArr(1,:) = otherHeadingRad;
            
            %%%% Perform position prediction

            % egoPosArr(2:end,:) = RiskAssessment.predictLoc(egoPos, egoVel1D, egoAcc1D, egoHeadingRad, egoYawRateRad, RiskAssessment.PREDICTION_TIME_STEP, totalPredictionCount);
            % otherPosArr(2:end,:) = RiskAssessment.predictLoc(otherPos, otherVel1D, otherAcc1D, otherHeadingRad, otherYawRateRad, RiskAssessment.PREDICTION_TIME_STEP, totalPredictionCount);

            %%%% Perform position, velocity and heading angle prediction
            [egoPosArr(2:end,:), egoVelArr(2:end,:), egoHeadingArr(2:end,:)] = RiskAssessment.predictStatus(egoPos, egoVel1D, egoAcc1D, egoHeadingRad, egoYawRateRad, RiskAssessment.PREDICTION_TIME_STEP, totalPredictionCount);
            [otherPosArr(2:end,:), otherVelArr(2:end,:), otherHeadingArr(2:end,:)] = RiskAssessment.predictStatus(otherPos, otherVel1D, otherAcc1D, otherHeadingRad, otherYawRateRad, RiskAssessment.PREDICTION_TIME_STEP, totalPredictionCount);
            
            %%%% Check if collisions can happen
            distThreshold = otherRadius + egoRadius;
            
            diffArr = egoPosArr - otherPosArr;
            distArr = (diffArr(:,1) .^ 2 + diffArr(:,2) .^ 2) .^ 0.5;
            
            collisionIndex = (find(distArr < distThreshold, 1) - 1);
            if collisionIndex == 0
                collisionIndex = 1;
            end
            
            
            collisionSeverity = (otherRadius / (egoRadius + otherRadius)) * sqrt((egoVelArr(collisionIndex, :)) .^ 2 + (otherVelArr(collisionIndex, :)) .^ 2 - 2 * egoVelArr(collisionIndex, :) .* otherVelArr(collisionIndex, :) .* cos(egoHeadingArr(collisionIndex, :) - otherHeadingArr(collisionIndex, :)));
            % fprintf('Collision severity: %f\n', collisionSeverity);

            ttc = collisionIndex * RiskAssessment.PREDICTION_TIME_STEP / collisionSeverity;
            
            if isempty(ttc)
                ttc = nan; % if the two vehicles will not collide set to nan
            end
            
            %%%% Debug and Vidualization
            
            if RiskAssessment.DEBUG_FLAG_TTC
                
                figure
                
                subplot(4,1,1)
                hold on
                plot(egoPosArr(:,1), egoPosArr(:,2), '-bo')
                plot(egoPosArr(collisionIndex,1), egoPosArr(collisionIndex,2), '-bx', 'MarkerSize',20)
                plot(otherPosArr(:,1), otherPosArr(:,2), '-ro')
                plot(otherPosArr(collisionIndex,1), otherPosArr(collisionIndex,2), '-rx', 'MarkerSize',20)
                viscircles(egoPosArr(collisionIndex,:), distThreshold,'Color', 'b');
                viscircles(otherPosArr(collisionIndex,:), distThreshold,'Color', 'r');
                axis equal
                grid on                
                hold off
                
                subplot(4,1,2)
                hold on
                plot(egoPosArr(:,1), '-bo')
                plot(otherPosArr(:,1), '-ro')
                
                subplot(4,1,3)
                hold on
                plot(egoPosArr(:,2), '-bo')
                plot(otherPosArr(:,2), '-ro')
                
                subplot(4,1,4)
                plot(distArr)
                
            end
            
        end
        
        function anomalousFlagArr = getAnomalousFlagArr(solutionCell)                        
            
            solutionCount = length(solutionCell);
            
            anomalousFlagArr = false(SimulationVehicleData.MAX_VEHICLE_COUNT,solutionCount);
            
            for sIndex = 1 : solutionCount
                
                if ~isnan(solutionCell{sIndex})                
                    for vIndex = 1 : (length(solutionCell{1}) / AnomalyEquationSolver.INDEX_ALL_TYPES)
                        anomalousFlagArr(vIndex, sIndex)...
                            = (sum(solutionCell{sIndex}((vIndex-1) * AnomalyEquationSolver.INDEX_ALL_TYPES+1:vIndex * AnomalyEquationSolver.INDEX_ALL_TYPES)) ~= AnomalyEquationSolver.INDEX_ALL_TYPES);                    
                    end
                end
                
            end
            
            anomalousFlagArr = anomalousFlagArr(1:vIndex,:);
            
        end
        
        function tu = computeMinAvoidTime(egoState, otherState)
            
            %tu  = nan;
            
            egoVel = egoState(1, SimulationVehicleData.TABLE_INDEX_V_X:SimulationVehicleData.TABLE_INDEX_V_Y);
            egoPos = egoState(1, SimulationVehicleData.TABLE_INDEX_POS_X:SimulationVehicleData.TABLE_INDEX_POS_Y);
            
            otherVel = otherState(1, SimulationVehicleData.TABLE_INDEX_V_X:SimulationVehicleData.TABLE_INDEX_V_Y);
            otherPos = otherState(1, SimulationVehicleData.TABLE_INDEX_POS_X:SimulationVehicleData.TABLE_INDEX_POS_Y);
            
            otherAcc = otherState(1, SimulationVehicleData.TABLE_INDEX_ACC_X:SimulationVehicleData.TABLE_INDEX_ACC_Y);
            
            egoVelMag = norm(egoVel);
            egoVelMagUnit = egoVel / egoVelMag;
            
            otherAccLong = dot(otherAcc,egoVelMagUnit);
            
            
            relPos = dot(otherPos - egoPos, egoVelMagUnit);
            relVel = dot(otherVel - egoVel, egoVelMagUnit);
            
            if (relVel > 0) && (relPos > 0)                
                tu = relVel / (RiskAssessment.ACCELERATION_MAX - otherAccLong);                
            elseif (relVel < 0) && (relPos < 0)
                tu = abs(relVel) / (RiskAssessment.DECELERATION_MAX + otherAccLong);
            else
                tu = 0;
            end
            
        end
        
        function vDims = getVehicleDims(actors)
            
            vDims = zeros(SimulationVehicleData.MAX_VEHICLE_COUNT,2);
            
%             for vIndex = 1 : length(actors)
%                 actorTemp = actors(vIndex);
%                 vDims(actorTemp.ActorID,:) = [actorTemp.Length, actorTemp.Width];
%             end
            
            for vIndex = 1 : length(actors)
                %actorTemp = actors(vIndex);
                vDims(vIndex+1,:) = [4.7, 1.8];
            end
            
        end
        
        function posPredict = predictLoc(posOld, vel1D, acc1D, headingRad, yawRateRad, timeInterval, stepTotalCount)
            
            tempPos = zeros(stepTotalCount+1,2);
            tempPos(1,:) = posOld;
            
            velInc = acc1D * timeInterval;
            
            tempVel = vel1D;            
            tempHeading = headingRad;
            
            if yawRateRad < 0.001
                yawRateRad = 0;
            end
            
            if acc1D < 0.01
                acc1D = 0;
            end
            
            if vel1D > 0
                initVelPositive = true;
            elseif vel1D < 0
                initVelPositive = false;
            else
                initVelPositive = nan;
            end
            
            if yawRateRad == 0
                
                dirVecTimesTime = [cos(headingRad) sin(headingRad)] * timeInterval;
                accDisplacement = dirVecTimesTime * timeInterval * 0.5 * acc1D;
                
                for pIndex = 1 : stepTotalCount
                    
                    tempPos(pIndex+1,:) = tempPos(pIndex,:) + dirVecTimesTime * tempVel + accDisplacement;
                    tempVel = tempVel + velInc;
                    
                    if (initVelPositive == true) && (tempVel < 0)                        
                        tempVel = 0;
                    elseif (initVelPositive == false) && (tempVel > 0)
                        tempVel = 0;
                    end
                    
                end
                
            else
                
                accIntervalDivYawRate = acc1D * timeInterval / yawRateRad;
                accDivYawRateSqaure = acc1D / (yawRateRad ^ 2);
                headingInc = yawRateRad * timeInterval;
                
                for pIndex = 1 : stepTotalCount
                    hPrime = tempHeading + headingInc;
                    
                    tempPos(pIndex+1,1) = tempPos(pIndex,1)... 
                        + tempVel / yawRateRad * (sin(hPrime) - sin(tempHeading))... 
                        + accIntervalDivYawRate * sin(hPrime)... 
                        - accDivYawRateSqaure * (cos(tempHeading) - cos(hPrime));
                    
                    tempPos(pIndex+1,2) = tempPos(pIndex,2)... 
                        + tempVel / yawRateRad * (cos(tempHeading) - cos(hPrime))... 
                        - accIntervalDivYawRate * cos(hPrime)... 
                        - accDivYawRateSqaure * (sin(tempHeading) - sin(hPrime));
                    
                    tempVel = tempVel + velInc;
                    tempHeading = hPrime;
                end
            end
            
            posPredict = tempPos(2:end,:);
            
        end

        function [posPredict, velPredict, headPredict] = predictStatus(posOld, vel1D, acc1D, headingRad, yawRateRad, timeInterval, stepTotalCount)
            tempPos = zeros(stepTotalCount+1,2);
            tempPos(1,:) = posOld;
                    
            tempVelArr = zeros(stepTotalCount+1,1);
            tempVelArr(1) = vel1D;

            tempHeadingArr = zeros(stepTotalCount+1,1);
            tempHeadingArr(1) = headingRad;

            velInc = acc1D * timeInterval;
                    
            tempVel = vel1D;            
            tempHeading = headingRad;
                    
            if yawRateRad < 0.001
                yawRateRad = 0;
            end
                    
            if acc1D < 0.01
                acc1D = 0;
            end
                    
            if vel1D > 0
                initVelPositive = true;
            elseif vel1D < 0
                initVelPositive = false;
            else
                initVelPositive = nan;
            end
                    
            if yawRateRad == 0
                        
                dirVecTimesTime = [cos(headingRad) sin(headingRad)] * timeInterval;
                accDisplacement = dirVecTimesTime * timeInterval * 0.5 * acc1D;
                        
                for pIndex = 1 : stepTotalCount
                            
                    tempPos(pIndex+1,:) = tempPos(pIndex,:) + dirVecTimesTime * tempVel + accDisplacement;
                    tempVel = tempVel + velInc;
                            
                    if (initVelPositive == true) && (tempVel < 0)                        
                        tempVel = 0;
                    elseif (initVelPositive == false) && (tempVel > 0)
                        tempVel = 0;
                    end
                    tempVelArr(pIndex+1) = tempVel;
                            
                end
                        
            else
                        
                accIntervalDivYawRate = acc1D * timeInterval / yawRateRad;
                accDivYawRateSqaure = acc1D / (yawRateRad ^ 2);
                headingInc = yawRateRad * timeInterval;
                        
                for pIndex = 1 : stepTotalCount
                    hPrime = tempHeading + headingInc;
                            
                    tempPos(pIndex+1,1) = tempPos(pIndex,1)... 
                        + tempVel / yawRateRad * (sin(hPrime) - sin(tempHeading))... 
                        + accIntervalDivYawRate * sin(hPrime)... 
                        - accDivYawRateSqaure * (cos(tempHeading) - cos(hPrime));
                            
                    tempPos(pIndex+1,2) = tempPos(pIndex,2)... 
                        + tempVel / yawRateRad * (cos(tempHeading) - cos(hPrime))... 
                        - accIntervalDivYawRate * cos(hPrime)... 
                        - accDivYawRateSqaure * (sin(tempHeading) - sin(hPrime));
                            
                    tempVel = tempVel + velInc;
                    tempHeading = hPrime;
                    tempVelArr(pIndex+1) = tempVel;
                    tempHeadingArr(pIndex+1) = tempHeading;
                end
            end
                    
            posPredict = tempPos(2:end,:);
            velPredict = tempVelArr(2:end);
            headPredict = tempHeadingArr(2:end);
                    
        end

        
    end
end

