classdef CADCA %< matlab.mixin.Copyable
    %CADCA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        scenario;
        consistencyChecker;
        riskAssessor;
    end
    
    methods
        
        function obj = CADCA(scenario)
            obj.scenario = scenario;
            obj.consistencyChecker = ConsistencyCheck();
            obj.riskAssessor = RiskAssessment(scenario);
        end
        
        function obj_out = copy(obj)
            obj_out = CADCA(obj.scenario);
        end
        
        function [result, controlIndex, correctedTable] = performCompleteProcess(obj, stateTable, stateTableOld, controlInputs)
            
            % (Input) controlInputs: should be the same as defined in RiskAssessment
            
            interval = stateTable(1,SimulationVehicleData.TABLE_INDEX_TIMESTAMP) - stateTableOld(1,SimulationVehicleData.TABLE_INDEX_TIMESTAMP);            
            
            [eqSys, eqVal, maxVehicle] = obj.consistencyChecker.performConsistencyCheck(stateTable, stateTableOld, interval);            
            solutionCell = AnomalyEquationSolver.solveEquationSystem(eqSys, eqVal, maxVehicle);                        
            localViews = LocalViewConstructor.constructLocalViews(solutionCell, stateTable, stateTableOld);            
            [isSafeArr, ttcArr] = obj.riskAssessor.performRiskAssessment(controlInputs, localViews);
            anomalousFlagArr = obj.riskAssessor.getAnomalousFlagArr(solutionCell);
            [controlIndex, mostLikelyScenario] = obj.riskAssessor.riskResultAggregation(isSafeArr, ttcArr, anomalousFlagArr);
            correctedTable = localViews{mostLikelyScenario};
            result = controlInputs(controlIndex,:);
        end
        
    end
end

