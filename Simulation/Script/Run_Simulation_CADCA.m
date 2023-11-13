
%% Prepare and run simulation

modelNameStr = 'LaneFollowingTestBenchExample_Test';

open_system(modelNameStr)
%load_system(modelNameStr)

if ~strcmp(modelNameStr, modelName)
    disp('Simulink Model Names do not match!!!')
    return
end

%%

helperLFSetUp_Test
sim(modelName)

%% Post processing data

try
    conciseTable = SimulationVehicleData.getConciseTable(dataTableOld.Data,tableUpdated);
catch errMsg
    errMsg
end

%% Save Data

fileName = [scenariosNames{scenarioId}(1,1:end-4) '_Data.mat'];
save(fileName,'conciseTable','scenario');