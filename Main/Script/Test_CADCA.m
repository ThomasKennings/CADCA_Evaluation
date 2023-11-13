%% Load Data and Initialize CADCA


load('ConciseTableExample.mat')
load('ConciseTableExample_Scenario.mat')

%% Prepare Data

stateTableOld = conciseTable(:,:,2);
stateTable = conciseTable(:,:,3);

controlInputs = [0 0; -1 0];

cadca = CADCA(scenario);

%% Run CADCA

controlDecision = cadca.performCompleteProcess(stateTable, stateTableOld, controlInputs);

%% 

eCADCA = EvaluationCADCA();
eCADCA.performSingleTestCase(scenario, conciseTable, controlInputs, 13);