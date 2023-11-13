%% Load example data

load('ConciseTableExample.mat');
load('ConciseTableExample_Scenario.mat');

%% Create Risk Assessment Instance

ra = RiskAssessment(scenario);

%% Test Vehicle Location Prediction

posOld = [0 0];
vel1D = 10;
acc1D = 0;
headingRad = pi / 4;
yawRateRad = pi / 8;
timeInterval = 1;
stepTotalCount = 30;
locArr = ra.predictLoc(posOld, vel1D, acc1D, headingRad, yawRateRad, timeInterval, stepTotalCount);

plot(locArr(:,1), locArr(:,2), '-o')

%% Test TTC Computation

egoState = [0, 1, -40, 0, 10, 0, 0, 0, 0, 0, 0];
%otherState = [0, 2, -43, 0, -10, 0, 0, 0, 180, 0, 0];
otherState = [0, 2, 0, -40, 0, 10, 0, 0, -90, 0, 0];
distThreshold = norm(ra.vehicleDims(1,:)) / 2;

[ttc, tu, egoPosArr, otherPosArr] = RiskAssessment.computeTTCAndTu(egoState, otherState, distThreshold)

%%

egoState = [0, 1, -40, 0, 10, 0, 0, 0, 0, 0, 0];
other1State = [0, 2, -43, 0, -10, 0, 0, 0, 180, 0, 0];
other2State = [0, 3, 0, -40, 0, 10, 0, 0, -90, 0, 0];
stateTable = [egoState; other1State; other2State];

[isSafe, isSafeArr, ttcArr] = ra.performRiskAssessment(stateTable)