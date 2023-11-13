%% Load Data

dataId = 8;

dataMatName = {
    'Parallel_Simple_10mps_Data.mat';                    % 1
    'Parallel_Simple_20mps_Data.mat';                    % 2
    'Parallel_Simple_30mps_Data.mat';                    % 3
    'Parallel_Change_Lane_10mps_Crash_1_Data.mat';       % 4
    'Parallel_Change_Lane_10mps_Crash_2_Data.mat';       % 5
    'Parallel_Change_Lane_10mps_No_Crash_1_Data.mat';    % 6
    'Parallel_Change_Lane_10mps_No_Crash_2_Data.mat';    % 7
    'Intersection_10mps_Ignore_Red_Light_Crash_1_Data.mat'; % 8
    '.mat';
    '.mat';
    '.mat';
    '.mat';
    '.mat';
    '.mat';
    '.mat';
    '.mat';
    '.mat';
    '.mat';
    '.mat';
    };

load(dataMatName{dataId});

%% Load Contrl Input Profile

usePredefinedControl = true;

controlProfileId = 2;

if usePredefinedControl
    controlProfileName = {
        'ControlProfile_Template.mat';
        'ControlProfile_Template_Decelerate_North.mat';
        'ControlProfile_Template.mat';
        };

    load(controlProfileName{controlProfileId});
end

%% Perform Evaluation

eva = EvaluationCADCA();

ra = RiskAssessment(scenario);

totalTimeStepCount = size(conciseTable,3);
conciseTable = SimulationVehicleData.recomputeYawRate(conciseTable);

if isnan(controlInsertInterval(2))
    controlInsertInterval(2) = totalTimeStepCount;
end

result = eva.performSingleTestCase(scenario, conciseTable, controlInput, controlInsertInterval(1), conciseTable);

% result = zeros(diff(controlInsertInterval)+1,1);
% 
% for aIndex = controlInsertInterval(1) : controlInsertInterval(2)
%     result(aIndex-controlInsertInterval(1)+1) = eva.performSingleTestCase(scenario, conciseTable, controlInput, aIndex);
% end

%% Compute Ground Truth TTC

ttcArr = ra.getGroundTruthTTCEstimation(controlInput, conciseTable);

%% Visualize Results

figure

targetId = 6;

totalTimeCount = size(result,1);

totalPlot = 5;

subplot(totalPlot,1,1)
hold on
plot((squeeze(conciseTable(1,SimulationVehicleData.TABLE_INDEX_POS_X,:))), 'b', 'DisplayName', 'Vehicle-Ego')
plot((squeeze(conciseTable(targetId,SimulationVehicleData.TABLE_INDEX_POS_X,:))), 'r', 'DisplayName', 'Vehicle-Target')
plot((squeeze(conciseTable(3,SimulationVehicleData.TABLE_INDEX_POS_X,:))), 'g', 'DisplayName', 'Vehicle-3')
ylabel('Location-X (m)')
xlim([0 totalTimeCount])
legend('Location', 'best')

subplot(totalPlot,1,2)
hold on
plot((squeeze(conciseTable(1,SimulationVehicleData.TABLE_INDEX_POS_Y,:))), 'b', 'DisplayName', 'Vehicle-Ego')
plot((squeeze(conciseTable(targetId,SimulationVehicleData.TABLE_INDEX_POS_Y,:))), 'r', 'DisplayName', 'Vehicle-Target')
plot((squeeze(conciseTable(3,SimulationVehicleData.TABLE_INDEX_POS_Y,:))), 'g', 'DisplayName', 'Vehicle-3')
ylabel('Location-Y (m)')
xlim([0 totalTimeCount])
legend('Location', 'best')

subplot(totalPlot,1,3)
hold on
plot((squeeze(conciseTable(1,SimulationVehicleData.TABLE_INDEX_V_X,:))), 'b', 'DisplayName', 'Vehicle-Ego')
plot((squeeze(conciseTable(targetId,SimulationVehicleData.TABLE_INDEX_V_X,:))), 'r', 'DisplayName', 'Vehicle-2')
plot((squeeze(conciseTable(3,SimulationVehicleData.TABLE_INDEX_V_X,:))), 'g', 'DisplayName', 'Vehicle-3')
ylabel('Velocity-X (m)')
xlim([0 totalTimeCount])
legend('Location', 'best')

subplot(totalPlot,1,4)
hold on
plot(ttcArr(2,:), 'b', 'DisplayName', 'Aunomous Control')
plot(ttcArr(1,:), 'r', 'DisplayName', 'Manual Control')
ylabel('TTC (s)')
xlim([0 totalTimeCount])
legend('Location', 'best')

subplot(totalPlot,1,5)
stem(result)
ylabel('Control Decision')
xlim([0 totalTimeCount])


