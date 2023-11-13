%% Load Data

dataId = 6;

dataMatName = {
    'Parallel_Simple_10mps_Data.mat';                    % 1
    'Parallel_Simple_20mps_Data.mat';                    % 2
    'Parallel_Simple_30mps_Data.mat';                    % 3
    'Parallel_Change_Lane_10mps_Crash_1_Data.mat';       % 4
    'Parallel_Change_Lane_10mps_Crash_2_Data.mat';       % 5
    'Parallel_Change_Lane_10mps_No_Crash_1_Data.mat';    % 6
    'Parallel_Change_Lane_10mps_No_Crash_2_Data.mat';    % 7
    'Intersection_10mps_Ignore_Red_Light_Crash_1_Data.mat';  % 8
    'Intersection_10mps_Ignore_Red_Light_Crash_2_Data.mat';  % 9
    'Intersection_10mps_Unsafe_Right_Turn_Crash_1_Data.mat'; %10
    'Intersection_10mps_Unsafe_Right_Turn_Crash_2_Data.mat'; %11
    'Intersection_10mps_Unsafe_Left_Turn_Crash_1_Data.mat';  %12
    'Intersection_10mps_Unsafe_Left_Turn_Crash_2_Data.mat';  %13
    '.mat';
    '.mat';
    '.mat';
    '.mat';
    '.mat';
    '.mat';
    '.mat';
    };

clear scenario

load(dataMatName{dataId});

%% Load Contrl Input Profile

usePredefinedControl = true;

controlProfileId = 1;

if usePredefinedControl
    controlProfileName = {
        'ControlProfile_Template.mat';
        'ControlProfile_Template_Decelerate_North.mat';
        'ControlProfile_Template_Decelerate_East.mat';
        };

    load(controlProfileName{controlProfileId});
end

safeControlIndex = 2;

%% Anomaly Settings

vehicleIDManipulated = 6;
dataIndex = [SimulationVehicleData.TABLE_INDEX_POS_X SimulationVehicleData.TABLE_INDEX_POS_Y];
%dataIndex = [SimulationVehicleData.TABLE_INDEX_V_X SimulationVehicleData.TABLE_INDEX_V_Y];

offsetRaw = [
    5, 0;
    10, 0;
    15, 0;
    20, 0;
    25, 0;
    ];

% offsetRaw = [
%     0, -5;
%     0, -10;
%     0, -15;
%     0, -20;
%     0, -25;
%     ];

startEndIndex = [1 nan];

%% Perform Evaluation

eva = EvaluationCADCA();

ra = RiskAssessment(scenario);

totalTimeStepCount = size(conciseTable,3);
conciseTable = SimulationVehicleData.recomputeYawRate(conciseTable);

if isnan(controlInsertInterval(2))
    controlInsertInterval(2) = totalTimeStepCount;
end

totalOffsetRow = size(offsetRaw,1);

fprintf('Method \t Miss Count \t FP Count \t Delay Count \t Correct Count \t Total Count \t Safe Count \t Unsafe Count\n')

gtResult = eva.performSingleTestCase(scenario, conciseTable, controlInput, controlInsertInterval(1), conciseTable);

for oIndex = 1 : totalOffsetRow

    offset = offsetRaw(oIndex,:);
    
    

    manipulatedTable = AnomalyGenerator.shiftData(conciseTable, vehicleIDManipulated, dataIndex, offset, startEndIndex);

    result = eva.performSingleTestCase(scenario, manipulatedTable, controlInput, controlInsertInterval(1), conciseTable);

    %% Compute Ground Truth TTC

    ttcArr = ra.getGroundTruthTTCEstimation(controlInput, manipulatedTable);
    raOnlyResult = eva.pureRiskAssessmentChoice(scenario, controlInput, manipulatedTable);

    %% Compute and Output Statistics

    [missCount, fpCount, delayCount, correctCount, totalCount, safeCount, unsafeCount] = eva.summarizePerformance(gtResult, result, safeControlIndex);
    [missCountRA, fpCountRA, delayCountRA, correctCountRA, totalCountRA, safeCountRA, unsafeCountRA] = eva.summarizePerformance(gtResult, raOnlyResult, safeControlIndex);

    %% Visualize Results

    
    fprintf('CADCA \t %d \t %d \t %d \t %d \t %d \t %d \t %d \n', missCount, fpCount, delayCount, correctCount, totalCount, safeCount, unsafeCount)
    fprintf('Naive RA \t %d \t %d \t %d \t %d \t %d \t %d \t %d \n', missCountRA, fpCountRA, delayCountRA, correctCountRA, totalCountRA, safeCountRA, unsafeCountRA)

    plotFigure = false;

    if plotFigure

        figure

        totalTimeCount = size(result,1);
        totalSubPlot = 6;

        subplot(totalSubPlot,1,1)
        hold on
        plot((squeeze(conciseTable(1,SimulationVehicleData.TABLE_INDEX_POS_X,:))), 'b', 'DisplayName', 'Vehicle-Ego')
        plot((squeeze(conciseTable(vehicleIDManipulated,SimulationVehicleData.TABLE_INDEX_POS_X,:))), 'r', 'DisplayName', 'Vehicle-Manipulated')
        plot((squeeze(conciseTable(3,SimulationVehicleData.TABLE_INDEX_POS_X,:))), 'g', 'DisplayName', 'Vehicle-3')
        ylabel('Location-X (m)')
        xlim([0 totalTimeCount])
        legend('Location', 'best')

        subplot(totalSubPlot,1,2)
        hold on
        plot((squeeze(conciseTable(1,SimulationVehicleData.TABLE_INDEX_V_X,:))), 'b', 'DisplayName', 'Vehicle-Ego')
        plot((squeeze(conciseTable(vehicleIDManipulated,SimulationVehicleData.TABLE_INDEX_V_X,:))), 'r', 'DisplayName', 'Vehicle-Manipulated')
        plot((squeeze(conciseTable(3,SimulationVehicleData.TABLE_INDEX_V_X,:))), 'g', 'DisplayName', 'Vehicle-3')
        ylabel('Velocity-X (m)')
        xlim([0 totalTimeCount])
        legend('Location', 'best')

        subplot(totalSubPlot,1,3)
        hold on
        plot(ttcArr(2,:), 'b', 'DisplayName', 'Control-2')
        plot(ttcArr(1,:), 'r', 'DisplayName', 'Control-1')
        ylabel('TTC (s) - Mani.')
        xlim([0 totalTimeCount])
        legend('Location', 'best')

        subplot(totalSubPlot,1,4)
        stem(result)
        ylabel('CADCA Decision')
        xlim([0 totalTimeCount])

        subplot(totalSubPlot,1,5)
        stem(raOnlyResult)
        ylabel('RA Decision')
        xlim([0 totalTimeCount])

        subplot(totalSubPlot,1,6)
        stem(gtResult)
        ylabel('GT Decision')
        xlim([0 totalTimeCount])
    end

end