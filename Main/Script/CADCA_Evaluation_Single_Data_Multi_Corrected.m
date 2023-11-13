%% Load Data

dataId = 25;

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
    'Parallel_Simple_10mps_Data_0p1.mat';                    %14
    'Parallel_Simple_20mps_Data_0p1.mat';                    %15
    'Parallel_Simple_30mps_Data_0p1.mat';                    %16
    'Parallel_Simple_10mps_Data_0p5.mat';                    %17
    'Parallel_Simple_20mps_Data_0p5.mat';                    %18
    'Parallel_Simple_30mps_Data_0p5.mat';                    %19
    'Parallel_Simple_10mps_Data_0p9.mat';                    %20
    'Parallel_Simple_20mps_Data_0p9.mat';                    %21
    'Parallel_Simple_30mps_Data_0p9.mat';                    %22
    'Parallel_Change_Lane_10mps_Crash_1_Data_0p1.mat';       %23
    'Parallel_Change_Lane_10mps_Crash_1_Data_0p5.mat';       %24
    'Parallel_Change_Lane_10mps_Crash_1_Data_0p9.mat';       %25
    'Intersection_10mps_Ignore_Red_Light_Crash_1_Data_0p1.mat'; %26
    'Intersection_10mps_Ignore_Red_Light_Crash_1_Data_0p5.mat'; %27
    'Intersection_10mps_Ignore_Red_Light_Crash_1_Data_0p9.mat'; %28
    'Intersection_10mps_Unsafe_Left_Turn_Crash_1_Data_0p1.mat'; %29
    'Intersection_10mps_Unsafe_Left_Turn_Crash_1_Data_0p5.mat'; %30
    'Intersection_10mps_Unsafe_Left_Turn_Crash_1_Data_0p9.mat'; %31
    };

clear scenario

load(dataMatName{dataId});

%% Load Contrl Input Profile

usePredefinedControl = true;

controlProfileId = 2;

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
%dataIndex = [SimulationVehicleData.TABLE_INDEX_POS_X SimulationVehicleData.TABLE_INDEX_POS_Y];
%dataIndex = [SimulationVehicleData.TABLE_INDEX_V_X SimulationVehicleData.TABLE_INDEX_V_Y];
%dataIndex = [SimulationVehicleData.TABLE_INDEX_DIST];
dataIndex = [SimulationVehicleData.TABLE_INDEX_POS_X SimulationVehicleData.TABLE_INDEX_POS_Y SimulationVehicleData.TABLE_INDEX_DIST];

% offsetRaw = [
%     5, 0;
%     10, 0;
%     15, 0;
%     20, 0;
%     25, 0;
%     ];

% offsetRaw = [
%     0, -5;
%     0, -10;
%     0, -15;
%     0, -20;
%     0, -25;
%     ];

% offsetRaw = [    
%     6;
%     7;
%     8;
%     9;
%     10;
%     ];

offsetRaw = [
     5, 0, 6;
    10, 0, 7;
    15, 0, 8;
    20, 0, 9;
    25, 0, 10;
    ];

% offsetRaw = [
%     0, 5, 6;
%     0, 10, 7;
%     0, 15, 8;
%     0, 20, 9;
%     0, 25, 10;
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

    result = eva.performSingleTestCaseWithEstimationUpdate(scenario, manipulatedTable, controlInput, controlInsertInterval(1), conciseTable);

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