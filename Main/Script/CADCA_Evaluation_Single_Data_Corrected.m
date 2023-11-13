%% Load Data

dataId = 3;

dataMatName = {
    'Parallel_Simple_10mps_Data.mat';
    'Parallel_Simple_20mps_Data.mat';
    'Parallel_Simple_30mps_Data.mat';
    };

clear scenario

load(dataMatName{dataId});

%% Load Contrl Input Profile

usePredefinedControl = true;

if usePredefinedControl
    controlProfileName = {
        'ControlProfile_Template.mat';
        'ControlProfile_Template.mat';
        'ControlProfile_Template.mat';
        };

    load(controlProfileName{dataId});
end

%% Anomaly Settings

vehicleID = 2;
%dataIndex = [SimulationVehicleData.TABLE_INDEX_POS_X SimulationVehicleData.TABLE_INDEX_POS_Y];
dataIndex = [SimulationVehicleData.TABLE_INDEX_V_X SimulationVehicleData.TABLE_INDEX_V_Y];
offset = [20, 0];
startEndIndex = [1 nan];

%% Perform Evaluation

eva = EvaluationCADCA();

ra = RiskAssessment(scenario);

totalTimeStepCount = size(conciseTable,3);
conciseTable = SimulationVehicleData.recomputeYawRate(conciseTable);

if isnan(controlInsertInterval(2))
    controlInsertInterval(2) = totalTimeStepCount;
end

manipulatedTable = AnomalyGenerator.shiftData(conciseTable, vehicleID, dataIndex, offset, startEndIndex);

result = eva.performSingleTestCase(scenario, manipulatedTable, controlInput, controlInsertInterval(1), conciseTable);

gtResult = eva.performSingleTestCase(scenario, conciseTable, controlInput, controlInsertInterval(1), conciseTable);


%% Compute Ground Truth TTC

ttcArr = ra.getGroundTruthTTCEstimation(controlInput, manipulatedTable);
raOnlyResult = eva.pureRiskAssessmentChoice(scenario, controlInput, manipulatedTable);

%% Compute and Output Statistics

[missCount, fpCount, delayCount, correctCount, totalCount, safeCount, unsafeCount] = eva.summarizePerformance(gtResult, result, 2);
[missCountRA, fpCountRA, delayCountRA, correctCountRA, totalCountRA, safeCountRA, unsafeCountRA] = eva.summarizePerformance(gtResult, raOnlyResult, 2);

%% Visualize Results

fprintf('Method \t Miss Count \t FP Count \t Delay Count \t Correct Count \t Total Count \t Safe Count \t Unsafe Count\n')
fprintf('CADCA \t %d \t %d \t %d \t %d \t %d \t %d \t %d \n', missCount, fpCount, delayCount, correctCount, totalCount, safeCount, unsafeCount)
fprintf('Naive RA \t %d \t %d \t %d \t %d \t %d \t %d \t %d \n', missCountRA, fpCountRA, delayCountRA, correctCountRA, totalCountRA, safeCountRA, unsafeCountRA)

plotFigure = true;


if plotFigure

    figure

    totalTimeCount = size(result,1);
    totalSubPlot = 6;

    subplot(totalSubPlot,1,1)
    hold on
    plot((squeeze(conciseTable(1,SimulationVehicleData.TABLE_INDEX_POS_X,:))), 'b', 'DisplayName', 'Vehicle-Ego')
    plot((squeeze(conciseTable(2,SimulationVehicleData.TABLE_INDEX_POS_X,:))), 'r', 'DisplayName', 'Vehicle-2')
    plot((squeeze(conciseTable(3,SimulationVehicleData.TABLE_INDEX_POS_X,:))), 'g', 'DisplayName', 'Vehicle-3')
    ylabel('Location-X (m)')
    xlim([0 totalTimeCount])
    legend('Location', 'best')

    subplot(totalSubPlot,1,2)
    hold on
    plot((squeeze(conciseTable(1,SimulationVehicleData.TABLE_INDEX_V_X,:))), 'b', 'DisplayName', 'Vehicle-Ego')
    plot((squeeze(conciseTable(2,SimulationVehicleData.TABLE_INDEX_V_X,:))), 'r', 'DisplayName', 'Vehicle-2')
    plot((squeeze(conciseTable(3,SimulationVehicleData.TABLE_INDEX_V_X,:))), 'g', 'DisplayName', 'Vehicle-3')
    ylabel('Velocity-X (m)')
    xlim([0 totalTimeCount])
    legend('Location', 'best')

    subplot(totalSubPlot,1,3)
    hold on
    plot(ttcArr(2,:), 'b', 'DisplayName', 'Aunomous Control')
    plot(ttcArr(1,:), 'r', 'DisplayName', 'Manual Control')
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