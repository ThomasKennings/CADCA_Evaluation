% Set up Script for the Lane Following Example
%
% This script initializes the lane following example model. It loads
% necessary control constants and sets up the buses required for the
% referenced model.
%
%   This is a helper script for example purposes and may be removed or
%   modified in the future.

%   Copyright 2018 The MathWorks, Inc.

%% General Model Parameters
Ts = 0.1;               % Simulation sample time  (s)

%% Path following Controller Parameters
time_gap        = 1.5;   % time gap               (s)
default_spacing = 10;    % default spacing        (m)
max_ac          = 2;     % Maximum acceleration   (m/s^2)
min_ac          = -3;    % Minimum acceleration   (m/s^2)
max_steer       = 0.26;  % Maximum steering       (rad)
min_steer       = -0.26; % Minimum steering       (rad) 
PredictionHorizon = 30;  % Prediction horizon     

%% Create driving scenario
% The scenario name is a scenario file created by the Driving Scenario Designer App. 
scenariosNames = {
    'ACC_01_ISO_TargetDiscriminationTest.mat',...               % scenarioId = 1
    'ACC_02_ISO_AutoRetargetTest.mat',...                       % scenarioId = 2
    'ACC_03_ISO_CurveTest.mat',...                              % scenarioId = 3
    'ACC_04_StopnGo.mat',...                                    % scenarioId = 4
    'LFACC_01_DoubleCurve_DecelTarget.mat',...                  % scenarioId = 5
    'LFACC_02_DoubleCurve_AutoRetarget.mat',...                 % scenarioId = 6
    'LFACC_03_DoubleCurve_StopnGo.mat',...                      % scenarioId = 7
    'LFACC_04_Curve_CutInOut.mat',...                           % scenarioId = 8
    'LFACC_06_Curve_CutInOut_TooClose_test.mat',...             % scenarioId = 9    
    'Intersection_S01_Unsafe_Right_Turn.mat'...                 % scenarioId = 10
    'Parallel_Simple_10mps.mat',...                             % scenarioId = 11
    'Parallel_Simple_20mps.mat',...                             % scenarioId = 12
    'Parallel_Simple_30mps.mat',...                             % scenarioId = 13
    'Parallel_Change_Lane_10mps_Crash_1.mat',...                % scenarioId = 14
    'Parallel_Change_Lane_10mps_Crash_2.mat',...                % scenarioId = 15
    'Parallel_Change_Lane_10mps_No_Crash_1.mat',...             % scenarioId = 16
    'Parallel_Change_Lane_10mps_No_Crash_2.mat',...             % scenarioId = 17
    'Parallel_Change_Lane_20mps_Crash_1.mat',...                % scenarioId = 18
    'Parallel_Change_Lane_20mps_Crash_2.mat',...                % scenarioId = 19
    'Parallel_Change_Lane_20mps_No_Crash_1.mat',...             % scenarioId = 20
    'Parallel_Change_Lane_20mps_No_Crash_2.mat',...             % scenarioId = 21
    'Parallel_Change_Lane_30mps_Crash_1.mat',...                % scenarioId = 22
    'Parallel_Change_Lane_30mps_Crash_2.mat',...                % scenarioId = 23
    'Parallel_Change_Lane_30mps_No_Crash_1.mat',...             % scenarioId = 24
    'Parallel_Change_Lane_30mps_No_Crash_2.mat',...             % scenarioId = 25
    'Intersection_10mps_Ignore_Red_Light_No_Crash_1.mat',...    % scenarioId = 26
    'Intersection_10mps_Ignore_Red_Light_No_Crash_2.mat',...    % scenarioId = 27
    'Intersection_10mps_Ignore_Red_Light_Crash_1.mat',...       % scenarioId = 28
    'Intersection_10mps_Ignore_Red_Light_Crash_2.mat',...       % scenarioId = 29
    'Intersection_20mps_Ignore_Red_Light_No_Crash_1.mat',...    % scenarioId = 30
    'Intersection_20mps_Ignore_Red_Light_No_Crash_2.mat',...    % scenarioId = 31
    'Intersection_30mps_Ignore_Red_Light_No_Crash_1.mat',...    % scenarioId = 32
    'Intersection_30mps_Ignore_Red_Light_No_Crash_2.mat',...    % scenarioId = 33
    'Intersection_20mps_Ignore_Red_Light_Crash_1.mat',...       % scenarioId = 34
    'Intersection_20mps_Ignore_Red_Light_Crash_2.mat',...       % scenarioId = 35
    'Intersection_30mps_Ignore_Red_Light_Crash_1.mat',...       % scenarioId = 36
    'Intersection_30mps_Ignore_Red_Light_Crash_2.mat',...       % scenarioId = 37
    'Intersection_10mps_Unsafe_Right_Turn_No_Crash_1.mat',...   % scenarioId = 38
    'Intersection_10mps_Unsafe_Right_Turn_No_Crash_2.mat',...   % scenarioId = 39
    'Intersection_10mps_Unsafe_Right_Turn_Crash_1.mat',...      % scenarioId = 40
    'Intersection_10mps_Unsafe_Right_Turn_Crash_2.mat',...      % scenarioId = 41
    'Intersection_20mps_Unsafe_Right_Turn_No_Crash_1.mat',...   % scenarioId = 42
    'Intersection_20mps_Unsafe_Right_Turn_No_Crash_2.mat',...   % scenarioId = 43
    'Intersection_20mps_Unsafe_Right_Turn_Crash_1.mat',...      % scenarioId = 44
    'Intersection_20mps_Unsafe_Right_Turn_Crash_2.mat',...      % scenarioId = 45
    'Intersection_30mps_Unsafe_Right_Turn_Crash_1.mat',...      % scenarioId = 46
    'Intersection_30mps_Unsafe_Right_Turn_Crash_2.mat',...      % scenarioId = 47
    'Intersection_30mps_Unsafe_Right_Turn_No_Crash_1.mat',...   % scenarioId = 48
    'Intersection_30mps_Unsafe_Right_Turn_No_Crash_2.mat',...   % scenarioId = 49
    'Intersection_10mps_Unsafe_Left_Turn_Crash_1.mat',...       % scenarioId = 50
    'Intersection_10mps_Unsafe_Left_Turn_Crash_2.mat',...       % scenarioId = 51
    'Intersection_10mps_Unsafe_Left_Turn_No_Crash_1.mat',...    % scenarioId = 52
    'Intersection_10mps_Unsafe_Left_Turn_No_Crash_2.mat',...    % scenarioId = 53
    'Intersection_20mps_Unsafe_Left_Turn_Crash_1.mat',...       % scenarioId = 54
    'Intersection_20mps_Unsafe_Left_Turn_Crash_2.mat',...       % scenarioId = 55
    'Intersection_20mps_Unsafe_Left_Turn_No_Crash_1.mat',...    % scenarioId = 56
    'Intersection_20mps_Unsafe_Left_Turn_No_Crash_2.mat',...    % scenarioId = 57
    'Intersection_30mps_Unsafe_Left_Turn_Crash_1.mat',...       % scenarioId = 58
    'Intersection_30mps_Unsafe_Left_Turn_Crash_2.mat',...       % scenarioId = 59
    'Intersection_30mps_Unsafe_Left_Turn_No_Crash_1.mat',...    % scenarioId = 60
    'Intersection_30mps_Unsafe_Left_Turn_No_Crash_2.mat',...    % scenarioId = 61
    };

scenarioStopTimes = [19.82 17.99 21.99 28.61 27.3 39.66 34.28 22.8 23.47 23.47 30 30 30 10 10 8 8 8 9 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10];
scenarioId = 28;

% The scenario file is converted to a drivingScenario object
% initial conditions of ego car and actor profiles
[scenario,egoCar,actor_Profiles] = helperSessionToScenario(scenariosNames{scenarioId});

if scenarioId == 8 || scenarioId == 9 || scenarioId == 10
    %v_set = 21.5;       % ACC set speed (m/s)
    v_set = 20.6;       % ACC set speed (m/s)
else
    v_set = egoCar.v0;  % ACC set speed (m/s)
end

% Initial condition for the ego car in ISO 8855 coordinates
v0_ego   = egoCar.v0;          % Initial speed of the ego car           (m/s)
x0_ego   = egoCar.x0;          % Initial x position of ego car          (m)
y0_ego   = egoCar.y0;          % Initial y position of ego car          (m)
yaw0_ego = egoCar.yaw0;        % Initial yaw angle of ego car           (rad)

% Convert ISO 8855 to SAE J670E coordinates
y0_ego = -y0_ego;
%yaw0_ego = deg2rad(-yaw0_ego);
yaw0_ego = -yaw0_ego;

% Define a simulation stop time
simStopTime = scenarioStopTimes(scenarioId);

%% Tracking and Sensor Fusion Parameters                        Units
clusterSize = 4;        % Distance for clustering               (m)
assigThresh = 20;       % Tracker assignment threshold          (N/A)
M           = 2;        % Tracker M value for M-out-of-N logic  (N/A)
N           = 3;        % Tracker M value for M-out-of-N logic  (N/A)
numCoasts   = 5;        % Number of track coasting steps        (N/A)
numTracks   = 100;       % Maximum number of tracks              (N/A)
numSensors  = 2;        % Maximum number of sensors             (N/A)

% Position and velocity selectors from track state
% The filter initialization function used in this example is initcvekf that 
% defines a state that is: [x;vx;y;vy;z;vz]. 
posSelector = [1,0,0,0,0,0; 0,0,1,0,0,0]; % Position selector   (N/A)
velSelector = [0,1,0,0,0,0; 0,0,0,1,0,0]; % Velocity selector   (N/A)

%% Ego Car Parameters
% Dynamics modeling parameters
m       = 1575;     % Total mass of vehicle                          (kg)
Iz      = 2875;     % Yaw moment of inertia of vehicle               (m*N*s^2)
lf      = 1.2;      % Longitudinal distance from c.g. to front tires (m)
lr      = 1.6;      % Longitudinal distance from c.g. to rear tires  (m)
Cf      = 19000;    % Cornering stiffness of front tires             (N/rad)
Cr      = 33000;    % Cornering stiffness of rear tires              (N/rad)
tau     = 0.5;      % time constant for longitudinal dynamics 1/s/(tau*s+1)
%% Bus Creation
% Load the Simulink model
modelName = 'LaneFollowingTestBenchExample_Test';
wasModelLoaded = bdIsLoaded(modelName);
if ~wasModelLoaded
    load_system(modelName)
end

% Create buses for lane sensor and lane sensor boundaries
createLaneSensorBuses;

% load the bus for scenario reader
blk=find_system(modelName,'System','driving.scenario.internal.ScenarioReader');
s = get_param(blk{1},'PortHandles');
get(s.Outport(1),'SignalHierarchy');

% Set the scenario reader file name to the selected scenario
set_param(blk{1},'ScenarioFileName',scenariosNames{scenarioId});

% Create the bus of tracks (output from referenced model)
refModel = 'LFRefMdl';
wasReModelLoaded = bdIsLoaded(refModel);
if ~wasReModelLoaded
    load_system(refModel)
    blk=find_system(refModel,'System','multiObjectTracker');
    multiObjectTracker.createBus(blk{1});
    close_system(refModel)
else
    blk=find_system(refModel,'System','multiObjectTracker');
    multiObjectTracker.createBus(blk{1});
end

%% Code generation
% To generate code, uncomment the following commands.
% refModel = 'LFRefMdl';
% rtwbuild(refModel)