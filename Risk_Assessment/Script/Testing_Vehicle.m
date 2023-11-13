%% Basic Vehicle Settings

ve = Vehicle([0 0 0 0 0 0 0]);
v1 = Vehicle([5 5 0 0 0 0 0]);

%% Dist and Dir test

[dist, dir] = Vehicle.getDistDir(ve, v1)

%% Compute Collision Time
%%%% Perpenticular

ve = Vehicle([0 0 0 1 0 0 0]);
v1 = Vehicle([4 4 0 1 0 -pi/2 0]);
Vehicle.computeTimeToCollision(ve, v1)

%% Compute Collision Time

ve = Vehicle([0 0 0 10 0 0 0]);
v1 = Vehicle([0 5 0  0 0 0 0]);
[t, distMin, distMax, dV, dA, vertCoor] = Vehicle.computeTimeToCollision(ve, v1);
tAC = Vehicle.computeTimeToAvoidCollision(ve, v1, distMin, distMax, dV, dA, vertCoor)

%% 

ve = Vehicle([0 0 0 10 0 0 0]);
Vehicle.performPredictionNoSteering(ve, 5, true);