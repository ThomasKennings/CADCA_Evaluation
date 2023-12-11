% Each statetable row is a car. Each column is different data, indexed per
% the list below.
% TABLE_INDEX_TIMESTAMP  = 1;
% TABLE_INDEX_VEHICLE_ID = 2;
% TABLE_INDEX_POS_X      = 3; % meters
% TABLE_INDEX_POS_Y      = 4; % meters
% TABLE_INDEX_V_X        = 5; % m/s
% TABLE_INDEX_V_Y        = 6; % m/s
% TABLE_INDEX_ACC_X      = 7; % m/s2
% TABLE_INDEX_ACC_Y      = 8; % m/s2
% TABLE_INDEX_H          = 9; % degree
% TABLE_INDEX_YAW_RATE   =10; % degree / s
% TABLE_INDEX_DIST       =11;
% TABLE_INDEX_TOTAL      =11;

time = squeeze(dataTable.Data(1, 1, :));
car_1_position = squeeze(dataTable.Data(1, 3, :));

plot(time, car_1_position);