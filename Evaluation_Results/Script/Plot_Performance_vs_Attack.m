%% Need to prepare data first

%%%% Define resultTable

INDEX_MISS_COUNT    = 1;
INDEX_FP_COUNT      = 2;
INDEX_DELAY         = 3;
INDEX_CORRECT_COUNT = 4;
INDEX_TOTAL_COUNT   = 5;
INDEX_SAFE_COUNT    = 6;
INDEX_UNSAFE_COUNT  = 7;

cadcaPerformance = resultTable(1:2:end,:);
raPerformance = resultTable(2:2:end,:);

%%
succBlockingRate = [1 - cadcaPerformance(:,INDEX_MISS_COUNT) ./ cadcaPerformance(:,INDEX_SAFE_COUNT), 1 - raPerformance(:,INDEX_MISS_COUNT) ./ raPerformance(:,INDEX_SAFE_COUNT)];
fpRate = [cadcaPerformance(:,INDEX_FP_COUNT) ./ cadcaPerformance(:,INDEX_UNSAFE_COUNT), raPerformance(:,INDEX_FP_COUNT) ./ raPerformance(:,INDEX_UNSAFE_COUNT)];

%%
combinedTable = [succBlockingRate; fpRate]';

%combinedTable = [];

%return
%% Plot figure

%figure
bar(combinedTable' * 100)

%% Format figure

grid on
ylim([0 100])
set(gca,'FontSize',20)