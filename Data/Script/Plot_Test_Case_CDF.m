%% Prepare data

%dataSampleCount = [298	106	64	48	17	24	24];
%dataSampleCount = [298	106	64	48	17	24];
dataSampleCount = [192 95 64	98	98	57	69];


tempArr = [];

for dIndex = 1 : length(dataSampleCount)
    tempArr = [tempArr, 1:dataSampleCount(dIndex)];
end

%% Plot data

figure
ecdf(tempArr/10)
grid on
set(gca,'FontSize',20)
ylabel('CDF')
xlabel('Lead Time (s)')

%%
grid on
ax = gca;
ax.GridColor = [0, 0, 0];
ax.GridAlpha = 0.5;
set(gca,'FontSize',35)