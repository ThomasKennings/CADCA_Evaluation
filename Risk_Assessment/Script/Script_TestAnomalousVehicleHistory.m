%data = [zeros(10,1); 1; zeros(50,1)];
%data = ones(100,1);
data = [ones(20,1) ;zeros(100,1)];

%WAC = zeros(size(data,1),1);

%for endIndex = 1 : size(data,1)
    
    %WAC(endIndex,1) = AnomalousVehicleHistory.computeWeightedLikelyhood(data(1:endIndex,1));
    
%end

%WAC = AnomalousVehicleHistory.computeWeightedLikelyhood(data);
WAC = AnomalousVehicleHistory.computeWeightedAnomalyCount(data);

figure
subplot(2,1,1)
plot(WAC)
subplot(2,1,2)
stem(data, 'LineWidth', 6)