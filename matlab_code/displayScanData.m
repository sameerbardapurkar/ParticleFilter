function [] = displayScanData(scanData, scanFilename)

%% Parameters
pointSize = 5;  % Size of plotted point
numRays = scanData.numRays;
points = scanData.points;

figure
% Adjust potential hit points (intensity above intThresh) for visualization
ptSize = pointSize * ones(1, numRays);
  % Build colormap for intensity values
colormap(jet)
scatter(points(1,:), points(2,:), ptSize, points(3,:), 'filled')
hold on
  % Plot robot
rectangle('Position', [-0.9 -0.275 1.0 0.55], 'Curvature', [0.95, 0.25], 'FaceColor', [1 0 0.1]);
th = linspace(scanData.minAngle, scanData.maxAngle, 250); radius = 0.15; x = radius*cos(th); y = radius*sin(th); patch(x, y, 'blue');
ax = gca;
% ax.XAxisLocation = 'origin';
% ax.YAxisLocation = 'origin';
grid on
axis equal
title(strcat('Mean range data for', {' '}, scanFilename(19:end-4)))
xlabel('y (m)')
ylabel('x (m)')
hold off

%% Display mean and variance of range data
figure
errorbar(1:scanData.numRays, scanData.rangeData, scanData.rangeDataVariance, 'o')
title(strcat('Mean and variance of range data for',{' '}, scanFilename(19:end-4)))
xlabel('LRF angle index (counterclockwise)')
ylabel('Range (m)')
xlim([-10 scanData.numRays+10])
return