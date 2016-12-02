function [] = displayScanData(scanData)

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
hold off

return