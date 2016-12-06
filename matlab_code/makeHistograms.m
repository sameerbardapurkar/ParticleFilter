function [] = makeHistograms( scanDataCell, scanData, makeGIF )
% Take in scanData and scanDataCell and plot/make gif of histograms

filename = '../GIFS/laserGIF.gif'; % Specify the output file name for GIF
rangeStart = 12;  % Index of starting range
scanDataMatrix = cell2mat(scanDataCell);  % Convert to matrix
% allPoints = zeros(size(scanDataMatrix, 1) * scanData.numRays, 2);  % Initialize array to store all (x,y) points
allPoints = [];

% Build out arrays with all (x,y) points
for i = 1:scanData.numRays
    x = scanDataMatrix(:, rangeStart - 1 + i)*cos(scanData.minAngle + scanData.angleIncrement*(i-1));  %  z-value
    y = scanDataMatrix(:, rangeStart - 1 + i)*sin(scanData.minAngle + scanData.angleIncrement*(i-1));  %  x-value
    allPoints = [allPoints; [x y]];
end
% Remove all points
B = allPoints(:,1).*allPoints(:,2);
allPoints(abs(B) < 0.05, :) = [];

% Plot
hFig = figure('Position', [1, 1, 1920/2*.5, (1080-100)*.5]);
subplot(2, 1, 1)
rectangle('Position', [-0.275 -0.9 0.55 1.0], 'Curvature', [0.25, 0.95], 'FaceColor', [1 0 0.1]);
th = linspace(scanData.minAngle, scanData.maxAngle, 250); radius = 0.15; x = radius*cos(th+pi/2); y = radius*sin(th+pi/2); patch(x, y, 'blue');
xlabel('x (m)')
ylabel('y (m)')
for i = -14:13
    line([(38.1 + 76.2*i)/100 (38.1 + 76.2*i)/100], [-0.5 10], 'Color', [0 1 0]);
end
axis equal
xlim([-10 10])
ylim([-1.5 10])

for i = 1:scanData.numRays
    subplot(2, 1, 1)
    hLine = line([0 10*sin(scanData.minAngle - i*scanData.angleIncrement+pi)], ...
            [0 10*cos(scanData.minAngle - i*scanData.angleIncrement+pi)]);
    title(['Laser range finder angle: ' num2str(rad2deg(scanData.minAngle + i*scanData.angleIncrement)) ' degrees'])
    
    subplot(2, 1, 2)
    angleRanges = cell2mat(scanDataCell(:, rangeStart + i));
    histogram(angleRanges(angleRanges ~=0), 0:.05:10)
    xlim([0 10])
    ylim([0 300])
    xlabel('Range (m)')
    ylabel('# results')
    title(['Laser range finder angle: ' num2str(rad2deg(scanData.minAngle + i*scanData.angleIncrement)) ' degrees'])
    grid on
    drawnow
    if makeGIF
        frame = getframe(hFig);
        im{i} = frame2im(frame);
    end
    delete(hLine)
end

xHist = figure();
histogram(allPoints(:,2), 1000)
xlabel('x (m)')
ylabel('# results')

%% Make GIF
if makeGIF
    for idx = 1:scanData.numRays
        [A,map] = rgb2ind(im{idx},256);
        if idx == 1
            imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0.05);
        else
            imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.05);
        end
    end
end



end

