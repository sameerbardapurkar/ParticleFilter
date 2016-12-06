function [data] = loadScanData(scanDataCell)
% Loads scan data from text file, puts them into structure file

% Take mean of each column, store in matrix
scanData = mean(cell2mat(scanDataCell));
scanDataVar = var(cell2mat(scanDataCell));

% Build

% Extract useful data from scan
data.minAngle = scanData(5);
data.maxAngle = scanData(6);
data.angleIncrement = scanData(7);
data.numRays = round((data.maxAngle - data.minAngle)/data.angleIncrement);  % Number of points in each scan
rangeStart = 12;  % index of starting range

% Extract scan data from images
data.rangeData = zeros(1, data.numRays);
data.rangeDataVariance = zeros(1, data.numRays);
data.intensityData = zeros(1, data.numRays);
data.intensityDataVariance = zeros(1, data.numRays);
data.points = zeros(3, data.numRays);

for i = 1:data.numRays
    data.rangeData(i) = scanData(rangeStart - 1 + i);
    data.rangeDataVariance(i) = scanDataVar(rangeStart - 1 + i);
    data.intensityData(i) = scanData(rangeStart + data.numRays + i);
    data.intensityDataVariance(i) = scanDataVar(rangeStart + data.numRays + i);
    data.points(1, i) = data.rangeData(i) * cos(data.minAngle + data.angleIncrement*(i-1));  %  z-value
    data.points(2, i) = data.rangeData(i) * sin(data.minAngle + data.angleIncrement*(i-1));  %  x-value
    data.points(3, i) = data.intensityData(i);  %  intensity value
end

return

