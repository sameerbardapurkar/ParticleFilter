% clearvars
close all
clc

%% Parameters
scanFilename = './data/field_data/9-25-16_15degree_scan.txt';
makeGIF = 0;  % Flag to save gif of histograms

%% Import data from files.
if (exist (strcat(scanFilename(1:end-4),'.mat'))) == 2
    load (strcat(scanFilename(1:end-4),'.mat'))
else
    scanDataCell = importScanData(scanFilename);
end

scanData = loadScanData(scanDataCell);  % returns structure scanData
save(strcat(scanFilename(1:end-4),'.mat'), 'scanDataCell', 'scanData')

% displayScanData(scanData, scanFilename)
makeHistograms(scanDataCell, scanData, makeGIF)

