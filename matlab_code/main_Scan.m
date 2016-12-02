clearvars
close all
clc

%% Parameters
scanFilename = './data/field_data/9-25-16_0degree_scan.txt';
% imgFilename = 'image_data/scan4.jpg';

%% Import data from files.
scanDataCell = importScanData(scanFilename);
scanData = loadScanData(scanDataCell);  % returns structure scanData
displayScanData(scanData)


