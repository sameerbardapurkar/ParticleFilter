clearvars
clc
close all

imgFiles = dir('../*.png');
numfiles = length(imgFiles);
data = cell(1, numfiles);

for i = 1:numfiles
    mydata{i} = imread(strcat('./16-831-ParticleFilter-e0296f95b45b1fb7cde53bdc94afbffc6da98c3f/scripts/images/image', num2str(i), '.png'));
end

filename = 'rodotData6-merged.gif'; % Specify the output file name
for idx = 1:numfiles
    [A,map] = rgb2ind(mydata{idx},256);
    if idx == 1
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0.1);
    else
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.1);
    end
end