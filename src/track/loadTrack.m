function [x, y] = loadTrack(csvFilePath)
data = readmatrix(csvFilePath);

if isempty(data) || size(data,2) < 2
    error('Track CSV must contain at least two columns (x, y).');
end

x = data(:,1);
y = data(:,2);
end
