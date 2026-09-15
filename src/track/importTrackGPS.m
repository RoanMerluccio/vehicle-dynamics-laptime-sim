function [x, y] = importTrackGPS(csvFilePath)
%IMPORTTRACKGPS Convert GPS coordinates (lat, lon) to Cartesian coordinates (x, y in metres).
R_earth = 6371000;  % mean Earth radius (m)

data = readmatrix(csvFilePath);
if isempty(data) || size(data,2) < 2
    error('GPS CSV must have at least two columns: lat, lon.');
end

lat_deg = data(:,1);
lon_deg = data(:,2);

% Reference point = first coordinate
lat0 = lat_deg(1) * pi/180;
lon0 = lon_deg(1) * pi/180;

lat_rad = lat_deg * pi/180;
lon_rad = lon_deg * pi/180;

% Equirectangular projection
x = R_earth * (lon_rad - lon0) .* cos(lat0);   % East (m)
y = R_earth * (lat_rad - lat0);                  % North (m)
end
