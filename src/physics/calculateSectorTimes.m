function sectors = calculateSectorTimes(track, speed, n_sectors)
%CALCULATESECTORTIMES Divide the track into sectors and compute time per sector.
if nargin < 3
    n_sectors = 3;
end

s_total = track.s(end);
ds      = track.ds(:);
v       = max(speed(:), 1e-3);   % avoid divide-by-zero
dt      = ds ./ v(1:end-1);      % time per segment

s_bounds = linspace(0, s_total, n_sectors+1);

times_s       = zeros(1, n_sectors);
avg_speed_mps = zeros(1, n_sectors);

for k = 1:n_sectors
    s_start = s_bounds(k);
    s_end   = s_bounds(k+1);

    % Indices of segments that fall within this sector
    s_mid = (track.s(1:end-1) + track.s(2:end)) / 2;  % segment midpoints
    in_sector = s_mid >= s_start & s_mid < s_end;

    if ~any(in_sector)
        continue;
    end
    times_s(k)       = sum(dt(in_sector));
    avg_speed_mps(k) = sum(ds(in_sector)) / times_s(k);
end

sectors.boundaries_m  = s_bounds;
sectors.times_s       = times_s;
sectors.avg_speed_mps = avg_speed_mps;
sectors.labels        = arrayfun(@(k) sprintf('S%d', k), 1:n_sectors, 'UniformOutput', false);
end
