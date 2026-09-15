function results = calculateLapTime(track, speed)
%CALCULATELAPTIME Calculate total lap time and acceleration statistics.

EPS = 1e-3;
v   = max(speed(:), EPS);
ds  = track.ds(:);

% Calculus: velocity v = ds / dt => dt = ds / v
dt = ds ./ v(1:end-1);

% Total lap time is the Riemann sum: integral of (1/v) ds
results.lap_time_s            = sum(dt);
results.average_speed_mps     = sum(ds) / results.lap_time_s;
results.max_speed_mps         = max(v);
results.min_speed_mps         = min(v(v > EPS));
results.dt_s                  = dt;
results.cumulative_time_s     = [0; cumsum(dt)];

% Longitudinal acceleration: a = dv / dt
results.longitudinal_acc_mps2 = [diff(v) ./ dt; 0];
end
