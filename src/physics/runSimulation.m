function results = runSimulation(trackCsvPath, vehicle)
%RUNSIMULATION Orchestrate the full lap-time simulation pipeline.
[x, y]   = loadTrack(trackCsvPath);
track    = calculateTrackGeometry(x, y);

% Use smoothed curvature for cornering and speed profile
v_corner = calculateCorneringSpeed(vehicle, track.kappa);
speed    = calculateSpeedProfile(track, vehicle, v_corner);
results  = calculateLapTime(track, speed);

% Lateral acceleration: a_lat = v^2 * |kappa|
results.lateral_acc_mps2  = speed.^2 .* abs(track.kappa);
results.speed_profile_mps = speed;
results.track             = track;
results.vehicle           = vehicle;

% Sector analysis (3 sectors by default)
results.sectors = calculateSectorTimes(track, speed, 3);
end
