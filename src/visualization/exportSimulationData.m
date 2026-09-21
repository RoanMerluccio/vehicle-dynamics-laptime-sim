function jsonData = exportSimulationData(results, outputPath)
%EXPORTSIMULATIONDATA Export lap simulation telemetry to JSON for 3D visualization.
%
%   jsonData = exportSimulationData(results)
%   jsonData = exportSimulationData(results, outputPath)
%
%   Inputs:
%     results    - Struct returned by calculateLapTime or runSimulation
%     outputPath - (Optional) File path to write the JSON data (e.g. 'web/sim_data.json')
%
%   Outputs:
%     jsonData   - Encoded JSON string containing track 3D points, telemetry series,
%                  and vehicle summary.

track   = results.track;
speed   = results.speed_profile_mps;
vehicle = results.vehicle;
g0      = 9.81;

N = numel(track.x);

% Ensure 3D elevation
if isfield(track, 'z') && ~isempty(track.z) && numel(track.z) == N
    z = track.z(:);
else
    % If Spa track (length ~ 7000 m), use Spa elevation profile
    if max(track.s) > 5000
        z = generateSpaElevation(track.s(:));
    else
        z = zeros(N, 1);
    end
end

% Cumulative time along the lap
if isfield(results, 'cumulative_time_s') && numel(results.cumulative_time_s) == N
    t = results.cumulative_time_s(:);
elseif isfield(results, 'dt_s') && numel(results.dt_s) == N - 1
    t = [0; cumsum(results.dt_s(:))];
else
    ds = track.ds(:);
    v_safe = max(speed(:), 0.5);
    dt = ds ./ v_safe(1:end-1);
    t = [0; cumsum(dt)];
end

% Accelerations in g
latG = (results.lateral_acc_mps2(:)) / g0;
lonG = (results.longitudinal_acc_mps2(:)) / g0;

% Estimate throttle and brake pedals (0 to 1)
throttle = max(0, min(1, lonG / 1.5));
brake    = max(0, min(1, -lonG / 2.0));

% Estimate gear and RPM from speed profile
gears = vehicle.gear_ratios;
fd    = vehicle.final_drive;
r_w   = vehicle.wheel_radius_m;

gearSeries = ones(N, 1);
rpmSeries  = zeros(N, 1);

for i = 1:N
    v = speed(i);
    w_wheel = v / r_w;  % rad/s
    bestGear = 1;
    bestRpm = 4000;
    
    for gIdx = 1:numel(gears)
        testRpm = w_wheel * gears(gIdx) * fd * 60 / (2 * pi);
        if testRpm <= 12500 && testRpm >= 3500
            bestGear = gIdx;
            bestRpm = testRpm;
        elseif testRpm < 3500 && gIdx == 1
            bestGear = 1;
            bestRpm = max(2500, testRpm);
        end
    end
    gearSeries(i) = bestGear;
    rpmSeries(i)  = round(min(12500, max(2500, bestRpm)));
end

% Heading angle psi (rad)
dx = gradient(track.x(:));
dy = gradient(track.y(:));
heading_rad = atan2(dy, dx);

% Package output struct
simData = struct();
simData.track_name = 'Spa-Francorchamps Circuit';
simData.lap_time_s = results.lap_time_s;
simData.max_speed_kmh = results.max_speed_mps * 3.6;
simData.avg_speed_kmh = results.average_speed_mps * 3.6;
simData.track_length_m = max(track.s);
simData.num_points = N;

simData.telemetry = struct( ...
    's', round(track.s(:), 2), ...
    'x', round(track.x(:), 3), ...
    'y', round(track.y(:), 3), ...
    'z', round(z, 2), ...
    'time', round(t, 3), ...
    'speed_mps', round(speed(:), 2), ...
    'speed_kmh', round(speed(:) * 3.6, 1), ...
    'lat_g', round(latG, 3), ...
    'lon_g', round(lonG, 3), ...
    'throttle', round(throttle, 2), ...
    'brake', round(brake, 2), ...
    'gear', gearSeries, ...
    'rpm', rpmSeries, ...
    'heading_rad', round(heading_rad, 4) ...
);

if isfield(results, 'sectors')
    simData.sectors = results.sectors;
end

% Track curvature + segment lengths, needed by the browser-side physics
% solver (web/spa_3d_simulation.html "What-If" sliders) to re-run the
% cornering-speed / two-pass solve client-side without MATLAB.
simData.track_kappa = round(track.kappa(:), 6);
simData.track_ds    = round(track.ds(:), 4);

% Baseline vehicle parameters, so the browser solver starts from the same
% numbers MATLAB used (sliders then apply deltas on top of these).
simData.vehicle_baseline = buildVehicleBaseline(vehicle);

jsonData = jsonencode(simData);

% Write to file if outputPath specified
if nargin >= 2 && ~isempty(outputPath)
    outDir = fileparts(outputPath);
    if ~isempty(outDir) && ~exist(outDir, 'dir')
        mkdir(outDir);
    end
    [~, ~, ext] = fileparts(outputPath);
    if strcmpi(ext, '.js')
        % Browser <script src="...js"> needs an assignment, not bare JSON.
        fileContent = ['window.SPA_SIM_DATA = ' jsonData ';'];
    else
        fileContent = jsonData;
    end
    fid = fopen(outputPath, 'w');
    if fid == -1
        error('Failed to open file for writing: %s', outputPath);
    end
    fwrite(fid, fileContent, 'char');
    fclose(fid);
    fprintf('Telemetry exported to %s (%.1f KB)\n', outputPath, numel(fileContent)/1024);
end
end

function vb = buildVehicleBaseline(vehicle)
%BUILDVEHICLEBASELINE Defensively pull the fields the web solver needs,
%   falling back to the RUN_SPA.m defaults for any that are missing.
vb.mass_kg          = getf(vehicle, 'mass_kg', 1200);
vb.tire_mu          = getf(vehicle, 'tire_mu', 1.6);
vb.Cd               = getf(vehicle, 'Cd', 0.8);
vb.Cl               = getf(vehicle, 'Cl', 1.5);
vb.frontal_area_m2  = getf(vehicle, 'frontal_area_m2', 0.5);
vb.air_density_kgm3 = getf(vehicle, 'air_density_kgm3', 1.225);
vb.wheel_radius_m   = getf(vehicle, 'wheel_radius_m', 0.230);
vb.brake_max_N      = getf(vehicle, 'brake_max_N', 10000);
vb.torque_rpm       = getf(vehicle, 'torque_rpm', [3000 4000 5000 6000 7000 8000 9000 10000 11000 12000]);
vb.torque_Nm        = getf(vehicle, 'torque_Nm', [38 52 62 68 70 69 65 58 47 32]);
vb.gear_ratios      = getf(vehicle, 'gear_ratios', [3.6 2.4 1.8 1.4 1.1]);
vb.final_drive      = getf(vehicle, 'final_drive', 3.8);
vb.transmission_eff = getf(vehicle, 'transmission_eff', 0.92);
end

function v = getf(s, f, d)
if isfield(s, f) && ~isempty(s.(f))
    v = s.(f);
else
    v = d;
end
end
