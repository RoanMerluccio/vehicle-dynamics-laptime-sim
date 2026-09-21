% ==========================================================
%  Vehicle Dynamics & Lap-Time Simulator
%  RUN_SPA.m  -  Circuit de Spa-Francorchamps
% ==========================================================
%
%  Track data: TU Munich racetrack database (open-source, CC BY 4.0)
%  Source: github.com/TUMFTM/racetrack-database
%  Format: pre-projected Cartesian x/y centreline (metres), ~7-m track width
%  Points: 1401 waypoints, ~7 m spacing
%  Actual Spa lap length: ~7.004 km

%% 1. Setup
projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(projectRoot, 'src')));
cd(projectRoot);

%% 2. Load the Spa track
%    The TUM CSV has columns: x_m, y_m, w_tr_right_m, w_tr_left_m
%    We only need the first two (x and y in metres).
fprintf('\n--- Loading Spa-Francorchamps track ---\n');
raw = readmatrix(fullfile(projectRoot, 'data', 'spa_francorchamps.csv'));
x = raw(:,1);
y = raw(:,2);
fprintf('    %d track points. Approx length: %.0f m\n', numel(x), ...
        sum(sqrt(diff(x).^2 + diff(y).^2)));

%% 3. Vehicle parameters
%    Formula SAE car.
%    Edit any value below.

vehicle.mass_kg          = 1200;     % kg (car + driver)
vehicle.tire_mu          = 1.6;     % slick tire on dry tarmac
vehicle.Cd               = 0.8;     % aero package drag
vehicle.Cl               = 1.5;     % aero package downforce
vehicle.frontal_area_m2  = 0.5;    % m^2
vehicle.air_density_kgm3 = 1.225;
vehicle.wheel_radius_m   = 0.230;
vehicle.brake_max_N      = 10000;
vehicle.cg_height_m      = 0.30;
vehicle.wheelbase_m      = 1.55;

% Tire model: 'pacejka' uses Magic Formula 94 (industry-standard nonlinear model)
%             'linear'  uses Coulomb friction (classic F = mu*N, backward compatible)
vehicle.tire_model       = 'pacejka';   % <-- upgrade from linear
% Pacejka coeffs (FSAE dry-slick defaults; override with real TTC data if available):
% vehicle.pacejka_coeffs = struct('B', 10.0, 'C', 1.9, 'D', 1.6, 'E', 0.97);

% Powertrain: torque curve from a typical 600cc inline-4 (FSAE)
vehicle.torque_rpm  = [3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 11000, 12000];
vehicle.torque_Nm   = [38,   52,   62,   68,   70,   69,   65,   58,    47,    32  ];
vehicle.gear_ratios = [3.6,  2.4,  1.8,  1.4,  1.1];  % 5-speed
vehicle.final_drive      = 3.8;
vehicle.transmission_eff = 0.92;

%% 4. Run simulation
track    = calculateTrackGeometry(x, y);
v_corner = calculateCorneringSpeed(vehicle, track.kappa);
speed    = calculateSpeedProfile(track, vehicle, v_corner);
results  = calculateLapTime(track, speed);

results.lateral_acc_mps2  = speed.^2 .* abs(track.kappa);
results.speed_profile_mps = speed;
results.track             = track;
results.vehicle           = vehicle;
results.sectors           = calculateSectorTimes(track, speed, 3);

%% 5. Print summary
fprintf('\n========= SPA-FRANCORCHAMPS RESULTS =========\n');
fprintf('  Lap time      : %6.2f s  (%.2f min)\n', ...
        results.lap_time_s, results.lap_time_s/60);
fprintf('  Avg speed     : %6.2f m/s  (%.1f km/h)\n', ...
        results.average_speed_mps, results.average_speed_mps*3.6);
fprintf('  Max speed     : %6.2f m/s  (%.1f km/h)\n', ...
        results.max_speed_mps, results.max_speed_mps*3.6);
fprintf('  Max lat acc   : %6.2f m/s2 (%.2f g)\n', ...
        max(results.lateral_acc_mps2), max(results.lateral_acc_mps2)/9.81);
fprintf('  Max long acc  : %6.2f m/s2 (%.2f g)\n', ...
        max(abs(results.longitudinal_acc_mps2)), ...
        max(abs(results.longitudinal_acc_mps2))/9.81);
fprintf('\n  Sector times (equal-distance thirds):\n');
for k = 1:numel(results.sectors.times_s)
    fprintf('    %s: %.2f s  (avg %.1f km/h)\n', ...
        results.sectors.labels{k}, ...
        results.sectors.times_s(k), ...
        results.sectors.avg_speed_mps(k)*3.6);
end
fprintf('=============================================\n\n');

%% 6. Plots
visualizeResults(results);

%% 7. Interactive 3D Simulation
%  Export telemetry for WebGL browser viewer
exportSimulationData(results, fullfile(projectRoot, 'web', 'spa_sim_data.js'));

%  Launch native MATLAB 3D simulation with live HUD and camera controls:
fprintf('Launching 3D Car Simulation...\n');
simulateCar3D(results);

%% 8. Bonus: Tire model comparison (linear Coulomb vs. Pacejka Magic Formula)
%    Quantifies the improvement from upgrading to the industry-standard nonlinear model.
fprintf('\n--- Tire Model Comparison ---\n');
vehicle_linear            = vehicle;
vehicle_linear.tire_model = 'linear';
track2    = calculateTrackGeometry(x, y);
vc_lin    = calculateCorneringSpeed(vehicle_linear, track2.kappa);
spd_lin   = calculateSpeedProfile(track2, vehicle_linear, vc_lin);
res_lin   = calculateLapTime(track2, spd_lin);
fprintf('  Linear (Coulomb) tire:   %6.2f s  (%.2f min)\n', ...
    res_lin.lap_time_s, res_lin.lap_time_s/60);
fprintf('  Pacejka Magic Formula:   %6.2f s  (%.2f min)\n', ...
    results.lap_time_s, results.lap_time_s/60);
dt_compare = res_lin.lap_time_s - results.lap_time_s;
if dt_compare >= 0
    fprintf('  Delta: Pacejka is %.2f s faster (nonlinear load-sensitivity effect)\n', dt_compare);
else
    fprintf('  Delta: Pacejka predicts %.2f s longer lap\n', abs(dt_compare));
end
fprintf('----------------------------\n\n');
