% ==========================================================
%  Vehicle Dynamics & Lap-Time Simulator
%  RUN_ME.m  -  Double-click this in MATLAB to get started.
% ==========================================================

%% 1. Setup
projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(projectRoot, 'src')));
cd(projectRoot);

%% 2. Vehicle parameters
%    Edit any value here and re-run.

vehicle.mass_kg          = 250;     % total vehicle mass (kg)
vehicle.tire_mu          = 1.0;     % peak tire friction coefficient
vehicle.Cd               = 0.6;     % drag coefficient
vehicle.Cl               = 0.3;     % downforce coefficient
vehicle.frontal_area_m2  = 0.5;    % frontal area (m^2)
vehicle.air_density_kgm3 = 1.225;  % air density - sea level (kg/m^3)
vehicle.wheel_radius_m   = 0.230;  % effective wheel radius (m)
vehicle.brake_max_N      = 8000;   % maximum brake force (N)

% CG and wheelbase for longitudinal weight transfer
vehicle.cg_height_m      = 0.30;   % CG height (m)
vehicle.wheelbase_m      = 1.55;   % wheelbase (m)

% --- POWERTRAIN: choose Option A or Option B ---

% Option A: Constant power (simple)
vehicle.power_W = 50000;   % 50 kW constant

% Option B: Torque curve + gearbox (more realistic)
% Uncomment this block to use the torque curve model:
%
% vehicle.torque_rpm  = [3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 11000];
% vehicle.torque_Nm   = [50,   65,   72,   75,   74,   71,   65,   55,    40  ];
% vehicle.gear_ratios = [3.6,  2.4,  1.8,  1.4,  1.1];   % internal gear ratios
% vehicle.final_drive = 3.5;                               % final drive ratio
% vehicle.transmission_eff = 0.92;                        % drivetrain efficiency

%% 3. Track file
trackFile = fullfile(projectRoot, 'data', 'example_track.csv');

%% 4. Run simulation
fprintf('\n--- Running simulation ---\n');
results = runSimulation(trackFile, vehicle);

%% 5. Print summary
fprintf('\n========= RESULTS =========\n');
fprintf('  Lap time      : %6.2f s\n',   results.lap_time_s);
fprintf('  Avg speed     : %6.2f m/s  (%.1f km/h)\n', ...
        results.average_speed_mps, results.average_speed_mps*3.6);
fprintf('  Max speed     : %6.2f m/s  (%.1f km/h)\n', ...
        results.max_speed_mps, results.max_speed_mps*3.6);
fprintf('  Max lat acc   : %6.2f m/s2 (%.2f g)\n', ...
        max(results.lateral_acc_mps2), max(results.lateral_acc_mps2)/9.81);
fprintf('  Max long acc  : %6.2f m/s2 (%.2f g)\n', ...
        max(abs(results.longitudinal_acc_mps2)), ...
        max(abs(results.longitudinal_acc_mps2))/9.81);
fprintf('\n  Sector times:\n');
for k = 1:numel(results.sectors.times_s)
    fprintf('    %s: %.2f s  (avg %.1f m/s)\n', ...
        results.sectors.labels{k}, ...
        results.sectors.times_s(k), ...
        results.sectors.avg_speed_mps(k));
end
fprintf('===========================\n\n');

%% 6. Plots
visualizeResults(results);

%% 7. 3D Car Simulation (optional)
%  Uncomment to watch the car drive around the track in interactive 3D:
%  simulateCar3D(results);
%    Uncomment to see how lap time varies with mass.
%
% masses = 180:20:380;
% lap_times = zeros(size(masses));
% for i = 1:numel(masses)
%     vehicle.mass_kg = masses(i);
%     r = runSimulation(trackFile, vehicle);
%     lap_times(i) = r.lap_time_s;
% end
% figure('Name','Mass Sensitivity');
% plot(masses, lap_times, 'bo-', 'LineWidth', 1.5);
% grid on; xlabel('Mass (kg)'); ylabel('Lap time (s)');
% title('Lap time vs. vehicle mass');

fprintf('Done. Edit vehicle parameters in Section 2 and re-run.\n');
