function fig = simulateCar3D(results, varargin)
%SIMULATECAR3D Interactive 3D vehicle lap simulation and animation.
%
%   fig = simulateCar3D(results)
%   fig = simulateCar3D(results, 'OptionName', OptionValue, ...)
%
%   Inputs:
%     results   - Struct returned by calculateLapTime or runSimulation.
%                 Must contain .track and .speed_profile_mps.
%
%   Optional Name-Value arguments:
%     'TrackWidth'   - Road width in metres (default: 10.0 m)
%     'Elevation'    - Track z coordinates vector (default: auto-detect Spa or flat)
%     'PlaybackSpeed'- Initial time multiplier (default: 1.0x)
%     'DefaultCam'   - Initial camera: 'chase', 'cockpit', 'orbit', 'overview' (default: 'chase')
%     'CarColor'     - RGB color vector for chassis (default: [0.85, 0.15, 0.15] - racing red)
%
%   Features:
%     - 3D road mesh with asphalt texturing, track boundaries, and kerbs
%     - Detailed 3D Formula car model with wings, halo, and wheels
%     - 4 Dynamic Camera Modes: Chase Cam, Cockpit View, Orbit Follow, Overview
%     - Live Telemetry HUD: Speedometer, Gear, Tachometer, G-G friction circle,
%       Throttle/Brake pedal inputs, Lap timer, and Top-Down track minimap
%     - Interactive Controls: Play/Pause, Reset, Scrubber, Speed multipliers (0.5x, 1x, 2x, 5x)

% Allow calling simulateCar3D with 0 arguments
if nargin < 1 || isempty(results)
    if evalin('base', 'exist(''results'', ''var'')')
        results = evalin('base', 'results');
    else
        fprintf('\nNo simulation results in workspace. Running Spa-Francorchamps...\n');
        projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
        evalin('base', sprintf('run(''%s'')', fullfile(projectRoot, 'RUN_SPA.m')));
        return;
    end
end

p = inputParser;
addOptional(p, 'results', results, @isstruct);
addParameter(p, 'TrackWidth', 10.0, @isnumeric);
addParameter(p, 'Elevation', [], @isnumeric);
addParameter(p, 'PlaybackSpeed', 1.0, @isnumeric);
addParameter(p, 'DefaultCam', 'chase', @ischar);
addParameter(p, 'CarColor', [0.85, 0.15, 0.15], @isnumeric);
addParameter(p, 'MaxSeconds', Inf, @isnumeric);
parse(p, results, varargin{:});
maxSec  = p.Results.MaxSeconds;

track   = results.track;
speed   = results.speed_profile_mps;
vehicle = results.vehicle;
g0      = 9.81;
N       = numel(track.x);

% 1. Determine Elevation
if ~isempty(p.Results.Elevation) && numel(p.Results.Elevation) == N
    z = p.Results.Elevation(:);
elseif isfield(track, 'z') && ~isempty(track.z) && numel(track.z) == N
    z = track.z(:);
elseif max(track.s) > 5000
    z = generateSpaElevation(track.s(:));
else
    z = zeros(N, 1);
end

% 2. Calculate time vector
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
totalLapTime = max(t);

% 3. Accelerations, Gear, RPM, Throttle, Brake
latG = (speed(:).^2 .* abs(track.kappa(:))) / g0;
lonG = results.longitudinal_acc_mps2(:) / g0;
throttle = max(0, min(1, lonG / 1.5));
brake    = max(0, min(1, -lonG / 2.0));

gears = vehicle.gear_ratios;
fd    = vehicle.final_drive;
r_w   = vehicle.wheel_radius_m;
gearSeries = ones(N, 1);
rpmSeries  = zeros(N, 1);
for i = 1:N
    v = speed(i);
    w_wheel = v / r_w;
    bestGear = 1;
    bestRpm = 4000;
    for gIdx = 1:numel(gears)
        testRpm = w_wheel * gears(gIdx) * fd * 60 / (2 * pi);
        if testRpm <= 12500 && testRpm >= 3500
            bestGear = gIdx; bestRpm = testRpm;
        elseif testRpm < 3500 && gIdx == 1
            bestGear = 1; bestRpm = max(2500, testRpm);
        end
    end
    gearSeries(i) = bestGear;
    rpmSeries(i)  = round(min(12500, max(2500, bestRpm)));
end

% 4. Tangent and Normal vectors along 3D track
dx = gradient(track.x(:));
dy = gradient(track.y(:));
dz = gradient(z);
ds_vec = hypot(dx, dy) + 1e-12;
nx = -dy ./ ds_vec;
ny =  dx ./ ds_vec;

yaw   = atan2(dy, dx);
pitch = atan2(dz, ds_vec);

% Build Road Ribbon Mesh
w = p.Results.TrackWidth;
xl = track.x(:) + (w/2) * nx;  yl = track.y(:) + (w/2) * ny;  zl = z;
xr = track.x(:) - (w/2) * nx;  yr = track.y(:) - (w/2) * ny;  zr = z;

X_road = [xl, xr]';
Y_road = [yl, yr]';
Z_road = [zl, zr]';
C_road = [speed(:), speed(:)]';

% 5. Create Simulation Figure
fig = figure('Name', 'Vehicle Lap Simulation 3D', ...
             'NumberTitle', 'off', ...
             'Color', [0.08, 0.09, 0.12], ...
             'Position', [80, 80, 1280, 760]);

% Main 3D Axes
ax3D = axes('Parent', fig, ...
            'Position', [0.02, 0.08, 0.74, 0.88], ...
            'Color', [0.07, 0.08, 0.10]);
hold(ax3D, 'on');
grid(ax3D, 'on');
ax3D.GridColor = [0.25, 0.28, 0.35];
ax3D.GridAlpha = 0.4;
axis(ax3D, 'equal');
xlabel(ax3D, 'X (m)', 'Color', [0.7, 0.7, 0.7]);
ylabel(ax3D, 'Y (m)', 'Color', [0.7, 0.7, 0.7]);
zlabel(ax3D, 'Elevation (m)', 'Color', [0.7, 0.7, 0.7]);
ax3D.XColor = [0.5, 0.5, 0.5];
ax3D.YColor = [0.5, 0.5, 0.5];
ax3D.ZColor = [0.5, 0.5, 0.5];

% Draw Road Surface
surf(ax3D, X_road, Y_road, Z_road, C_road, ...
     'EdgeColor', 'none', 'FaceAlpha', 0.96);
colormap(ax3D, turbo);
cb = colorbar(ax3D, 'Location', 'southoutside');
cb.Position = [0.05, 0.12, 0.25, 0.015];
cb.Color = [0.9, 0.9, 0.9];
cb.Label.String = 'Speed (m/s)';
cb.Label.Color = [0.9, 0.9, 0.9];

% Draw White Track Outer/Inner Edges
plot3(ax3D, xl, yl, zl, 'w-', 'LineWidth', 1.8);
plot3(ax3D, xr, yr, zr, 'w-', 'LineWidth', 1.8);

% Draw Centerline / Racing line
plot3(ax3D, track.x, track.y, z + 0.05, ':', 'Color', [1 1 1 0.4], 'LineWidth', 1.2);

% Draw Start/Finish Gantry Marker
s0_x = [xl(1), xr(1)];
s0_y = [yl(1), yr(1)];
s0_z = [zl(1), zr(1)];
plot3(ax3D, s0_x, s0_y, s0_z + 0.08, 'w-', 'LineWidth', 4);

% Apex Kerbs on high curvature sections
isApex = abs(track.kappa) > 0.0035;
if any(isApex)
    plot3(ax3D, xl(isApex), yl(isApex), zl(isApex)+0.06, 'r.', 'MarkerSize', 10);
    plot3(ax3D, xr(isApex), yr(isApex), zr(isApex)+0.06, 'r.', 'MarkerSize', 10);
end

% 6. Build 3D Car Model attached to an hgtransform
carTransform = hgtransform('Parent', ax3D);
buildCarMesh(carTransform, p.Results.CarColor);

% Trailing Tire Skid/Smoke Mark Handle
trailHandle = plot3(ax3D, NaN, NaN, NaN, 'g-', 'LineWidth', 2.5);

% 7. Telemetry HUD Panels (Top Right)
hudPanel = uipanel('Parent', fig, ...
                   'Position', [0.77, 0.08, 0.21, 0.88], ...
                   'BackgroundColor', [0.12, 0.14, 0.18], ...
                   'BorderType', 'line', ...
                   'HighlightColor', [0.25, 0.28, 0.35]);

% HUD Title
uicontrol('Parent', hudPanel, 'Style', 'text', ...
          'String', 'TELEMETRY HUD', ...
          'Units', 'normalized', 'Position', [0.05, 0.94, 0.90, 0.04], ...
          'BackgroundColor', [0.12, 0.14, 0.18], 'ForegroundColor', [0.3, 0.8, 1.0], ...
          'FontSize', 12, 'FontWeight', 'bold');

% Speed Display
speedText = uicontrol('Parent', hudPanel, 'Style', 'text', ...
                      'String', '0 km/h', ...
                      'Units', 'normalized', 'Position', [0.05, 0.87, 0.90, 0.06], ...
                      'BackgroundColor', [0.12, 0.14, 0.18], 'ForegroundColor', [1.0, 1.0, 1.0], ...
                      'FontSize', 18, 'FontWeight', 'bold');

% Gear & RPM Display
gearText = uicontrol('Parent', hudPanel, 'Style', 'text', ...
                     'String', 'GEAR: 1  |  4000 RPM', ...
                     'Units', 'normalized', 'Position', [0.05, 0.82, 0.90, 0.04], ...
                     'BackgroundColor', [0.12, 0.14, 0.18], 'ForegroundColor', [1.0, 0.8, 0.2], ...
                     'FontSize', 10, 'FontWeight', 'bold');

% Lap Time Display
timeText = uicontrol('Parent', hudPanel, 'Style', 'text', ...
                     'String', 'LAP TIME: 00:00.00', ...
                     'Units', 'normalized', 'Position', [0.05, 0.77, 0.90, 0.04], ...
                     'BackgroundColor', [0.12, 0.14, 0.18], 'ForegroundColor', [0.8, 0.9, 0.8], ...
                     'FontSize', 10);

% Distance Progress Display
distText = uicontrol('Parent', hudPanel, 'Style', 'text', ...
                     'String', 'DIST: 0 / 7004 m (0%)', ...
                     'Units', 'normalized', 'Position', [0.05, 0.72, 0.90, 0.04], ...
                     'BackgroundColor', [0.12, 0.14, 0.18], 'ForegroundColor', [0.7, 0.7, 0.7], ...
                     'FontSize', 9);

% Throttle and Brake Bars Axes
pedalAx = axes('Parent', hudPanel, 'Position', [0.12, 0.58, 0.76, 0.12], ...
               'Color', [0.08, 0.09, 0.11]);
hold(pedalAx, 'on');
barThrottle = barh(pedalAx, 1, 0, 'FaceColor', [0.2, 0.8, 0.2], 'BarWidth', 0.5);
barBrake    = barh(pedalAx, 2, 0, 'FaceColor', [0.9, 0.2, 0.2], 'BarWidth', 0.5);
xlim(pedalAx, [0, 1]); ylim(pedalAx, [0.4, 2.6]);
pedalAx.YTick = [1, 2]; pedalAx.YTickLabel = {'THR', 'BRK'};
pedalAx.XColor = [0.5, 0.5, 0.5]; pedalAx.YColor = [0.9, 0.9, 0.9];
title(pedalAx, 'Pedals (Throttle / Brake)', 'Color', [0.8, 0.8, 0.8], 'FontSize', 8);

% G-G Diagram Inset Axes (Friction Circle)
ggAx = axes('Parent', hudPanel, 'Position', [0.18, 0.32, 0.64, 0.21], ...
            'Color', [0.08, 0.09, 0.11]);
hold(ggAx, 'on'); axis(ggAx, 'equal');
theta = linspace(0, 2*pi, 100);
plot(ggAx, 1.0*cos(theta), 1.0*sin(theta), 'Color', [0.3, 0.4, 0.5], 'LineWidth', 0.8);
plot(ggAx, 1.6*cos(theta), 1.6*sin(theta), '--', 'Color', [0.5, 0.6, 0.7], 'LineWidth', 1.0);
plot(ggAx, [-2, 2], [0, 0], 'Color', [0.3, 0.3, 0.3]);
plot(ggAx, [0, 0], [-2, 2], 'Color', [0.3, 0.3, 0.3]);
xlim(ggAx, [-2.2, 2.2]); ylim(ggAx, [-2.2, 2.2]);
ggPoint = plot(ggAx, 0, 0, 'yo', 'MarkerFaceColor', 'y', 'MarkerSize', 7);
ggAx.XColor = [0.5, 0.5, 0.5]; ggAx.YColor = [0.5, 0.5, 0.5];
title(ggAx, 'G-Force Friction Circle', 'Color', [0.8, 0.8, 0.8], 'FontSize', 8);
xlabel(ggAx, 'Lat (g)', 'Color', [0.7, 0.7, 0.7], 'FontSize', 7);
ylabel(ggAx, 'Lon (g)', 'Color', [0.7, 0.7, 0.7], 'FontSize', 7);

% Top-Down 2D Minimap Inset Axes
mapAx = axes('Parent', hudPanel, 'Position', [0.12, 0.04, 0.76, 0.24], ...
             'Color', [0.08, 0.09, 0.11]);
hold(mapAx, 'on'); axis(mapAx, 'equal');
plot(mapAx, track.x, track.y, 'Color', [0.4, 0.45, 0.55], 'LineWidth', 1.2);
mapBlip = plot(mapAx, track.x(1), track.y(1), 'co', 'MarkerFaceColor', 'c', 'MarkerSize', 6);
mapAx.Visible = 'off';
title(mapAx, 'Track Radar', 'Color', [0.8, 0.8, 0.8], 'FontSize', 8);

% 8. Interactive Control Buttons at Bottom
btnPlay = uicontrol('Parent', fig, 'Style', 'pushbutton', ...
                    'String', 'PAUSE', ...
                    'Units', 'normalized', 'Position', [0.02, 0.02, 0.08, 0.04], ...
                    'BackgroundColor', [0.2, 0.25, 0.32], 'ForegroundColor', [1 1 1], ...
                    'FontWeight', 'bold');

btnReset = uicontrol('Parent', fig, 'Style', 'pushbutton', ...
                     'String', 'RESET', ...
                     'Units', 'normalized', 'Position', [0.11, 0.02, 0.07, 0.04], ...
                     'BackgroundColor', [0.2, 0.25, 0.32], 'ForegroundColor', [1 1 1]);

% Camera Mode Dropdown
uicontrol('Parent', fig, 'Style', 'text', 'String', 'Camera:', ...
          'Units', 'normalized', 'Position', [0.19, 0.02, 0.05, 0.035], ...
          'BackgroundColor', [0.08, 0.09, 0.12], 'ForegroundColor', [0.8, 0.8, 0.8]);
camMenu = uicontrol('Parent', fig, 'Style', 'popupmenu', ...
                    'String', {'Chase Cam', 'Cockpit Cam', 'Orbit Follow', 'Track Overview'}, ...
                    'Units', 'normalized', 'Position', [0.24, 0.02, 0.11, 0.04], ...
                    'BackgroundColor', [0.2, 0.25, 0.32], 'ForegroundColor', [1 1 1]);

% Set default camera selection
switch lower(p.Results.DefaultCam)
    case 'cockpit',  camMenu.Value = 2;
    case 'orbit',    camMenu.Value = 3;
    case 'overview', camMenu.Value = 4;
    otherwise,       camMenu.Value = 1;
end

% Speed multiplier buttons
uicontrol('Parent', fig, 'Style', 'text', 'String', 'Speed:', ...
          'Units', 'normalized', 'Position', [0.36, 0.02, 0.04, 0.035], ...
          'BackgroundColor', [0.08, 0.09, 0.12], 'ForegroundColor', [0.8, 0.8, 0.8]);
speedMenu = uicontrol('Parent', fig, 'Style', 'popupmenu', ...
                      'String', {'0.5x', '1.0x Realtime', '2.0x Fast', '5.0x Turbo'}, ...
                      'Value', 2, ...
                      'Units', 'normalized', 'Position', [0.41, 0.02, 0.10, 0.04], ...
                      'BackgroundColor', [0.2, 0.25, 0.32], 'ForegroundColor', [1 1 1]);

% Progress Slider
sliderProg = uicontrol('Parent', fig, 'Style', 'slider', ...
                       'Min', 0, 'Max', totalLapTime, 'Value', 0, ...
                       'Units', 'normalized', 'Position', [0.52, 0.02, 0.24, 0.04], ...
                       'BackgroundColor', [0.2, 0.25, 0.32]);

% 9. State and Callback bindings
state = struct();
state.isRunning = true;
state.simTime   = 0;
state.speedMult = p.Results.PlaybackSpeed;
state.trailX    = [];
state.trailY    = [];
state.trailZ    = [];

btnPlay.Callback  = @(s, e) togglePlay();
btnReset.Callback = @(s, e) resetSim();
camMenu.Callback  = @(s, e) updateCamMode();
speedMenu.Callback = @(s, e) updateSpeedMult();
sliderProg.Callback = @(s, e) scrubTime();

    function togglePlay()
        state.isRunning = ~state.isRunning;
        if state.isRunning
            btnPlay.String = 'PAUSE';
        else
            btnPlay.String = 'PLAY';
        end
    end

    function resetSim()
        state.simTime = 0;
        state.trailX = []; state.trailY = []; state.trailZ = [];
    end

    function updateCamMode()
        if camMenu.Value == 4
            rotate3d(ax3D, 'on');
        else
            rotate3d(ax3D, 'off');
        end
    end

    function updateSpeedMult()
        speeds = [0.5, 1.0, 2.0, 5.0];
        state.speedMult = speeds(speedMenu.Value);
    end

    function scrubTime()
        state.simTime = sliderProg.Value;
        state.trailX = []; state.trailY = []; state.trailZ = [];
    end

% 10. Animation Loop
fps = 30;
dt_frame = 1 / fps;
t_clock = tic;

while isvalid(fig) && (isinf(maxSec) || toc(t_clock) < maxSec)
    t_loop_start = toc(t_clock);
    
    if state.isRunning
        state.simTime = state.simTime + dt_frame * state.speedMult;
        if state.simTime > totalLapTime
            state.simTime = 0; % Loop back to start
            state.trailX = []; state.trailY = []; state.trailZ = [];
        end
    end
    
    curTime = state.simTime;
    sliderProg.Value = min(curTime, totalLapTime);
    
    % Interpolate vehicle states at curTime
    cur_x     = interp1(t, track.x, curTime, 'linear', 'extrap');
    cur_y     = interp1(t, track.y, curTime, 'linear', 'extrap');
    cur_z     = interp1(t, z, curTime, 'linear', 'extrap');
    cur_yaw   = interp1(t, unwrap(yaw), curTime, 'linear', 'extrap');
    cur_pitch = interp1(t, pitch, curTime, 'linear', 'extrap');
    cur_speed = interp1(t, speed, curTime, 'linear', 'extrap');
    cur_latG  = interp1(t, latG, curTime, 'linear', 'extrap');
    cur_lonG  = interp1(t, lonG, curTime, 'linear', 'extrap');
    cur_s     = interp1(t, track.s, curTime, 'linear', 'extrap');
    cur_rpm   = round(interp1(t, rpmSeries, curTime, 'nearest', 'extrap'));
    cur_gear  = round(interp1(t, gearSeries, curTime, 'nearest', 'extrap'));
    cur_thr   = interp1(t, throttle, curTime, 'linear', 'extrap');
    cur_brk   = interp1(t, brake, curTime, 'linear', 'extrap');
    
    % Update 3D Car Transformation Matrix
    M_trans = makehgtform('translate', [cur_x, cur_y, cur_z + 0.15]);
    M_rot_z = makehgtform('zrotate', cur_yaw);
    M_rot_y = makehgtform('yrotate', -cur_pitch);
    set(carTransform, 'Matrix', M_trans * M_rot_z * M_rot_y);
    
    % Update Trailing Tire Marks
    state.trailX = [state.trailX, cur_x];
    state.trailY = [state.trailY, cur_y];
    state.trailZ = [state.trailZ, cur_z + 0.05];
    if numel(state.trailX) > 40
        state.trailX(1) = []; state.trailY(1) = []; state.trailZ(1) = [];
    end
    set(trailHandle, 'XData', state.trailX, 'YData', state.trailY, 'ZData', state.trailZ);
    
    % Update Camera View
    switch camMenu.Value
        case 1 % Chase Cam
            d_back = 12.0; h_cam = 4.0; d_look = 16.0;
            cam_x = cur_x - d_back * cos(cur_yaw);
            cam_y = cur_y - d_back * sin(cur_yaw);
            cam_z = cur_z + h_cam;
            tgt_x = cur_x + d_look * cos(cur_yaw);
            tgt_y = cur_y + d_look * sin(cur_yaw);
            tgt_z = cur_z + 1.0;
            campos(ax3D, [cam_x, cam_y, cam_z]);
            camtarget(ax3D, [tgt_x, tgt_y, tgt_z]);
            camup(ax3D, [0, 0, 1]);
            camva(ax3D, 50);
            
        case 2 % Cockpit Cam
            cam_x = cur_x + 0.1 * cos(cur_yaw);
            cam_y = cur_y + 0.1 * sin(cur_yaw);
            cam_z = cur_z + 0.8;
            tgt_x = cur_x + 30.0 * cos(cur_yaw);
            tgt_y = cur_y + 30.0 * sin(cur_yaw);
            tgt_z = cur_z + 0.8 + 30.0 * sin(cur_pitch);
            campos(ax3D, [cam_x, cam_y, cam_z]);
            camtarget(ax3D, [tgt_x, tgt_y, tgt_z]);
            camup(ax3D, [0, 0, 1]);
            camva(ax3D, 65);
            
        case 3 % Orbit Follow Cam
            d_orb = 28.0; h_orb = 18.0;
            campos(ax3D, [cur_x + d_orb*0.7, cur_y - d_orb*0.7, cur_z + h_orb]);
            camtarget(ax3D, [cur_x, cur_y, cur_z]);
            camup(ax3D, [0, 0, 1]);
            camva(ax3D, 40);
            
        case 4 % Track Overview (User can rotate)
            % Keep existing user camera position, just update target if needed
    end
    
    % Update Telemetry HUD
    speed_kmh = cur_speed * 3.6;
    speedText.String = sprintf('%.0f km/h  (%.1f m/s)', speed_kmh, cur_speed);
    gearText.String  = sprintf('GEAR: %d  |  %d RPM', cur_gear, cur_rpm);
    
    mins = floor(curTime / 60);
    secs = curTime - mins * 60;
    timeText.String = sprintf('LAP TIME: %02d:%05.2f', mins, secs);
    distText.String = sprintf('DIST: %.0f / %.0f m (%.1f%%)', ...
                              cur_s, max(track.s), (cur_s/max(track.s))*100);
    
    % Update Pedals
    barThrottle.XData = cur_thr;
    barBrake.XData    = cur_brk;
    
    % Update G-G crosshair
    set(ggPoint, 'XData', cur_latG, 'YData', cur_lonG);
    
    % Update Radar Minimap Blip
    set(mapBlip, 'XData', cur_x, 'YData', cur_y);
    
    drawnow limitrate;
    
    % Frame Rate Limiter
    elapsed = toc(t_clock) - t_loop_start;
    sleep_time = dt_frame - elapsed;
    if sleep_time > 0.001
        pause(sleep_time);
    end
end
end

% -------------------------------------------------------------------------
% Helper: Construct 3D Formula / FSAE Car Model
% -------------------------------------------------------------------------
function buildCarMesh(hParent, bodyColor)
% Scale in metres: length ~ 3.0 m, width ~ 1.4 m, height ~ 0.8 m

% 1. Main Chassis & Nosecone
nose_x = [ 1.5,  0.6,  0.6, -1.2, -1.2,  0.6,  0.6,  1.5 ];
nose_y = [ 0.0,  0.25, -0.25, -0.35,  0.35,  0.25, -0.25,  0.0 ];
nose_z = [ 0.15, 0.40,  0.40,  0.45,  0.45,  0.40,  0.40,  0.15 ];
patch('Parent', hParent, 'XData', nose_x, 'YData', nose_y, 'ZData', nose_z, ...
      'FaceColor', bodyColor, 'EdgeColor', [0.1, 0.1, 0.1], 'LineWidth', 1.0);

% 2. Front Wing
fw_x = [1.3, 1.6, 1.6, 1.3];
fw_y = [-0.7, -0.7, 0.7, 0.7];
fw_z = [0.1, 0.1, 0.1, 0.1];
patch('Parent', hParent, 'XData', fw_x, 'YData', fw_y, 'ZData', fw_z, ...
      'FaceColor', [0.1, 0.1, 0.1], 'EdgeColor', [0.8, 0.8, 0.8]);

% Front Wing Endplates
patch('Parent', hParent, 'XData', [1.3, 1.6, 1.6, 1.3], 'YData', [0.7, 0.7, 0.7, 0.7], ...
      'ZData', [0.05, 0.05, 0.25, 0.25], 'FaceColor', bodyColor);
patch('Parent', hParent, 'XData', [1.3, 1.6, 1.6, 1.3], 'YData', [-0.7, -0.7, -0.7, -0.7], ...
      'ZData', [0.05, 0.05, 0.25, 0.25], 'FaceColor', bodyColor);

% 3. Rear Wing
rw_x = [-1.3, -1.0, -1.0, -1.3];
rw_y = [-0.65, -0.65, 0.65, 0.65];
rw_z = [0.85, 0.85, 0.85, 0.85];
patch('Parent', hParent, 'XData', rw_x, 'YData', rw_y, 'ZData', rw_z, ...
      'FaceColor', [0.1, 0.1, 0.1], 'EdgeColor', [0.8, 0.8, 0.8]);

% Rear Wing Endplates
patch('Parent', hParent, 'XData', [-1.4, -1.0, -1.0, -1.4], 'YData', [0.65, 0.65, 0.65, 0.65], ...
      'ZData', [0.45, 0.45, 0.95, 0.95], 'FaceColor', bodyColor);
patch('Parent', hParent, 'XData', [-1.4, -1.0, -1.0, -1.4], 'YData', [-0.65, -0.65, -0.65, -0.65], ...
      'ZData', [0.45, 0.45, 0.95, 0.95], 'FaceColor', bodyColor);

% 4. Cockpit / Helmet
[hx, hy, hz] = sphere(10);
r_helm = 0.14;
surf(r_helm*hx - 0.1, r_helm*hy, r_helm*hz + 0.52, ...
     'Parent', hParent, 'FaceColor', [1.0, 0.85, 0.0], 'EdgeColor', 'none'); % Yellow driver helmet

% 5. 4 Wheels (Front & Rear)
wheel_pos = [ ...
     0.9,  0.65, 0.20; ... % Front Left
     0.9, -0.65, 0.20; ... % Front Right
    -0.8,  0.65, 0.22; ... % Rear Left
    -0.8, -0.65, 0.22  ... % Rear Right
];

r_tire = 0.23;
w_tire = 0.16;
[cx, cy, cz] = cylinder(r_tire, 12);
% Align cylinder along Y-axis for wheel width
for wIdx = 1:4
    wx = wheel_pos(wIdx, 1);
    wy = wheel_pos(wIdx, 2);
    wz = wheel_pos(wIdx, 3);
    surf(wx + cx*0, wy + (cz - 0.5)*w_tire, wz + cy, ...
         'Parent', hParent, 'FaceColor', [0.05, 0.05, 0.05], 'EdgeColor', [0.2, 0.2, 0.2]);
end
end
