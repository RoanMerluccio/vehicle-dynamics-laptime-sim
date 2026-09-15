% ==========================================================
%  Vehicle Dynamics & Lap-Time Simulator
%  RUN_3D.m  -  1-Click 3D Car Simulation & HUD
% ==========================================================
%
%  HOW TO RUN:
%    In the MATLAB Command Window, simply type:
%      RUN_3D
%    and press Enter!
%
%  This script will:
%    1. Load Circuit de Spa-Francorchamps
%    2. Run the vehicle dynamics and speed simulation
%    3. Launch the 3D car animation with live telemetry HUD!

projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(projectRoot, 'src')));
cd(projectRoot);

fprintf('\n==============================================\n');
fprintf('  STARTING 3D VEHICLE LAP SIMULATION\n');
fprintf('  Circuit de Spa-Francorchamps (7.004 km)\n');
fprintf('==============================================\n');

% Run Spa physics simulation and launch 3D animation
RUN_SPA;
