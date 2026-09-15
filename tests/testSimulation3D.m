function tests = testSimulation3D
%TESTSIMULATION3D Verify 3D simulation components, elevation generator, and export.
%   Run with: runtests('tests/testSimulation3D')
tests = functiontests(localfunctions);
end

function testSpaElevationProfile(testCase)
    s = linspace(0, 7004, 500);
    z = generateSpaElevation(s);
    
    testCase.verifyEqual(numel(z), numel(s), 'Elevation vector length must match input s');
    testCase.verifyGreaterThanOrEqual(min(z), 370, 'Spa lowest point should be ~372 m');
    testCase.verifyLessThanOrEqual(max(z), 480, 'Spa peak altitude should be ~473.2 m');
    testCase.verifyEqual(z(1), z(end), 'AbsTol', 1e-3, 'Loop track start and end elevations must match');
end

function testExportSimulationData(testCase)
    % Setup synthetic track & vehicle
    track.x = [0; 100; 200; 100; 0];
    track.y = [0; 0; 100; 100; 0];
    track.s = [0; 100; 241; 341; 441];
    track.ds = diff(track.s);
    track.kappa = [0; 0.01; 0.02; 0.01; 0];
    
    speed = [20; 25; 22; 24; 20];
    results.track = track;
    results.speed_profile_mps = speed;
    results.lap_time_s = 20.5;
    results.average_speed_mps = 21.5;
    results.max_speed_mps = 25.0;
    results.lateral_acc_mps2 = speed.^2 .* abs(track.kappa);
    results.longitudinal_acc_mps2 = [1; -1; 0.5; -1.5; 0];
    
    vehicle.gear_ratios = [3.6, 2.4, 1.8, 1.4, 1.1];
    vehicle.final_drive = 3.8;
    vehicle.wheel_radius_m = 0.230;
    results.vehicle = vehicle;
    
    tmpFile = fullfile(tempdir, 'test_sim_data.json');
    jsonData = exportSimulationData(results, tmpFile);
    
    testCase.verifyNotEmpty(jsonData, 'Export JSON data must not be empty');
    decoded = jsondecode(jsonData);
    testCase.verifyTrue(isfield(decoded, 'telemetry'), 'JSON must contain telemetry struct');
    testCase.verifyEqual(numel(decoded.telemetry.speed_kmh), numel(track.x), 'Telemetry points match track points');
    
    if exist(tmpFile, 'file')
        delete(tmpFile);
    end
end

function testSimulateCar3DBatchRun(testCase)
    % Setup synthetic results
    track.x = [0; 50; 100; 50; 0];
    track.y = [0; 0; 50; 50; 0];
    track.s = [0; 50; 120; 170; 220];
    track.ds = diff(track.s);
    track.kappa = [0; 0.02; 0.02; 0.02; 0];
    
    speed = [15; 18; 16; 17; 15];
    results.track = track;
    results.speed_profile_mps = speed;
    results.lap_time_s = 12.0;
    results.average_speed_mps = 18.3;
    results.max_speed_mps = 18.0;
    results.lateral_acc_mps2 = speed.^2 .* abs(track.kappa);
    results.longitudinal_acc_mps2 = [1; -0.5; 0.5; -1; 0];
    
    vehicle.gear_ratios = [3.6, 2.4, 1.8, 1.4, 1.1];
    vehicle.final_drive = 3.8;
    vehicle.wheel_radius_m = 0.230;
    results.vehicle = vehicle;
    
    % Run simulation for 0.5s in batch
    fig = simulateCar3D(results, 'PlaybackSpeed', 10, 'MaxSeconds', 0.5);
    testCase.verifyTrue(isvalid(fig), 'Simulation figure should be valid handle');
    close(fig);
end
