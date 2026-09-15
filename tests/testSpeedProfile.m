function tests = testSpeedProfile
%TESTSPEEDPROFILE Sanity checks on calculateSpeedProfile and calculateLapTime.
%   run with: runtests('tests')
tests = functiontests(localfunctions);
end

function v = baseVehicle
v.mass_kg          = 250;
v.tire_mu          = 1.0;
v.Cd               = 0.6;
v.Cl               = 0.3;
v.frontal_area_m2  = 0.5;
v.air_density_kgm3 = 1.225;
v.power_W          = 50000;
v.brake_max_N      = 8000;
v.wheel_radius_m   = 0.230;
end

function track = straightTrack(len)
% Simple straight track of given length
x = linspace(0, len, 50)';
y = zeros(50,1);
track = calculateTrackGeometry(x, y);
end

function testSpeedNonNegative(testCase)
    vehicle = baseVehicle;
    track   = straightTrack(200);
    v_corner = calculateCorneringSpeed(vehicle, track.kappa);
    speed    = calculateSpeedProfile(track, vehicle, v_corner);
    verifyGreaterThanOrEqual(testCase, min(speed), 0);
end

function testNoNanInSpeed(testCase)
    vehicle = baseVehicle;
    track   = straightTrack(200);
    v_corner = calculateCorneringSpeed(vehicle, track.kappa);
    speed    = calculateSpeedProfile(track, vehicle, v_corner);
    verifyFalse(testCase, any(isnan(speed)));
end

function testHigherPowerReducesLapTime(testCase)
    v1 = baseVehicle; v1.power_W = 30000;
    v2 = baseVehicle; v2.power_W = 80000;
    track    = straightTrack(300);
    v_corner = calculateCorneringSpeed(v1, track.kappa);
    speed1   = calculateSpeedProfile(track, v1, v_corner);
    speed2   = calculateSpeedProfile(track, v2, v_corner);
    r1 = calculateLapTime(track, speed1);
    r2 = calculateLapTime(track, speed2);
    verifyLessThan(testCase, r2.lap_time_s, r1.lap_time_s);
end

function testHigherMassIncreasesLapTime(testCase)
    v1 = baseVehicle; v1.mass_kg = 200;
    v2 = baseVehicle; v2.mass_kg = 350;
    track = straightTrack(300);
    vc1 = calculateCorneringSpeed(v1, track.kappa);
    vc2 = calculateCorneringSpeed(v2, track.kappa);
    s1 = calculateSpeedProfile(track, v1, vc1);
    s2 = calculateSpeedProfile(track, v2, vc2);
    r1 = calculateLapTime(track, s1);
    r2 = calculateLapTime(track, s2);
    verifyGreaterThan(testCase, r2.lap_time_s, r1.lap_time_s);
end

function testHigherMuIncreasesCorneringSpeed(testCase)
    v1 = baseVehicle; v1.tire_mu = 0.8;
    v2 = baseVehicle; v2.tire_mu = 1.4;
    % Quarter circle, radius 20 m: kappa = 1/20 = 0.05
    theta = linspace(0, pi/2, 50)';
    r = 20;
    x = r*cos(theta); y = r*sin(theta);
    track = calculateTrackGeometry(x, y);
    vc1 = calculateCorneringSpeed(v1, track.kappa);
    vc2 = calculateCorneringSpeed(v2, track.kappa);
    % Mean cornering speed on curve should be higher for higher mu
    verifyGreaterThan(testCase, mean(vc2(~isinf(vc2))), mean(vc1(~isinf(vc1))));
end
