function tests = testTrackGeometry
%TESTTRACKGEOMETRY Verify distance and curvature calculations on synthetic tracks.
%   Run with: runtests('tests')
tests = functiontests(localfunctions);
end

function testStraightLine(testCase)
    % Straight line: curvature should be ~0, distance should be exact
    x = (0:10)';
    y = zeros(size(x));
    track = calculateTrackGeometry(x, y);
    verifyEqual(testCase, track.kappa, zeros(size(x)), 'AbsTol', 1e-6);
    verifyEqual(testCase, track.s(end), 10, 'AbsTol', 1e-6);
end

function testQuarterCircle(testCase)
    % Quarter circle, radius 10 m: kappa should be ~1/r = 0.1, length ~pi/2*r
    theta = linspace(0, pi/2, 50)';
    r = 10;
    x = r * cos(theta);
    y = r * sin(theta);
    track = calculateTrackGeometry(x, y);
    % Check interior points (smoothing affects end regions)
    interior = 5:numel(track.kappa)-5;
    verifyEqual(testCase, abs(track.kappa(interior)), 0.1*ones(numel(interior),1), 'AbsTol', 5e-3);
    % Arc length
    verifyEqual(testCase, track.s(end), (pi/2)*r, 'AbsTol', 0.1);
end

function testArcLengthAccumulates(testCase)
    % Cumulative distance should be monotonically increasing
    x = linspace(0, 100, 30)' + randn(30,1)*2;
    y = linspace(0, 50,  30)' + randn(30,1)*2;
    track = calculateTrackGeometry(x, y);
    verifyGreaterThan(testCase, min(diff(track.s)), 0);
end
