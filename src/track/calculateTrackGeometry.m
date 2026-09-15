function track = calculateTrackGeometry(x, y)
%CALCULATETRACKGEOMETRY Compute distance and curvature along a track centerline.

x = x(:);
y = y(:);
N = numel(x);
if N < 3
    error('calculateTrackGeometry: at least 3 track points required.');
end

% Segment distance formula: ds = sqrt(dx^2 + dy^2)
ds = hypot(diff(x), diff(y));

% Cumulative distance along the track (Riemann sum approximation of arc length)
s  = [0; cumsum(ds)];

% Curvature: fit a circle through 3 consecutive points, radius R, curvature kappa = 1/R
% Triangle side vectors between points P(i-1), P(i), P(i+1)
A_x = x(2:N-1) - x(1:N-2);   A_y = y(2:N-1) - y(1:N-2);
B_x = x(3:N)   - x(2:N-1);   B_y = y(3:N)   - y(2:N-1);
C_x = x(3:N)   - x(1:N-2);   C_y = y(3:N)   - y(1:N-2);

% 2 * Triangle Area / (|A| * |B| * |C|) = 1 / Radius = Curvature
cross2D = A_x .* B_y - A_y .* B_x;
denom   = hypot(A_x, A_y) .* hypot(B_x, B_y) .* hypot(C_x, C_y);

k_mid = zeros(N-2, 1);
ok    = denom > 1e-12;                       % avoid divide-by-zero on straights
k_mid(ok) = 2 * cross2D(ok) ./ denom(ok);
kappa = [0; k_mid; 0];                       % track start/finish line

% Smooth to reduce numerical noise
try
    kappa = smoothdata(kappa, 'gaussian', 7);
catch
    kappa = movmean(kappa, 5);
end

track.x     = x;
track.y     = y;
track.s     = s;
track.ds    = ds;
track.kappa = kappa;
end
