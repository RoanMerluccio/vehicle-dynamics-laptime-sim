function speed = calculateSpeedProfile(track, vehicle, v_corner)
%CALCULATESPEEDPROFILE Compute feasible speed profile using forward/backward passes.

MAX_ITER = 8;
CONV_TOL = 0.01;   % m/s

ds       = track.ds(:);
kappa    = track.kappa(:);
N        = numel(track.s);
v_corner = v_corner(:);

% Initial guess: mean finite cornering speed (physically motivated)
v_init = mean(v_corner(isfinite(v_corner)));
speed  = min(v_corner, v_init);
speed(isinf(speed)) = v_init;

for iter = 1:MAX_ITER
    speed_prev = speed;

    % --- Forward pass (acceleration) ---
    % Stepping forward along track: v_f = sqrt(v_i^2 + 2 * a * ds)
    v_fwd    = zeros(N,1);
    v_fwd(1) = speed(end);      % closed track loop: lap entry = previous lap exit
    for i = 2:N
        v = max(v_fwd(i-1), 0.1);
        f = calculateVehicleForces(vehicle, v);

        % Friction circle (Pythagorean theorem): lateral grip reduces available forward traction
        F_lat   = vehicle.mass_kg * v^2 * abs(kappa(i-1));
        F_avail = sqrt(max(f.F_trac_max^2 - F_lat^2, 0));

        % Net acceleration force: engine drive minus drag and rolling resistance
        F_net   = max(min(f.F_power, F_avail) - f.F_drag - f.F_roll, 0);
        a       = F_net / vehicle.mass_kg;  % Newton's 2nd Law (a = F / m)

        % Kinematics formula: v_f = sqrt(v_i^2 + 2 * a * ds)
        v_fwd(i) = min(sqrt(v^2 + 2*a*ds(i-1)), v_corner(i));
    end

    % --- Backward pass (braking) ---
    % Stepping backward from corner entry using maximum braking deceleration
    v_bwd      = zeros(N,1);
    v_bwd(end) = v_fwd(end);
    for i = N-1:-1:1
        v = max(v_bwd(i+1), 0.1);
        f = calculateVehicleForces(vehicle, v);

        % Friction circle during braking
        F_lat   = vehicle.mass_kg * v^2 * abs(kappa(i));
        F_bk    = sqrt(max(f.F_brake_max^2 - F_lat^2, 0));

        % Total braking deceleration: brakes + aero drag + rolling resistance
        a_brake = (F_bk + f.F_drag + f.F_roll) / vehicle.mass_kg;

        % Kinematics formula stepping backward
        v_bwd(i) = min(sqrt(v^2 + 2*a_brake*ds(i)), v_corner(i));
    end

    % Final speed is capped by cornering, acceleration, and braking limits
    speed = min(v_corner, min(v_fwd, v_bwd));
    speed(speed < 0 | isnan(speed)) = 0;

    if max(abs(speed - speed_prev)) < CONV_TOL; break; end
end
end
