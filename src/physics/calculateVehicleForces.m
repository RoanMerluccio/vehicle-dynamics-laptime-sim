function forces = calculateVehicleForces(vehicle, v)
%CALCULATEVEHICLEFORCES Compute drag, downforce, traction limits, and drive forces at speed v.
%
%   Supports two tire friction models (set vehicle.tire_model):
%     'linear'   - Coulomb friction: F = mu * Fn  (default, backward compatible)
%     'pacejka'  - Pacejka Magic Formula 94: nonlinear, load-sensitive
%                  Uses calculatePacejkaTire() with optional vehicle.pacejka_coeffs

g    = 9.81;
C_rr = 0.015;
v    = v(:);
N    = numel(v);

% Aerodynamic forces: 0.5 * rho * Area * v^2
qA = 0.5 * vehicle.air_density_kgm3 * vehicle.frontal_area_m2 .* v.^2;
F_drag = vehicle.Cd .* qA;                           % Drag opposes forward motion
F_down = vehicle.Cl .* qA;                           % Downforce pushes car into the road

% Normal force includes vehicle weight + aero downforce
F_normal   = vehicle.mass_kg * g + F_down;

% Tire friction model selection
tire_model = 'linear';
if isfield(vehicle, 'tire_model')
    tire_model = vehicle.tire_model;
end

switch lower(tire_model)
    case 'pacejka'
        % Pacejka Magic Formula 94: peak mu varies with normal load
        % This captures load-sensitivity (degressive behaviour at high Fz).
        pcoeffs = [];
        if isfield(vehicle, 'pacejka_coeffs'); pcoeffs = vehicle.pacejka_coeffs; end
        % calculatePacejkaTire's load-sensitivity curve (Fz_ref = 1500 N) is
        % calibrated per tire; F_normal here is the whole car, so divide by
        % 4 corners before evaluating it.
        mu_eff = zeros(N,1);
        for ki = 1:N
            [~, mu_eff(ki)] = calculatePacejkaTire(F_normal(ki) / 4, [], pcoeffs);
        end
        F_trac_max = mu_eff .* F_normal;
    otherwise % 'linear' (Coulomb)
        F_trac_max = vehicle.tire_mu .* F_normal;    % F = mu * N
end

F_roll = C_rr * vehicle.mass_kg * g * ones(N,1); % Rolling resistance

% Available engine drive force
if isfield(vehicle,'torque_rpm') && isfield(vehicle,'torque_Nm') ...
        && isfield(vehicle,'gear_ratios') && isfield(vehicle,'final_drive')
    % Gearbox: Wheel force = (Engine Torque * Gear Ratio * Efficiency) / Wheel Radius
    eta   = vehicle.transmission_eff;
    r     = vehicle.wheel_radius_m;
    gears = vehicle.gear_ratios;
    fd    = vehicle.final_drive;
    F_power = zeros(N,1);
    for i = 1:N
        omega_w = max(v(i), 0.1) / r;   % Wheel angular speed (rad/s)
        best_F  = 0;
        for k = 1:numel(gears)
            ratio = gears(k) * fd;
            rpm   = omega_w * ratio * 60 / (2*pi);
            if rpm > max(vehicle.torque_rpm)
                continue; % over-revved in this gear; the car would have shifted up already
            end
            rpm   = max(rpm, min(vehicle.torque_rpm)); % idle/lugging floor
            T     = interp1(vehicle.torque_rpm, vehicle.torque_Nm, rpm, 'linear');
            F_w   = T * ratio * eta / r;
            if F_w > best_F; best_F = F_w; end
        end
        F_power(i) = best_F;
    end
else
    % Constant power model: Power = Force * velocity => Force = Power / v
    F_power = zeros(N,1);
    moving  = v > 1e-3;
    F_power(moving) = vehicle.power_W ./ v(moving);
end

% Cannot exceed tire grip or brake hydraulic limits
F_power     = min(F_power, F_trac_max);
F_brake_max = min(vehicle.brake_max_N, F_trac_max);

forces.F_drag      = F_drag;
forces.F_roll      = F_roll;
forces.F_trac_max  = F_trac_max;
forces.F_power     = F_power;
forces.F_brake_max = F_brake_max;
end
