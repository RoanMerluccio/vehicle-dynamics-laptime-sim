function v_corner = calculateCorneringSpeed(vehicle, kappa)
%CALCULATE CORNERINGSPEED Maximum cornering speed based on tire grip and downforce.

g     = 9.81;
kappa = kappa(:);

% Aerodynamic grip gain coefficient from downforce
K_aero = vehicle.tire_mu * 0.5 * vehicle.air_density_kgm3 ...
    * vehicle.Cl * vehicle.frontal_area_m2 / vehicle.mass_kg;

kc    = abs(kappa);
denom = kc - K_aero;
limited = (kc > 1e-8) & (denom > 1e-8);

% Straight sections (kappa ~ 0) have no cornering speed limit (Inf)
v_corner = inf(size(kappa));
v_corner(limited) = sqrt(vehicle.tire_mu * g ./ denom(limited));
end
