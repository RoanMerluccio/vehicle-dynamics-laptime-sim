function tests = testVehicleModel
%TESTVEHICLEMODEL Sanity checks on calculateVehicleForces.
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

function testDragZeroAtRest(testCase)
    f = calculateVehicleForces(baseVehicle, 0);
    verifyEqual(testCase, f.F_drag, 0, 'AbsTol', 1e-10);
end

function testDragIncreasesWithSpeed(testCase)
    v = baseVehicle;
    f1 = calculateVehicleForces(v, 10);
    f2 = calculateVehicleForces(v, 30);
    verifyGreaterThan(testCase, f2.F_drag, f1.F_drag);
end

function testHigherMuIncreasesMaxTraction(testCase)
    v1 = baseVehicle; v1.tire_mu = 0.8;
    v2 = baseVehicle; v2.tire_mu = 1.4;
    f1 = calculateVehicleForces(v1, 15);
    f2 = calculateVehicleForces(v2, 15);
    verifyGreaterThan(testCase, f2.F_trac_max, f1.F_trac_max);
end

function testHigherPowerIncreasesWheelForce(testCase)
    v1 = baseVehicle; v1.power_W = 30000;
    v2 = baseVehicle; v2.power_W = 70000;
    f1 = calculateVehicleForces(v1, 20);
    f2 = calculateVehicleForces(v2, 20);
    verifyGreaterThan(testCase, f2.F_power, f1.F_power);
end

function testNoNanOrInf(testCase)
    v = baseVehicle;
    for spd = [0, 1, 10, 30, 50]
        f = calculateVehicleForces(v, spd);
        for fn = fieldnames(f)'
            val = f.(fn{1});
            verifyFalse(testCase, any(isnan(val(:))), ...
                sprintf('NaN in %s at v=%g', fn{1}, spd));
            verifyFalse(testCase, any(isinf(val(:))), ...
                sprintf('Inf in %s at v=%g', fn{1}, spd));
        end
    end
end
