function [X_pred, Y_pred, Z_pred] = predict_maneuver_node(x0, y0, z0, vx0, vy0, vz0) 
    wgs84 = wgs84Ellipsoid('meters');
    mu = 3.986004418e14;        
    R_eq = wgs84.SemimajorAxis; 

    r_mag = sqrt(x0^2 + y0^2 + z0^2);
    v_mag = sqrt(vx0^2 + vy0^2 + vz0^2);

    energy = (v_mag^2 / 2.0) - (mu / r_mag);

    if energy < 0
        a_semi = -mu / (2.0 * energy);
        total_time = 2.0 * pi * sqrt((a_semi^3) / mu);
    else
        total_time = 5600;
    end

    dt_pred = total_time / 150.0;        
    max_steps = 152;

    X_pred = zeros(1, max_steps);
    Y_pred = zeros(1, max_steps);
    Z_pred = zeros(1, max_steps);

    S = [x0; y0; z0; vx0; vy0; vz0];
    
    actual_steps = 0;
    for i = 1:max_steps
        r_curr = sqrt(S(1)^2 + S(2)^2 + S(3)^2);
        if r_curr < (R_eq - 300000.0) 
            break;
        end

        k1 = rk4_ballistic_derivatives(S, mu, R_eq);
        k2 = rk4_ballistic_derivatives(S + 0.5 * dt_pred * k1, mu, R_eq);
        k3 = rk4_ballistic_derivatives(S + 0.5 * dt_pred * k2, mu, R_eq);
        k4 = rk4_ballistic_derivatives(S + dt_pred * k3, mu, R_eq);

        S = S + (dt_pred / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);

        X_pred(i) = S(1);
        Y_pred(i) = S(2);
        Z_pred(i) = S(3);

        actual_steps = i;
    end

    X_pred = X_pred(1:actual_steps);
    Y_pred = Y_pred(1:actual_steps);
    Z_pred = Z_pred(1:actual_steps);
end

function dS = rk4_ballistic_derivatives(S, mu, R_eq)
    x = S(1);  y = S(2);  z = S(3);
    vx = S(4); vy = S(5); vz = S(6);

    r = sqrt(x^2 + y^2 + z^2);
    J2 = 1.08262668e-3;

    r_sq = r^2;
    z_over_r_sq = (z / r)^2;
    J2_factor = 1.5 * J2 * (mu / r_sq) * (R_eq / r)^2;

    ax_J2 = -J2_factor * (1.0 - 5.0 * z_over_r_sq) * (x / r);
    ay_J2 = -J2_factor * (1.0 - 5.0 * z_over_r_sq) * (y / r);
    az_J2 = -J2_factor * (3.0 - 5.0 * z_over_r_sq) * (z / r);

    dvx = -(mu * x) / (r^3) + ax_J2;
    dvy = -(mu * y) / (r^3) + ay_J2;
    dvz = -(mu * z) / (r^3) + az_J2;

    dS = [vx; vy; vz; dvx; dvy; dvz];
end
