function [x, y, z, vx, vy, vz, radius, q, omega] = propagate(x, y, z, vx, vy, vz, q, omega, dt, engine_state, M_ctrl)
    S = [x; y; z; vx; vy; vz; q(1); q(2); q(3); q(4); omega(1); omega(2); omega(3)];

    k1 = rk4_derivatives(S, engine_state, M_ctrl);
    k2 = rk4_derivatives(S + 0.5 * dt * k1, engine_state, M_ctrl);
    k3 = rk4_derivatives(S + 0.5 * dt * k2, engine_state, M_ctrl);
    k4 = rk4_derivatives(S + dt * k3, engine_state, M_ctrl);

    S_next = S + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);

    x  = S_next(1); y  = S_next(2); z  = S_next(3);
    vx = S_next(4); vy = S_next(5); vz = S_next(6);
    
    q     = [S_next(7), S_next(8), S_next(9), S_next(10)];
    omega = [S_next(11), S_next(12), S_next(13)];

    q_norm = sqrt(q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2);
    q = q / q_norm;

    radius = sqrt(x^2 + y^2 + z^2);
end

function dS = rk4_derivatives(S, engine_state, M_ctrl)
    wgs84 = wgs84Ellipsoid('meters');
    x_L = S(1); y_L = S(2); z_L = S(3);
    vx_L = S(4); vy_L = S(5); vz_L = S(6);
    q_L = [S(7); S(8); S(9); S(10)];
    omega_L = [S(11); S(12); S(13)];

    mu = 3.986004418e14;
    J2 = 1.08262668e-3;
    R_eq = wgs84.SemimajorAxis;
    
    radius_L = sqrt(x_L^2 + y_L^2 + z_L^2);
    altitude_L = radius_L - R_eq;

    rho_0 = 1.588e-11;
    h_scale = 50000.0;
    rho = rho_0 * exp(-(altitude_L - 400000.0) / h_scale);
    
    S_mid = 0.1 * 0.1;
    Cd = 2.2;
    m = 3.5;

    v_mag_L = sqrt(vx_L^2 + vy_L^2 + vz_L^2);
    a_drag_mag = (0.5 * rho * v_mag_L^2 * Cd * S_mid / m);

    ax_drag = -a_drag_mag * (vx_L / v_mag_L);
    ay_drag = -a_drag_mag * (vy_L / v_mag_L);
    az_drag = -a_drag_mag * (vz_L / v_mag_L);

    r_sq = radius_L^2;
    z_over_r_sq = (z_L / radius_L)^2;
    J2_factor = 1.5 * J2 * (mu / r_sq) * (R_eq / radius_L)^2;

    ax_J2 = -J2_factor * (1.0 - 5.0 * z_over_r_sq) * (x_L / radius_L);
    ay_J2 = -J2_factor * (1.0 - 5.0 * z_over_r_sq) * (y_L / radius_L);
    az_J2 = -J2_factor * (3.0 - 5.0 * z_over_r_sq) * (z_L / radius_L);

    ax_thrust = 0.0; ay_thrust = 0.0; az_thrust = 0.0;

    if engine_state == 1
        T = 0.6;

        a_thrust_mag = T / m;
        Thrust_B = [a_thrust_mag; 0.0; 0.0];

        q0 = q_L(1); q1 = q_L(2); q2 = q_L(3); q3 = q_L(4);

        DCM = [
            1.0 - 2.0*(q2^2 + q3^2),   2.0*(q1*q2 - q0*q3),       2.0*(q1*q3 + q0*q2);
            2.0*(q1*q2 + q0*q3),       1.0 - 2.0*(q1^2 + q3^2),   2.0*(q2*q3 - q0*q1);
            2.0*(q1*q3 - q0*q2),       2.0*(q2*q3 + q0*q1),       1.0 - 2.0*(q1^2 + q2^2)
        ];

        Thrust_ECI = DCM * Thrust_B;

        ax_thrust = Thrust_ECI(1);
        ay_thrust = Thrust_ECI(2);
        az_thrust = Thrust_ECI(3);
    end

    dvx = -(mu * x_L) / (radius_L^3) + ax_drag + ax_J2 + ax_thrust;
    dvy = -(mu * y_L) / (radius_L^3) + ay_drag + ay_J2 + ay_thrust;
    dvz = -(mu * z_L) / (radius_L^3) + az_drag + az_J2 + az_thrust;

    Jx = 0.00067; Jy = 0.042; Jz = 0.042;

    dwx = ((Jy - Jz) * omega_L(2) * omega_L(3) + M_ctrl(1)) / Jx;
    dwy = ((Jz - Jx) * omega_L(3) * omega_L(1) + M_ctrl(2)) / Jy;
    dwz = ((Jx - Jy) * omega_L(1) * omega_L(2) + M_ctrl(3)) / Jz;

    w_x = omega_L(1); w_y = omega_L(2); w_z = omega_L(3);

    dq0 = 0.5 * (-w_x*q_L(2) - w_y*q_L(3) - w_z*q_L(4));
    dq1 = 0.5 * ( w_x*q_L(1) - w_y*q_L(3) + w_z*q_L(4));
    dq2 = 0.5 * ( w_y*q_L(1) + w_z*q_L(2) - w_x*q_L(4));
    dq3 = 0.5 * ( w_z*q_L(1) - w_y*q_L(2) + w_x*q_L(3));

    dS = [vx_L; vy_L; vz_L; dvx; dvy; dvz; dq0; dq1; dq2; dq3; dwx; dwy; dwz];
end
