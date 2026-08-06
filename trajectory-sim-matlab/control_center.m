clear; clc; close all;

USE_PHYSICAL_HARDWARE = 1;

com_port = 'COM3';
baud_rate = 9600;
is_sil_mode = false;

txPacketSize = 11;
rxPacketSize = 7;

wgs84 = wgs84Ellipsoid('meters');
R_equatorial = wgs84.SemimajorAxis;
initial_altitude = 365000.0;
min_r = initial_altitude;
max_r = initial_altitude;

dt = 1.0;

x = R_equatorial + initial_altitude;
y = 0.0;
z = 0.0;

orbit_radius = sqrt(x^2 + y^2 + z^2);

v_cir = sqrt(3.986004418e14 / x);

v_mag = v_cir * 0.985;

inclination = deg2rad(45);
vx = 0.0;
vy = v_mag * cos(inclination);
vz = v_mag * sin(inclination);

q = [1.0, 0.0, 0.0, 0.0];

omega = [0.04, -0.03, 0.02];

sim_time = uint32(0);
mcu_time = uint32(0);
engine_state = 0;
adcs_state = 0;
cmd_maneuver = 0;
dot_prod = 1.0;
align_byte = 0;
q_error_integral_3d = [0.0, 0.0, 0.0];
M_ctrl = [0.0, 0.0, 0.0];
is_saving_orbit = false;
is_mission_complete = false;
is_circularizing = false;

hFig = figure('Name', 'HIL MCC: Trajectory modelling', ...
              'NumberTitle', 'off', 'Color', [0.1, 0.1, 0.1]);
ax_globe = axes('Parent', hFig, 'Color', [0.1, 0.1, 0.1], 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
hold(ax_globe, 'on');

[Xe, Ye, Ze] = ellipsoid(0, 0, 0, wgs84.SemimajorAxis, wgs84.SemimajorAxis, wgs84.SemiminorAxis, 40);

hEarth = surface(Xe, Ye, Ze, 'Parent', ax_globe, ...
                 'FaceColor', [0.1, 0.2, 0.4], 'EdgeColor', [0.2, 0.4, 0.8], ...
                 'FaceAlpha', 0.6);

grid(ax_globe, 'on');
view(ax_globe, 3);
axis(ax_globe, 'equal');
xlabel(ax_globe, 'X (meters)'); ylabel(ax_globe, 'Y (meters)'); zlabel(ax_globe, 'Z (meters)');
title(ax_globe, 'Satellite movement in (ECI / WGS84)', 'Color', 'w');

hTrajectory = animatedline(ax_globe, 'Color', [0.0, 1.0, 0.0], 'LineWidth', 2);
hSatellite = plot3(ax_globe, x, y, z, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

Maneuver_Node = plot3(ax_globe, x, y, z, 'c--', 'LineWidth', 1.5, 'Tag', 'Maneuver_Node');

hApogee_Marker  = plot3(ax_globe, x, y, z, 'm^', 'MarkerSize', 10, 'MarkerFaceColor', 'm', 'Tag', 'Apogee');
hPerigee_Marker = plot3(ax_globe, x, y, z, 'gv', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'Tag', 'Perigee');

tSatellite = text(ax_globe, x, y, z, '  Sat (Active)', 'Color', 'w', 'FontSize', 9, 'FontWeight', 'bold');
tApogee    = text(ax_globe, x, y, z, '  Apoapsis (Ap)', 'Color', 'm', 'FontSize', 9);
tPerigee   = text(ax_globe, x, y, z, '  Periapsis (Pe)', 'Color', 'g', 'FontSize', 9);

text(ax_globe, x*1.05, y, z, 'Maneuver Node Prediction', 'Color', 'c', 'FontSize', 9, 'HorizontalAlignment', 'left');

ring_buffer = uint8(zeros(1, rxPacketSize));
device = [];
is_hardware_available = false;
is_sil_mode = true;

if USE_PHYSICAL_HARDWARE == 1
    fprintf('\n=================================================================\n');
    fprintf('MCC CONNECT: Initializing RIGID HARDWARE-IN-THE-LOOP (HIL) Mode...\n');
    fprintf('MCC CONNECT: Opening serial communications on %s...\n', com_port);
    fprintf('=================================================================\n');
    try
        device = serialport(com_port, baud_rate, 'Timeout', 5.0);
        configureTerminator(device, "LF");
        setRTS(device, false);
        setDTR(device, false);
        flush(device);
        is_hardware_available = true;
        is_sil_mode = false;
        fprintf(' MCC STATUS: [ HIL MODE ACTIVE ] - Serial link established successfully.\n\n');
    catch ME
        is_hardware_available = false;
        is_sil_mode = true;
        device = [];
        fprintf('\n=================================================================\n');
        fprintf('️  MCC WARNING: Physical COM port %s is unavailable or busy.\n', com_port);
        fprintf('  System Message: %s\n', ME.message);
        fprintf('️  GNC SYSTEM: Fault-tolerant hot-swap into SOFTWARE-IN-THE-LOOP (SIL) Mode!\n');
        fprintf('=================================================================\n\n');
        pause(2.0);
    end
else
    is_hardware_available = false;
    is_sil_mode = true;
    fprintf('\n=================================================================\n');
    fprintf('  MCC STATUS: [ PURE SOFTWARE-IN-THE-LOOP (SIL) MODE ACTIVE ]\n');
    fprintf('  GNC SYSTEM: Running autonomous software-only simulation baseline.\n');
    fprintf('=================================================================\n\n');
end

while true
    t_step_start = tic;

    sim_time = sim_time + 1;

    [~, ~, altitude_wgs84] = ecef2geodetic(wgs84, x, y, z);

    r_vec = [x; y; z];
    v_vec = [vx; vy; vz];

    k_lvlh = -r_vec / norm(r_vec);
    j_lvlh = cross(k_lvlh, v_vec) / norm(cross(k_lvlh, v_vec));
    i_lvlh = cross(j_lvlh, k_lvlh);

    [X_p, Y_p, Z_p] = predict_maneuver_node(x, y, z, vx, vy, vz);

    alt_perigee = 0.0;
    alt_apogee = 0.0;
    mean_predicted_altitude = 0.0;

    if ~isempty(X_p)
        set(Maneuver_Node, 'XData', X_p, 'YData', Y_p, 'ZData', Z_p);

        pred_radii = sqrt(X_p.^2 + Y_p.^2 + Z_p.^2);
        pred_altitudes = pred_radii - wgs84.SemimajorAxis;

        mean_predicted_altitude = mean(pred_altitudes);

        [min_r, idx_perigee] = min(pred_radii);
        alt_perigee = min_r - wgs84.SemimajorAxis;
        x_perigee = X_p(idx_perigee);
        y_perigee = Y_p(idx_perigee);
        z_perigee = Z_p(idx_perigee);
        set(hPerigee_Marker, 'XData', x_perigee, 'YData', y_perigee, 'ZData', z_perigee);
        set(tPerigee, 'Position', [x_perigee, y_perigee, z_perigee]);

        [max_r, idx_apogee] = max(pred_radii);
        alt_apogee = max_r - wgs84.SemimajorAxis;
        x_apogee = X_p(idx_apogee);
        y_apogee = Y_p(idx_apogee);
        z_apogee = Z_p(idx_apogee);
        set(hApogee_Marker, 'XData', x_apogee, 'YData', y_apogee, 'ZData', z_apogee);
        set(tApogee, 'Position', [x_apogee, y_apogee, z_apogee]);
    else
        mean_predicted_altitude = 0.0;
        set(hPerigee_Marker, 'XData', [], 'YData', [], 'ZData', []);
        set(hApogee_Marker, 'XData', [], 'YData', [], 'ZData', []);
    end

    if isempty(is_saving_orbit), is_saving_orbit = false; end

    h_vector = cross([x, y, z], [vx, vy, vz]);
    h_norm = norm(h_vector);

    current_inclination_deg = rad2deg(acos(h_vector(3) / h_norm));

    if isempty(is_circularizing), is_circularizing = false; end

    if (current_inclination_deg >= 89.0) && (current_inclination_deg <= 91.0) && (is_circularizing == false) && (is_mission_complete == false)
        is_circularizing = true;
    end

    apsis_delta = abs(alt_apogee - alt_perigee);

    if is_circularizing && (apsis_delta <= 105000.0 || alt_apogee <= 365000.0)
        is_mission_complete = true;
    end

    if is_mission_complete == true
        cmd_maneuver = 0;
        is_saving_orbit = false;
        is_circularizing = false;
        R_mat = [i_lvlh'; j_lvlh'; k_lvlh'];
    else
        if is_circularizing == true
            R_mat = [-i_lvlh'; -j_lvlh'; k_lvlh'];

            r_curr_dir = [x; y; z] / norm([x, y, z]);
            r_peri_dir = [x_perigee; y_perigee; z_perigee] / norm([x_perigee, y_perigee, z_perigee]);
            cos_peri_window = dot(r_curr_dir, r_peri_dir);

            r_mag_now = norm([x, y, z]);
            v_radial = dot([x, y, z], [vx, vy, vz]) / r_mag_now;

            if (cos_peri_window > 0.88) || (v_radial > 0 && v_radial < 180.0)
                cmd_maneuver = 1;
            else
                cmd_maneuver = 0;
            end
        else
            if (altitude_wgs84 < 350000.0) || (mean_predicted_altitude < 351000.0)
                is_saving_orbit = true;
            end

            if mean_predicted_altitude > 353000.0 && is_saving_orbit
                is_saving_orbit = false;
            end

            if is_saving_orbit == true
                cmd_maneuver = 1;
                R_mat = [i_lvlh'; j_lvlh'; k_lvlh'];
            else
                cmd_maneuver = 1;
                target_X_row = -j_lvlh';
                X_eci = [1.0, 0.0, 0.0];
                target_Y_row = cross(target_X_row, X_eci);
                target_Y_row = target_Y_row / norm(target_Y_row);
                target_Z_row = cross(target_X_row, target_Y_row);
                R_mat = [target_X_row; target_Y_row; target_Z_row];
            end
        end
    end

    tr = trace(R_mat);
    if tr > 0
        S = sqrt(tr + 1.0) * 2;
        q_target0 = 0.25 * S;
        q_target1 = (R_mat(2,3) - R_mat(3,2)) / S;
        q_target2 = (R_mat(3,1) - R_mat(1,3)) / S;
        q_target3 = (R_mat(1,2) - R_mat(2,1)) / S;
    else
        if (R_mat(1,1) > R_mat(2,2)) && (R_mat(1,1) > R_mat(3,3))
            S = sqrt(1.0 + R_mat(1,1) - R_mat(2,2) - R_mat(3,3)) * 2;
            q_target0 = (R_mat(2,3) - R_mat(3,2)) / S;
            q_target1 = 0.25 * S;
            q_target2 = (R_mat(1,2) + R_mat(2,1)) / S;
            q_target3 = (R_mat(1,3) + R_mat(3,1)) / S;
        else
            S = sqrt(1.0 + R_mat(2,2) - R_mat(1,1) - R_mat(3,3)) * 2;
            q_target0 = (R_mat(3,1) - R_mat(1,3)) / S;
            q_target1 = (R_mat(1,2) + R_mat(2,1)) / S;
            q_target2 = 0.25 * S;
            q_target3 = (R_mat(2,3) + R_mat(3,2)) / S;
        end
    end

    q_target = [q_target0, q_target1, q_target2, q_target3];
    q_t_norm = norm(q_target);

    if q_t_norm > 0
        q_target = q_target / q_t_norm;
    end

    sub_steps = 20;
    dt_sub = dt / sub_steps;

    for s = 1:sub_steps
        dot_prod = q * q_target';

        sign_flip = sign(dot_prod);
        if sign_flip == 0, sign_flip = 1; end

        q_t0 = q_target(1) * sign_flip;
        q_t1 = q_target(2) * sign_flip;
        q_t2 = q_target(3) * sign_flip;
        q_t3 = q_target(4) * sign_flip;

        e0  =  q(1)*q_t0 + q(2)*q_t1 + q(3)*q_t2 + q(4)*q_t3;
        e_x = -q(1)*q_t1 + q(2)*q_t0 - q(3)*q_t3 + q(4)*q_t2;
        e_y = -q(1)*q_t2 + q(3)*q_t0 - q(4)*q_t1 + q(2)*q_t3;
        e_z = -q(1)*q_t3 + q(4)*q_t0 - q(2)*q_t2 + q(3)*q_t1;
        e_vector = [e_x, e_y, e_z];

        dot_spatial = abs(e0);
        if dot_spatial > 1.0, dot_spatial = 1.0; end
        align_byte = uint8(dot_spatial * 255);

        Kp = -1.15;
        Kd = -1.65;
        Ki = -0.01;

        r_mag_sq = x^2 + y^2 + z^2;
        omega_orbit_eci = cross([x, y, z], [vx, vy, vz]) / r_mag_sq;

        q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
        DCM_curr = [
            1.0 - 2.0*(q2^2 + q3^2),   2.0*(q1*q2 - q0*q3),       2.0*(q1*q3 + q0*q2);
            2.0*(q1*q2 + q0*q3),       1.0 - 2.0*(q1^2 + q3^2),   2.0*(q2*q3 - q0*q1);
            2.0*(q1*q3 - q0*q2),       2.0*(q2*q3 + q0*q1),       1.0 - 2.0*(q1^2 + q2^2)
        ];
        omega_orbit_body = (DCM_curr * omega_orbit_eci')';

        if abs(e_x) < 0.1 && abs(e_y) < 0.1 && abs(e_z) < 0.1
            q_error_integral_3d = q_error_integral_3d + e_vector * dt_sub;
        else
            q_error_integral_3d = [0.0, 0.0, 0.0];
        end
        max_integral = 0.05;
        q_error_integral_3d = max(-max_integral, min(max_integral, q_error_integral_3d));

        omega_rel = omega - omega_orbit_body;

        M_ctrl(1) = Kp * e_x + Kd * omega_rel(1) + Ki * q_error_integral_3d(1);
        M_ctrl(2) = Kp * e_y + Kd * omega_rel(2) + Ki * q_error_integral_3d(2);
        M_ctrl(3) = Kp * e_z + Kd * omega_rel(3) + Ki * q_error_integral_3d(3);

        [x, y, z, vx, vy, vz, orbit_radius, q, omega] = propagate(x, y, z, vx, vy, vz, q, omega, dt_sub, engine_state, M_ctrl);
    end

    bytes_altitude = typecast(single(altitude_wgs84), 'uint8');
    bytes_time     = typecast(uint32(sim_time), 'uint8');

    is_hil_active_this_step = false;
    packet_received = false;

    if (USE_PHYSICAL_HARDWARE == 1) && (isempty(device) || ~isvalid(device))
        try
            device = serialport(com_port, baud_rate, 'Timeout', 0.1);
            configureTerminator(device, "LF");
            setRTS(device, false);
            setDTR(device, false);
            flush(device);
            is_hardware_available = true;
            fprintf('\n MCC NOTIFICATION: USB Cable reconnected! HIL Interface Restored.\n\n');
        catch
            device = [];
            is_hardware_available = false;
        end
    end

    if (USE_PHYSICAL_HARDWARE == 1) && is_hardware_available && ~isempty(device)
        try
            tx_packet = [bytes_altitude(:)', bytes_time(:)', uint8(align_byte), uint8(cmd_maneuver)];
            tx_packet = tx_packet(1:10);
            tx_packet(11) = crc8_cal(tx_packet);

            write(device, tx_packet, "uint8");

            t_wait_start = tic;
            while toc(t_wait_start) < 0.5
                if device.NumBytesAvailable > 0
                    new_byte = read(device, 1, "uint8");
                    ring_buffer = [ring_buffer(2:end), new_byte];
                    calc_crc = crc8_cal(ring_buffer(1:6));
                    recv_crc = ring_buffer(7);
                    if (calc_crc == recv_crc) && (recv_crc ~= 0)
                        mcu_time = typecast(ring_buffer(1:4), 'uint32');
                        if mcu_time == sim_time
                            engine_state = ring_buffer(5);
                            adcs_state   = ring_buffer(6);
                            packet_received = true; 
                            is_hil_active_this_step = true;
                            break;
                        end
                    end
                end
                pause(0.001);
            end
        catch
            if ~isempty(device)
                try delete(device); catch; end
            end
            device = []; 
            is_hardware_available = false;
            is_hil_active_this_step = false;
            fprintf('\n MCC WARNING: USB Cable disconnected! Activating emergency SIL backup.\n\n');
        end
    end

    if is_hil_active_this_step
        flight_mode_status = '[ HARDWARE-IN-THE-LOOP (HIL) ]';
        flush(device);
        ring_buffer = uint8(zeros(1, rxPacketSize));
    else
        if (cmd_maneuver == 1) && (align_byte > 250)
            engine_state = 1;
        else
            engine_state = 0;
        end
        adcs_state = 1;
        flight_mode_status = '[ SOFTWARE-IN-THE-LOOP (SIL) ]';
        if (USE_PHYSICAL_HARDWARE == 1)
            flight_mode_status = '[ HARDWARE TIMEOUT - FORCED SIL BACKUP ]';
        end
        pause(0.01);
    end

        clc;
        fprintf('=========================================\n');
        fprintf('                  HIL                    \n');
        fprintf('=========================================\n');
        fprintf('Sync step (PC and MCU)  : %d s\n', sim_time);
        fprintf('Coordinates (X, Y, Z)   : %.1f, %.1f, %.1f m\n', x, y, z);
        fprintf('Velocity (Vx, Vy, Vz)   : %.2f, %.2f, %.2f m/s\n', vx, vy, vz);
        fprintf('Orbit Radius            : %.1f m\n', orbit_radius);
        fprintf('Altitude                : %.1f m (WGS84)\n', altitude_wgs84);
        fprintf(' Inclination            : %.2f deg (Target: 90.0)\n', current_inclination_deg);
        fprintf(' Periapsis (Pe)         : %.1f km\n', alt_perigee / 1000.0);
        fprintf(' Apoapsis  (Ap)         : %.1f km\n', alt_apogee / 1000.0);
        fprintf('Target quaternion (LVLH): [%.4f, %.4f, %.4f, %.4f]\n', q_target0, q_target1, q_target2, q_target3);
        fprintf('Quaternion              : [%.4f, %.4f, %.4f, %.4f]\n', q(1), q(2), q(3), q(4));
        fprintf('Angular Velocity        : [%.3f, %.3f, %.3f] rad/s\n', omega(1), omega(2), omega(3));
        fprintf('Alignment Factor        : %d / 255\n', align_byte);
        fprintf('-----------------------------------------\n');
        if adcs_state == 1
            fprintf('ADCS Status             : [ A C T I V E ]\n');
        else
            fprintf('ADCS Status             : [ S T A B I L I Z A T I O N ]\n');
        end
        if engine_state == 1
            fprintf('Engine Status           : [ O N ]\n');
        else
            fprintf('Engine Status           : [ O F F ]\n');
        end
        fprintf('=========================================\n');

        if is_mission_complete
            fprintf('MISSION STATUS          : [ M I S S I O N  C O M P L E T E ]\n');
        else
            if is_circularizing
                fprintf('GNC ACTIVE PHASE        : [ PHASE 3: ORBIT CIRCULARIZATION (RETROGRADE BURN) ]\n');
            elseif is_saving_orbit
                fprintf('GNC ACTIVE PHASE        : [ PHASE 1: ALTITUDE RESTORATION (PROGRADE BURN) ]\n');
            else
                fprintf('GNC ACTIVE PHASE        : [ PHASE 2: POLAR INCLINATION CHANGE (CROSS-TRACK BURN) ]\n');
            end
        end

        addpoints(hTrajectory, x, y, z);
        set(hSatellite, 'XData', x, 'YData', y, 'ZData', z);
        set(tSatellite, 'Position', [x, y, z]);
        drawnow;

    time_spent = toc(t_step_start);

    time_to_sleep = 1.0 - time_spent;

    if time_to_sleep > 0
        pause(time_to_sleep);
    else
        pause(0.001);
    end
end
