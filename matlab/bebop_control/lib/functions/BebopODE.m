% Johann Diep (jdiep@student.ethz.ch), Tinamu Labs - September 2019
%
% This function realizes the ODE for the Bebop drone.

function [x_k] = BebopODE(states,inputs)
    % model states
    vel_x = states(1);
    vel_y = states(2);
    roll_del = states(3);
    pitch_del = states(4);
    omega_roll = states(5);
    omega_pitch = states(6);

    % model inputs
    u_roll = inputs(1);
    u_pitch = inputs(2);
    u_vel_z = inputs(3);

    % model parameters
    C = 0.35; g = 9.81; omega_0 = 7.8; eta = 1.8;

    pos_x_k = vel_x;
    pos_y_k = vel_y;
    pos_z_k = u_vel_z;

    vel_x_k = tan(pitch_del)*g-C*vel_x;
    vel_y_k = -tan(roll_del)*g-C*vel_y;

    roll_del_k = omega_roll;
    pitch_del_k = omega_pitch;

    omega_roll_k = omega_0^2*(u_roll-roll_del)-eta*omega_0*omega_roll;
    omega_pitch_k = omega_0^2*(u_pitch-pitch_del)-eta*omega_0*omega_pitch;

    pos_k = [pos_x_k;pos_y_k;pos_z_k];
    vel_k = [vel_x_k;vel_y_k];

    rp_del_k = [roll_del_k;pitch_del_k];
    omega_k = [omega_roll_k;omega_pitch_k];

    x_k = [pos_k;vel_k;rp_del_k;omega_k];
end