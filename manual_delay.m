% Investigate the effect of delay 
s = tf('s');
G = 1/(s^2 * (s^2 + s + 4));

sys = ss(G);

C_d = 20 * (0.1 + 2 * s + 0.01/s)/(1 + s/100) * (s^2 + s + 4)/(1 + s * 0.001)^2;

w_cg = 300; % cross over frequency
sampling_freq = 2 * w_cg;

dist_track_sys =  minreal(C_d * G/(1 + C_d * G));

[balanced_sys_dist, v] = balreal(dist_track_sys);
dist_track_sys_d = c2d(balanced_sys_dist, 2*pi/sampling_freq, 'tustin');
settling_time = 0.446; peak = 1.175;
[y_r, t, x] = step(dist_track_sys_d);
step(dist_track_sys_d); hold on;

% one step delay 
size_A = size(dist_track_sys_d.A);
A_new = [dist_track_sys_d.A, dist_track_sys_d.B;
        zeros(1, size_A(1)), 0];
B_new = [zeros(size_A(1), 1); 1];
C_new = [dist_track_sys_d.C, 0];
D_new = 0;
delayed_pid_contr = ss(A_new, B_new, C_new, D_new, dist_track_sys_d.Ts);
delayed_pid_contr.B = delayed_pid_contr.B/dcgain(delayed_pid_contr);
settling_time_delay = 0.461; peak_delay = 1.1917;
[y_r_delay, t, x] = step(delayed_pid_contr);
step(delayed_pid_contr); 
title('Response to a reference step');
legend('Original', 'Delayed');
xlabel('Time [s]'); ylabel('Amplitude');

% same analysis as before but on a disturbance input
dist_track_sys =  minreal(G/(1 + C_d * G));

[balanced_sys_dist, v] = balreal(dist_track_sys);
dist_track_sys_d = c2d(balanced_sys_dist, 2*pi/sampling_freq, 'tustin');
settling_time = 0.446; peak = 1.175;
[y_r, t, x] = step(dist_track_sys_d);
figure;
step(dist_track_sys_d); hold on;


% one step delay 
size_A = size(dist_track_sys_d.A);
A_new = [dist_track_sys_d.A, dist_track_sys_d.B;
        zeros(1, size_A(1)), 0];
B_new = [zeros(size_A(1), 1); 1];
C_new = [dist_track_sys_d.C, 0];
D_new = 0;
delayed_pid_contr = ss(A_new, B_new, C_new, D_new, dist_track_sys_d.Ts, 'tustin');
delayed_pid_contr.B = delayed_pid_contr.B/dcgain(delayed_pid_contr);
settling_time_delay = 0.461; peak_delay = 1.1917;
% [y_r_delay, t, x] = step(delayed_pid_contr);
step(delayed_pid_contr); 
title('Response to a reference disturbance');
legend('Original', 'Delayed');
xlabel('Time [s]'); ylabel('Amplitude');



