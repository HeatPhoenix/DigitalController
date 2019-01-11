clear all;
%% Redising of the PID controller with limited input commands
% first task is get the control inputs given to the plant
s = tf('s');
G = 1/(s^2 * (s^2 + s + 4));

sys = ss(G);
C_d_original = 20 * (10 + 2 * s + 10/s)/(1 + s/100) * (s^2 + s + 4)/(1 + s * 0.001)^2;
C_d = 3 * (1 + 1 * s + 1/s)/(1 + s) * (s^2 + 0.1*s + 0.01)/(1 + s)^2;
% bode(C_d * G)
% controller and system must be discretized separately 
w_cg = 300; % cross over frequency
sampling_freq = 2*w_cg;
sampling_freq_new = 2*pi;

% % discretizing like this leads to numerical errors
% sys_discrete = c2d(sys, 2*pi/sampling_freq, 'zoh');
% G_d = c2d(G, 2*pi/sampling_freq, 'zoh');
% C_dd = c2d(C_d, 2*pi/sampling_freq, 'tustin');

% find control input of the original PID controller
dist_track_sys_original_ur =  minreal(C_d_original/(1 + C_d_original * G));
dist_track_sys_original_ud =  minreal(-G * C_d_original/(1 + C_d_original * G));

[balanced_sys_dist_original_ur, v] = balreal(dist_track_sys_original_ur);
dist_track_sys_d_original_ur = c2d(balanced_sys_dist_original_ur, 2*pi/sampling_freq, 'tustin');

[balanced_sys_dist_original_ud, v] = balreal(dist_track_sys_original_ur);
dist_track_sys_d_original_ud = c2d(balanced_sys_dist_original_ud, 2*pi/sampling_freq, 'tustin');

dist_track_sys_ur =  minreal(C_d/(1 + C_d * G));
dist_track_sys_ud =  minreal(- G * C_d/(1 + C_d * G));
% find contorl input of redisigned controller
[balanced_sys_dist_ur, v] = balreal(dist_track_sys_ur);
dist_track_sys_d_ur = c2d(balanced_sys_dist_ur, 2*pi/sampling_freq_new, 'tustin');

[balanced_sys_dist_ud, v] = balreal(dist_track_sys_ud);
dist_track_sys_d_ud = c2d(balanced_sys_dist_ud, 2*pi/sampling_freq_new, 'tustin');

% transfer function from reference to error or output ot error
figure;
[y, t, x] = step(dist_track_sys_d_original_ur);
subplot(2, 1, 1); stairs(t, y);
xlabel('Time [sec]'); ylabel('Amplitude');
title("Control input of PID controller to a step reference");
[y, t, x] = step(dist_track_sys_d_original_ud);
subplot(2, 1, 2); stairs(t, y);
xlabel('Time [sec]'); ylabel('Amplitude');
title("Control input of PID controller to a step disturbance");
saveas(gcf, 'images/control_input_pid_or.png');

figure;
[y, t, x] = step(dist_track_sys_d_ur);
subplot(2, 1, 1); stairs(t, y);
xlabel('Time [sec]'); ylabel('Amplitude');
title("Control input of redesigned PID controller to a step reference");
[y, t, x] = step(dist_track_sys_d_ud);
subplot(2, 1, 2); stairs(t, y);
xlabel('Time [sec]'); ylabel('Amplitude');
title("Control input of redesigned PID controller to a step disturbance");
saveas(gcf, 'images/control_input_pid_red.png');

%% Redesigning pole placement controller
% original controller

% controller and system must be discretized separately 
continous_poles = [-90, -115, -120, -130];
w_cg_or = min(abs(continous_poles)*5*pi); % cross over frequency
% the sampling period seems to affect the system!
sampling_freq_or = 2*w_cg_or;
ts = 2*pi/(sampling_freq_or);

discrete_poles_or = exp(continous_poles * ts);
sys_discrete = (c2d(sys, 2*pi/sampling_freq_or, 'zoh'));
feedback_sys_discrete = (c2d(sys, 2*pi/sampling_freq_or, 'zoh'));
original_poles_d = exp(pole(G) * 2*pi/sampling_freq_or);

K = place(sys_discrete.A, sys_discrete.B, discrete_poles_or);
feedback_sys_discrete.A = sys_discrete.A - sys_discrete.B * K;
dc_gain = dcgain(feedback_sys_discrete);
feedback_sys_discrete.B = feedback_sys_discrete.B/dc_gain;
% fig = figure;
% opt = stepDataOptions('StepAmplitude', 1/dc_gain);
figure;
[y, t, x] = step(feedback_sys_discrete);
stairs(t, -K * x');
xlabel('Time [sec]'); ylabel('Amplitude');
title("Control input of the pole placement controller for a step reference");
saveas(gcf, 'images/control_input_pole_place_or.png');

% redisigned controller
continous_poles = [-1.35, -1.45, -1.50, -1.55];
w_cg = min(abs(continous_poles)*2*pi); % cross over frequency
% the sampling period seems to affect the system!
sampling_freq = 2*w_cg;
ts = 2*pi/(sampling_freq);

discrete_poles = exp(continous_poles * ts);
sys_discrete = (c2d(sys, 2*pi/sampling_freq, 'zoh'));
feedback_sys_discrete = (c2d(sys, 2*pi/sampling_freq, 'zoh'));
original_poles_d = exp(pole(G) * 2*pi/sampling_freq);

K = place(sys_discrete.A, sys_discrete.B, discrete_poles);
feedback_sys_discrete.A = sys_discrete.A - sys_discrete.B * K;
dc_gain = dcgain(feedback_sys_discrete);
feedback_sys_discrete.B = feedback_sys_discrete.B/dc_gain;
figure;

[y, t, x] = step(feedback_sys_discrete);
subplot(2, 1, 1); stairs(t, -K * x');
xlabel('Time [sec]'); ylabel('Amplitude');
title("Control input of the pole placement controller for a step reference");
subplot(2, 1, 2); stairs(t, y);
xlabel('Time [sec]'); ylabel('Amplitude');
title("System response of the pole placement controller for a step reference");
saveas(gcf, 'images/control_input_pole_place_red.png');

%% Redisign of the LQR controller
sampling_freq = 16 * pi;
sys_discrete = c2d(sys, 2*pi/sampling_freq, 'zoh');
sys_discrete_lqr_or = c2d(sys, 2*pi/sampling_freq, 'zoh');
sys_discrete_lqr_red = c2d(sys, 2*pi/sampling_freq*3, 'zoh');

Q = zeros(4, 4);
Q(1, 1) = 1;
Q(2, 2) = 1;
Q(3, 3) = 100;
Q(4, 4) = 1000;
R = 0.1;

[K, S, E] = dlqr(sys_discrete.A, sys_discrete.B, Q, R);
sys_discrete_lqr_or.A = sys_discrete.A - sys_discrete.B * K;
sys_discrete_lqr_or.B = sys_discrete.B/dcgain(sys_discrete_lqr_or);

figure;
[y, t, x] = step(sys_discrete_lqr_or);
stairs(t, -K * x');
xlabel('Time [sec]'); ylabel('Amplitude');
title("Control input of LQR controller for a step reference");
saveas(gcf, 'images/control_input_lqr_or.png');

% redisigned matrices 
Q = zeros(4, 4);
Q(1, 1) = 1;
Q(2, 2) = 1;
Q(3, 3) = 1;
Q(4, 4) = 100;
R = 16;

[K_red, S, E] = dlqr(sys_discrete_lqr_red.A, sys_discrete_lqr_red.B, Q, R);
sys_discrete_lqr_red.A = sys_discrete_lqr_red.A - sys_discrete_lqr_red.B * K_red;
sys_discrete_lqr_red.B = sys_discrete_lqr_red.B/dcgain(sys_discrete_lqr_red);

figure;
[y, t, x] = step(sys_discrete_lqr_red);
subplot(2, 1, 1); stairs(t, -K_red * x');
xlabel('Time [sec]'); ylabel('Amplitude');
title("Control input of the LQR controller for a step reference");
subplot(2, 1, 2); stairs(t, y);
xlabel('Time [sec]'); ylabel('Amplitude');
title("System response of the LQR controller for a step reference");
saveas(gcf, 'images/control_input_lqr_red.png');
