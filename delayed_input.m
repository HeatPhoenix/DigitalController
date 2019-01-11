% Investigate the effect of delay 

s = tf('s');
G = 1/(s^2 * (s^2 + s + 4));

sys = ss(G);

%% PID like controller
% C_d = 20 * (0.1 + 2 * s + 0.01/s)/(1 + s/100) * (s^2 + s + 4)/(1 + s * 0.001)^2;
C_d = 3 * (1 + 1 * s + 1/s)/(1 + s) * (s^2 + 0.1*s + 0.01)/(1 + s)^2;
delay_cont = exp(-s * 0.1);
tf_sys_delay = feedback(C_d * delay_cont * G, 1);

w_cg = 5; % cross over frequency
sampling_freq = 2 * w_cg;

dist_track_sys_r =  minreal(C_d * G/(1 + C_d * G));
fig = step_response(dist_track_sys_r, sampling_freq);
title('Response to step response');

dist_track_sys_d = minreal(G / (1 + C_d * G));
fig = step_response(dist_track_sys_d, sampling_freq);
title('Response to a step disturbance');


%% Pole placement controller

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
% run simulink integral action 
feedback_sys_discrete.A = sys_discrete.A - sys_discrete.B * K;
dc_gain = dcgain(feedback_sys_discrete);
feedback_sys_discrete.B = feedback_sys_discrete.B/dc_gain;

figure;
plot(integral_action_data);
figure;
plot(state_space_delayed);
% figure;



function fig =  step_response(dist_track_sys, sampling_freq)
[balanced_sys_dist, v] = balreal(dist_track_sys);
dist_track_sys_d = c2d(balanced_sys_dist, 2*pi/sampling_freq, 'tustin');
settling_time = 0.446; peak = 1.175;
[y_r, t, x] = step(dist_track_sys_d);
fig = figure;
step(dist_track_sys_d); hold on;

delayed_contr = ss(dist_track_sys_d.A, dist_track_sys_d.B,  ...
                dist_track_sys_d.C, dist_track_sys_d.D, dist_track_sys_d.Ts, 'InputDelay', 1);
step(delayed_contr);
legend('Original', 'Delayed');
end



