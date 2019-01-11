s = tf('s');
G = 1/(s^2 * (s^2 + s + 4));

sys = ss(G);

C_r = 20 * (0.1 + 2 * s)/(1 + s/100) * (s^2 + s + 4)/(1 + s * 0.001)^2;
C_d = 20 * (10 + 2 * s + 10/s)/(1 + s/100) * (s^2 + s + 4)/(1 + s * 0.001)^2;

% controller and system must be discretized separately 
w_cg = 300; % cross over frequency
sampling_freq = 2*w_cg;
sys_discrete = c2d(sys, 2*pi/sampling_freq, 'zoh');
G_d = c2d(G, 2*pi/sampling_freq, 'zoh');
C_rd = c2d(C_r, 2*pi/sampling_freq, 'tustin');  
C_dd = c2d(C_d, 2*pi/sampling_freq, 'tustin');

ref_track_sys =  minreal(C_r * G/(1 + C_r * G));
dist_track_sys =  minreal( G/(1 + C_d * G));

[balanced_sys_ref, v] = balreal(ref_track_sys);
[balanced_sys_dist, v] = balreal(dist_track_sys);
ref_track_sys_d = c2d(balanced_sys, 2*pi/sampling_freq, 'tustin');
dist_track_sys_d = c2d(balanced_sys_dist, 2*pi/sampling_freq, 'tustin');

fig = figure;
step(ref_track_sys); hold on;
step(ref_track_sys_d);
legend('Continous', 'Discrete');
title('Response to a step in the reference signal');
saveas(fig, 'images/step_response_comp.jpg')
fig = figure;
step(dist_track_sys); hold on;
step(dist_track_sys_d, 'r');
legend('Continous', 'Discrete');
title('Response to a step disturbance');
saveas(fig, 'images/step_dist_response_comp.jpg');