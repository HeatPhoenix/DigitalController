s = tf('s');
G = 1/(s^2 * (s^2 + s + 4));

sys = ss(G);
% controller and system must be discretized separately 
continous_poles = [-90, -115, -120, -130];
w_cg = min(abs(continous_poles)*2*pi); % cross over frequency
% the sampling period seems to affect the system!
sampling_freq = 2*w_cg;
ts = 2*pi/(sampling_freq);

discrete_poles = exp(continous_poles * ts);
% poles = [0.2, 0.3, 0.25, 0.4];
% poles = [0.9, 0.91, 0.92, 0.93];
sys_discrete = c2d(sys, 2*pi/sampling_freq, 'zoh');
original_poles_d = exp(pole(G) * 2*pi/sampling_freq);

K = place(sys_discrete.A, sys_discrete.B, discrete_poles);
sys_discrete.A = sys_discrete.A - sys_discrete.B * K;
dc_gain = dcgain(sys_discrete);
% fig = figure;
opt = stepDataOptions('StepAmplitude',1/dcgain(sys_discrete));
step(sys_discrete, opt)
% stepinfo(sys_discrete)
% title('Feedback system reponse with pole placement');
% saveas(fig, 'images/pole_placement_step.png');

%% observer
observer = c2d(sys, 2*pi/sampling_freq, 'zoh');
K_obs = 1;
% sim('observer_model');
% plot(simout);