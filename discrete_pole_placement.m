s = tf('s');
G = 1/(s^2 * (s^2 + s + 4));

sys = ss(G);
% controller and system must be discretized separately 
continous_poles = [-90, -115, -120, -130];
w_cg = min(abs(continous_poles)*5*pi); % cross over frequency
% the sampling period seems to affect the system!
sampling_freq = 2*w_cg;
ts = 2*pi/(sampling_freq);

discrete_poles = exp(continous_poles * ts);
% poles = [0.2, 0.3, 0.25, 0.4];
% poles = [0.9, 0.91, 0.92, 0.93];
sys_discrete = (c2d(sys, 2*pi/sampling_freq, 'zoh'));
feedback_sys_discrete = (c2d(sys, 2*pi/sampling_freq, 'zoh'));
original_poles_d = exp(pole(G) * 2*pi/sampling_freq);

K = place(sys_discrete.A, sys_discrete.B, discrete_poles);
feedback_sys_discrete.A = sys_discrete.A - sys_discrete.B * K;
dc_gain = dcgain(feedback_sys_discrete);
% fig = figure;
opt = stepDataOptions('StepAmplitude', 1/dc_gain);
step(feedback_sys_discrete, opt)

%% Observer tracking sheme
poles_observer = [-90, -115, -120, -130]*2;
poles_observer_d = exp(poles_observer * ts);

K_obs = place(sys_discrete.A', sys_discrete.C', poles_observer_d);
K_obs = K_obs';
% state space form equivalent to the simulink model
A_obs = [sys_discrete.A, -sys_discrete.B * K;
        K_obs * sys_discrete.C, sys_discrete.A - sys_discrete.B * K - K_obs * sys_discrete.C];
    

B_obs = [sys_discrete.B; sys_discrete.B];
C_obs = [sys_discrete.C, zeros(1, 4);
         zeros(1, 4), sys_discrete.C];

observer = ss(A_obs, B_obs, C_obs, 0, 'Ts', sys_discrete.Ts); 
k_r = dcgain(observer);
k_r = k_r(1)^-1;

% compare the performance of the observer
x_0 = [0, 0, 0, 2, 0, 0, 0, 0];
time = 0:observer.Ts:0.1;
u =  k_r * ones(size(time));
[y, t, x] = lsim(observer, u, time, x_0);

stairs(t, y);
legend('Model', 'Observer');
title('Observer and model output');


