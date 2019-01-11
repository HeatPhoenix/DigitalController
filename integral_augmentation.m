%% In this file we use the integral action of the simulink file
% first task is get the control inputs given to the plant
s = tf('s');
G = 1/(s^2 * (s^2 + s + 4));

sys = ss(G);
sampling_freq = 6 * pi;
discrete_sys = c2d(sys, 2*pi/sampling_freq*3, 'zoh');
discrete_lqr = c2d(sys, 2*pi/sampling_freq*3, 'zoh');

% redisigned matrices 
Q = zeros(4, 4);
Q(1, 1) = 1;
Q(2, 2) = 1;
Q(3, 3) = 1;
Q(4, 4) = 100;
R = 16;

[K, S, E] = dlqr(discrete_sys.A, discrete_sys.B, Q, R);
discrete_lqr.A = discrete_lqr.A - discrete_lqr.B * K;
discrete_lqr.B = discrete_lqr.B/dcgain(discrete_lqr);
step(discrete_lqr);
figure;

% extend state space 
A_new = [discrete_sys.A, [0, 0; 0, 0; 0, 0; 0, 0];
        discrete_sys.C, 1, 0;
        0, 0, 0, 0, 0, 0];
B_new = [discrete_sys.B; 0; 1];
C_new = [discrete_sys.C, 0, 0];
K_new = [K, 0.1, -0.1];

aug_discrete_lqr = ss(A_new - B_new * K_new, B_new, C_new, 0, discrete_sys.Ts);
% aug_discrete_lqr.B = aug_discrete_lqr.B/dcgain(aug_discrete_lqr);

% step(aug_discrete_lqr);
sim('integral_action.slx');

plot(integral_action_data)
title('State space response to a step disturbance');
xlabel('Time [s]'); ylabel('Amplitude');
saveas(gcf, 'images/int_action_dist.png');