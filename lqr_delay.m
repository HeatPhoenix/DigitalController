clear all;
s = tf('s');
G = 1/(s^2 * (s^2 + s + 4));
sys = ss(G);
C = eye(4);
selector = [0, 0, 0, 1];
feedback_sys = ss(G);

w_cg = 8 * pi;
sampling_freq = 2*w_cg;
ts = 2*pi/(sampling_freq);

sys_discrete = c2d(sys, 2*pi/sampling_freq, 'zoh');
Q = zeros(4, 4);

% good matrices
Q(1, 1) = 1;
Q(2, 2) = 1;
Q(3, 3) = 100;
Q(4, 4) = 1000;
R = 0.1;

k_r = 30;
k_r_delay = 30;
k_i = 30;
k_i_delay = 30;
[K_d, S, E] = dlqr(sys_discrete.A, sys_discrete.B, Q, R);
[K_delay, S, E] = dlqr(sys_discrete.A, sys_discrete.B, Q, R);

sim('state_space_delay_3.slx');
figure;
plot(delay_data); hold on;
plot(data);
legend('Delayed', 'Original');
title('LQR controllers step response');
saveas(gcf, 'images/lqr_delay_old.png');

Q = zeros(4, 4);

%% Redisign
% good matrices
Q(1, 1) = 1;
Q(2, 2) = 1;
Q(3, 3) = 10;
Q(4, 4) = 100;
R = 0.1;
k_r_delay = 9;
k_i_delay = 9;
[K_delay, S, E] = dlqr(sys_discrete.A, sys_discrete.B, Q, R);
sim('state_space_delay_3.slx');
figure;
plot(delay_data); hold on;
plot(data);
legend('Delayed', 'Original');
title('Redesigned delayed LQR controller step response');
saveas(gcf, 'images/lqr_delay_new.png');


