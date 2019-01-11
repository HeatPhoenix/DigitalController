clear all;
s = tf('s');
G = 1/(s^2 * (s^2 + s + 4));
sys = ss(G);
C = eye(4);
selector = [0, 0, 0, 1];
feedback_sys = ss(G);

continous_poles = [-90, -115, -120, -130]
K = place(sys.A, sys.B, continous_poles);
feedback_sys.A = sys.A - sys.B * K;
feedback_sys.B = feedback_sys.B/dcgain(feedback_sys);

% step(feedback_sys)

w_cg = min(abs(continous_poles)*4*pi); % cross over frequency
% the sampling period seems to affect the system!
sampling_freq = 2*w_cg;
ts = 2*pi/(sampling_freq);

discrete_poles = exp(continous_poles * ts);
sys_discrete = (c2d(sys, 2*pi/sampling_freq, 'zoh'));
K_d = place(sys_discrete.A, sys_discrete.B, discrete_poles);
K_delay = place(sys_discrete.A, sys_discrete.B, discrete_poles);

sim('state_space_delay_2.slx');
figure;
plot(delay_data); hold on;
plot(data);
legend('Delayed', 'Original');
saveas(gcf, 'images/pole_delay_old.png');

continous_poles = [-90, -115, -120, -130] * 0.7;
discrete_poles = exp(continous_poles * ts);
K_delay = place(sys_discrete.A, sys_discrete.B, discrete_poles);

sim('state_space_delay_2.slx');
figure;
plot(delay_data); hold on;
plot(data);
legend('Delayed', 'Original');
title('Step response with redesigned controller');
saveas(gcf, 'images/pole_delay_new.png');

