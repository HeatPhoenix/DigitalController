s = tf('s');
G = 1/(s^2 * (s^2 + s + 4));

sys = ss(G);
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

% identity 
% Q = eye(4);
% R = 1;

% depenalize altitude
% Q(1, 1) = 1000;
% Q(2, 2) = 100;
% Q(3, 3) = 10;
% Q(4, 4) = 1;

[K, S, E] = dlqr(sys_discrete.A, sys_discrete.B, Q, R);
sys_discrete.A = sys_discrete.A - sys_discrete.B * K;
sys_discrete.B = sys_discrete.B/dcgain(sys_discrete);
step(sys_discrete)

title('Step Response of system with LQR controller');
saveas(gcf, 'images/step_response_q1');
