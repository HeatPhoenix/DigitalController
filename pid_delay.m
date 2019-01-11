clear all;
s = tf('s');
G = 1/(s^2 * (s^2 + s + 4));
sys = ss(G);

%% PID like controller
C_d = 20 * (0.1 + 2 * s + 0.01/s)/(1 + s/100) * (s^2 + s + 4)/(1 + s * 0.001)^2;
w_cg = 300; % cross over frequency
sampling_freq = 2 * w_cg;
Ts = 2 * pi/sampling_freq;
% C_d = 3 * (1 + 1 * s + 1/s)/(1 + s) * (s^2 + 0.1*s + 0.01)/(1 + s)^2;
delay_cont = exp(-s * Ts);
tf_delay = feedback(C_d * delay_cont * G, 1);
tf = feedback(C_d * G, 1);


%% Discrete version
tf_d = c2d(tf, 2*pi/sampling_freq, 'tustin');
tf_delay_d = c2d(tf_delay, 2*pi/sampling_freq, 'tustin');
figure;
step(tf_delay_d); hold on;
step(tf_d);
legend('Delayed', 'Original');
saveas(gcf, 'images/pid_original_delay.png');

%% Redisign of controller 
C_d = 10 * (0.1 + 2 * s + 0.01/s)/(1 + s/100) * (s^2 + s + 4)/(1 + s * 0.001)^2;
tf_delay = feedback(C_d * delay_cont * G, 1);

tf_delay_d = c2d(tf_delay, 2*pi/sampling_freq, 'tustin');
figure;
step(tf_delay_d); hold on;
step(tf_d);
legend('Delayed', 'Original');
title('Step response with redisigned controller');
saveas(gcf, 'images/pid_updated_delay.png');