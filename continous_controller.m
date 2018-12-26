s = tf('s');
G = 1/(s^2 * (s^2 + s + 4));
a = 50;
T = 0.001; 

wd = 1/T; wn = 1/(T * a); wm = 1/(T * a^0.5);
K = 20;
k_p = 0.1;
C = K * (k_p + 2 * s)/(1 + s/100) * (s^2 + s + 4)/(1 + s * T)^2;
% C = K * (k_p + 2 * s)/(1 + s/300) * (1 + s * T * a)^2/(1 + s * T)^2;
H_yr = C * G /(1 + C * G);
H_yq = G / (1 + C * G);
% time = linspace(0, 10, 1000);
figure;
hold on;
bode(C * G);
figure;
[y, t] = step(H_yr, time);
step(H_yr)
S = stepinfo(y, t,'SettlingTimeThreshold',0.01);
figure;
step(H_yq);