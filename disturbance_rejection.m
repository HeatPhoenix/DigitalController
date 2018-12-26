s = tf('s');
G = 1/(s^2 * (s^2 + s + 4));
a = 50;
T = 0.001; 

wd = 1/T; wn = 1/(T * a); wm = 1/(T * a^0.5);
C = 20 * (0.1 + 2 * s)/(1 + s/100) * (s^2 + s + 4)/(1 + s * T)^2;
C = 20 * (10 + 2 * s + 10/s)/(1 + s/100) * (s^2 + s + 4)/(1 + s * T)^2;

H_yr = C * G /(1 + C * G);
H_yq = G / (1 + C * G);
evalfr(H_yq, 0)
figure;
step(H_yr)
figure;
step(H_yq);


