s = tf('s');
G = 1/(s^2 * (s^2 + s + 4));
omega_g = 2;
damp = 1/4;

% sisotool(G);
C_1 = 1.7 * s; % design 1
C_2 = 1.7 * s * (1 + 0.34 * s + 0.42 * s^2); % design 2: too much noise
C_3 = 1.7 * s/(1 + 1.2 * s);
C_4 = 1.54 * s/(1 + 0.9 * s); % design 3
C_5 = 10 * (s/(s/100 + 1) + 1) * (s/0.5 + 1)^2/(s/100 + 1)^2;
figure;
bode(C_4 *  G);

%% Design 3 
H = G * C_5/(1 + G * C_5);
figure;
bode(C_5 * G)
% figure;
% bode(C_4 * G)
figure;
step(H);
stepinfo(H)

