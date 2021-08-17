% Roots of a polynomial
root = roots([1 3 2 4])
% Polynom coefficent from the roots
pol = poly(r);

% Open-loop transfer function
G1 = tf([1], [1 4]);
%----- or -----------
% s = tf('s');
G2 = 1/(s+4);
%----- or -----------
G3 = zpk([],[-4],1);

% Graphing step response (4x)
[y1,t1] = step(4*G1);
[y2,t2] = step(4*G1);
[y3,t3] = step(4*G1);
plot(t1, y1, 'r-', t2, y2,'g-.', t3, y3, 'b--', 'Linewidth', 2);

% Closed-loop transfer function
G = tf([1], [1 4]);
K = zpk([],roots([1 1 2]),1);
H = feedback(G, K);
step(H);

% Finding pid gains
clc; clear all; close all;
syms s Ki Kd Kp
eq1 = 0.435+ 3.274*s + 0.944*s^2;
eq2 = 0.232*Ki+ 0.7251*Kd*s + 0.11*Kp*s^2;
c1 = coeffs(eq1,s);
c2 = coeffs(eq2,s);
soln = solve(c1(1)==c2(1), c1(2)==c2(2), c1(3)==c2(3));


%% Lab - PID Joint Control
clc; clear all; close all;
syms s Kp Ki Kd
theta_d = [50 200];
D = 50; % Disturbance: 50/s

% os = .01;
Ts = 5;
zeta = 1; % suggested critically damped
wn = (4/(zeta*Ts));

Kp = wn^2*50;
Kd = 2*zeta*wn*50;
Ki = 0;

nom1 =  Ki - D/theta_d(1) + Kp*s;
nom2 =  Ki - D/theta_d(2) + Kp*s;
denom = Ki + Kp*s + (10+Kd)*s^2 + 50*s^3;
T1 = syms2tf(nom1/denom);
T2 = syms2tf(nom2/denom);

figure;
subplot(1,2,1)
step(theta_d(1)*T1);
subplot(1,2,2)
step(theta_d(2)*T2);



%% With desired criterias calculated
clc; clear all; close all;
syms z s zeta Kp Ki Kd
theta_d = [50 200];
D = 50; % Disturbance: 50/s
OS = .01;
Ts = 5;

Z = vpa(solve(exp(-z*pi/sqrt(1-z^2))== OS, z),2);
zeta = (Z(1));
wn = (4/(zeta*Ts));

% Desired polynom, with 1 pole-placement
des = poly2sym([1,2*zeta*wn,wn^2],s)*(s+zeta*wn);
Pds = coeffs(des,s);

% Denominator of close loop Tf
denom = Ki + Kp*s + (10+Kd)*s^2 + 50*s^3;
Pcs = coeffs(denom,s);

soln = solve(Pcs(1)==Pds(1), Pcs(2)==Pds(2), Pcs(3)==Pds(3));
Kp = double(soln.Kp);
Ki = double(soln.Ki);
Kd = double(soln.Kd);

%============= Applying calculated gains =====================
% C = pid(double(soln.Kp), double(soln.Ki), double(soln.Kd));
% s = tf('s');
% U = C-50/s; %with disturbance
% G = zpk([],[10,5,0],1);
% T = feedback(U*G,1)

nom1 =  Ki - D/theta_d(1) + Kp*s;
nom2 =  Ki - D/theta_d(2) + Kp*s;
denom = Ki + Kp*s + (10+Kd)*s^2 + 50*s^3;
T1 = syms2tf(nom1/denom);
T2 = syms2tf(nom2/denom);

figure;
subplot(1,2,1)
step(theta_d(1)*T1);
subplot(1,2,2)
step(theta_d(2)*T2);

