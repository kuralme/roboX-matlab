%% Lab 1 - Stability
clc; clear all; close all;
syms t real

A = [0 1 0;0 0 1;-6 -11 -6];
ini = [1 2 -1]'; %initial values

x_t = expm(A*t)*ini


%% Lab 2 - Stability Analysis of a Nonlinear System
% Lyapunov first method
clc; clear all; close all;
syms p a B lambda real % p = rho, a = alpha, b = beta 
syms k_p k_a k_b real

eq = [0 0 0]; % equilibrium point
p = p-eq(1);
a = a-eq(2);
B = B-eq(3);

% nonlinear system
xdot = [-cos(a) 0; sin(a)/p -1;-sin(a)/p 0]*[k_p 0 0;0 k_a k_b]*[p; a; B];

for i = 1:3
    [Jp(i,1)] = gradient(xdot(i),p);
    [Ja(i,1)] = gradient(xdot(i),a);
    [JB(i,1)] = gradient(xdot(i),B);
end

p=eq(1);
a=eq(2);
B=eq(3);
A = [eval(Jp) eval(Ja) eval(JB)] % linearized system

% Empirical gains [k_p k_a k_b];
k_p=1;
k_a=2;
k_b=-.25;

% or eig(A)
pol = det(lambda*eye(3,3)-A);
eigs = solve(eval(pol)==0);

if(double(eigs)<0)
   fprintf('System is stable with gains: [%.2f %.2f %.2f]\n', k_p, k_a, k_b);
   fprintf('Eigen values: [%.1f %.1f %.1f]\n', eigs(1), eigs(2), eigs(3));
end

%% Lab 3 - Stability Analysis of a Nonlinear System
% Lyapunov direct (second) method
clc; clear all; close all;
syms x1 x2 real

% x1dot = -x1^3 + x2;
% x2dot = -x1;

% Lyapunov function candidate
V = x1^2+x2^2;

% Lyapunov's direct method using V
Vdot = gradient(V,x1)*(-x1^3 + x2) + gradient(V,x2)*(-x1);
simplify(Vdot)



