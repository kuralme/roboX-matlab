A = [ 0 2; -2 -5];
% Eigenvalues
eig(A);

% Eigenvectors V and eigenvalues D
[v, d] = eig(A);

% Matrix exponential
expm(A);
expm(sym(A));

syms t real
expm(A*t);

[v, d] = eig(A);
v*expm(d)*inv(v); % = expm(A)

% State space matrices
A = [ 0 2; -2 -5];
B = [0; 1];
C = [1 0];

sys = ss(A, B, C, 0) % D =0
step(sys);

% State space to transfer function
[numc,denc] = ss2tf(A, B, C, 0);

% Controlability, det != 0
ctrb(A, B);
% or ctrb(sys)
% or [B B*A]
diag
% Observability, det != 0
obsv(A, C);
% or obsv(sys)
% or [C; C*A]

% Stability, eig < 0
if(eig(A)<0 == 1)
   disp('stable');
end

% (LQR)Linear quadratic optimal controller
A = [ 0 2; -2 -5];
B = [0; 1];
C = [1 0];
% Chosen
R = 3;
Q = C'*C;

% One controller that we can use to achieve desired performance from
% a state space system in a linear quadratic optimal controller.
% To find such a controller, we need to solve the algebraic Riccati equation and
% use the solution to find a set of gains.
% These gains, when implemented as state feedback, achieve optimal performance with
% respect to an energy-like cost function J which was introduced during the lecture.
% Given a state space system, in choosing values for R and Q as shown above,
% we can solve the algebraic Ricatti equation using MATLAB's CARE function,
% The outputs of care() are:
% P: a solution,
% L: the closed loop eigenvalues of the system,
% G: the gain matrix that will be used for state feedback
[P L G] = care(A, B, Q, R);

% From our understanding of linear quadratic optimal control, we can compute the gain
% matrix, G, and the closed loop eigenvalues ourselves and verify the solution
G1 = inv(R)*B'*P;  % G1 = G
L1 = eig(A - B*G); % L1 = L

%% Lab 1 - Two Link Planar Manipulator state space
% Inverse dynamic eqns:
% d11*q1ddot+d12*q2ddot + c121*q1dot*q2dot+c211*q2dot*q1dot+c221*q2dot^2 + g1 = tau1
% d21*q1ddot+d22*q2ddot +                c112*q1dot^2                    + g2 = tau2

% x1 = q1        x1dot = x2
% x2 = q1dot     x2dot = (tau - d2*x1dot - [c121*x2*x4+c211*x4*x2+c221*x4^2; c112*x2^2] - g)/d1
% x3 = q2        x3dot = x4
% x4 = q2dot     x4dot = (tau - d1*x2dot - [c121*x2*x4+c211*x4*x2-c221*x4^2; c112*x2^2] - g)/d2
% 
% s.t. xdot = f(x,tau)

clc; clear all; close all;
syms d11 d12 d21 d22 c121 c211 c221 c112 g1 g2 tau1 tau2 x1 x2 x3 x4 real
x=[x1; x2; x3; x4];

% f_x_tau = A*x;

%% Lab 2 - Pole-placement example
% Linear State Feedback Control Law: u = -Kx + r
% xdot = (A-BK)x
%    y = (A-CK)x
% 
% u = -K.'*x+r = -k1x-k2x+r
% xdot = [ 1 -3; -1 -2]x + [1; 2]u
%    y = [1 1]x
clc; clear all; close all;

A = [ 1 -3; -1 -2];
B = [ 1; 2];

P = [-2 -3]; % desired CL poles
K = -place(A,B,P)

%% Manual
syms lambda k1 k2
A = [ 1 -3; -1 -2];
B = [ 1; 2];
K = [ k1 k2];

% Desired polynom: poles at -2,-3
Pds = coeffs( (lambda+2)*(lambda+3), lambda);

% Feedback polynom: det(lambda*I-A-BK)
Pcs = coeffs( det(lambda*eye(2,2)- A - B*K), lambda);

soln = solve(Pcs(1)==Pds(1), Pcs(2)==Pds(2));
K = [ double(soln.k1) double(soln.k2)]
