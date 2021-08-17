% Show DH Table
% dh = table([a1;a2;a3],[alpha1;alpha2;alpha3],[d1;d2;d3],{q1;q2;q3},...
%     'VariableNames',{'a' 'Alpha' 'd' 'Theta'},...
%     'RowNames',{'Link1' 'Link2' 'Link3'})

% Transformation Matrices
% A(theta, alpha, a, d) =...
%     [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta); ...
%      sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta); ...
%      0, sin(alpha), cos(alpha), d; ...
%      0, 0, 0, 1];

% NOTE: Instead of sym(pi), possible to use sind(pi) and cosd(pi) etc


%% Lab 1 - 3-Links Sawyer robot arm
clear all;clc;
syms a1 d1 d2 q1 q2 alpha1 alpha2

a1 = 81;
a2 = 0;
alpha1 = -sym(pi)/2;
alpha2 = -sym(pi)/2;
d1 = 317;
d2 = 193;
q1 = 0;
q2 = 3*sym(pi)/2;

dh = table([a1;a2],[double(alpha1);double(alpha2)],[d1;d2],[double(q1);double(q2)],...
    'VariableNames',{'a' 'Alpha' 'd' 'Theta'},...
    'RowNames',{'Link1' 'Link2'})

% Rotation and transformation matrices
R01 = [cos(q1), -sin(q1)*cos(alpha1), sin(q1)*sin(alpha1); ...
     sin(q1), cos(q1)*cos(alpha1), -cos(q1)*sin(alpha1); ...
     0, sin(alpha1), cos(alpha1)];

R12 = [cos(q2), -sin(q2)*cos(alpha2), sin(q2)*sin(alpha2); ...
     sin(q2), cos(q2)*cos(alpha2), -cos(q2)*sin(alpha2); ...
     0, sin(alpha2), cos(alpha2)];

R02 = R01 * R12;

T01 = [cos(q1), -sin(q1)*cos(alpha1), sin(q1)*sin(alpha1), a1*cos(q1); ...
       sin(q1), cos(q1)*cos(alpha1), -cos(q1)*sin(alpha1), a1*sin(q1); ...
       0,       sin(alpha1),          cos(alpha1),         d1;...
       0,       0,                    0,                   1];

T12 = [cos(q2), -sin(q2)*cos(alpha2), sin(q2)*sin(alpha2), a1*cos(q2); ...
       sin(q2), cos(q2)*cos(alpha2), -cos(q2)*sin(alpha2), a1*sin(q2); ...
       0,       sin(alpha2),          cos(alpha2),         d2;...
       0,       0,                    1,                   0];
   

T02 = T01 * T12

% T20 = pinv(T02)
T20 = vpa(pinv(T02),4)

%% Lab 2 - Joint velocities
% qd1 and qd2 are the time derivatives of q1 and q2 respectively
clear all;clc;
syms a1 d1 d2 q1 q2 c1 c2 c3 qd1 qd2 real

% Vc1 = Jvc1*qdot
% Vc2 = Jvc2*qdot
qd = [qd1 ;qd2];
Jvc1 = [-c1*sin(q1) 0;
         c1*cos(q1) 0;
         0          0];
     
    
Jvc2 = [-c1*sin(q1)-rms([c2,c3])*sin(q1+q2) -rms([c2,c3])*sin(q1+q2);
         c1*cos(q1)+rms([c2,c3])*cos(q1+q2)  rms([c2,c3])*cos(q1+q2);
         0                         0];
     
% Point velocities  
vp1 = Jvc1*qd
vp2 = Jvc2*qd