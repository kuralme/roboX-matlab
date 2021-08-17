clear all;clc;

% number of links to consider
n = 2;

% DH parameters
syms a1 d1 d2 q1 q2 real

% the center of mass of each link measured relative to the link fixed frame
% (e.g. c1 = [c1x c1y c1z]' is measured relative to x1y1z1)
c = cell(n,1);
c{1} = [sym('c1x'); sym('c1y'); sym('c1z')];
c{2} = [sym('c2x'); sym('c2y'); sym('c2z')];
assume(vertcat(c{:}), 'real');

% the inertia tensor of each link relative to the inertial frame
I = cell(n,1);
I{1} = inertia_tensor(1);
I{2} = inertia_tensor(2);
% mass of each link
syms m1 m2 real
% joint velocities, where qd1 stands for 'q dot 1', the
% first derivative of q1 with respect to time
syms qd1 qd2 real
% acceleration due to gravity (assume g has the correct sign); in other words, if
% gravity were to act in the 'x' direction, the gravity vector would be [g 0 0]
syms g real

% initial conditions for the configuration of Sawyer shown in Figure 1.
% HINT: double(subs(expr, vars, vals)) evaluates a symbolic expression 'expr' by
% substituting each element of 'vals' with its corresponding symbolic variable in 'vars'
q = [0 3*sym(pi)/2]; % [q00 q20] mm
d = [317 192.5]; % [d10 d20] mm
a = [81 0]; % in mm
alpha = [-sym(pi)/2 -sym(pi)/2];

%% Lab 1 - Linear and Angular Velocity Jacobians
% cell array of your homogeneous transformations; each Ti{i} is a 4x4 symbolic 
% transformation matrix in terms of the given DH parameters that transforms objects
% in frame i to the inertial frame 0
Ti = cell(n,1);

% The angular velocity Jacobian as an nx1 cell array where each element, Jw{i} is 
% a 3xn symbolic matrix
Jw = cell(n,1);

% The linear velocity Jacobian as an nx1 cell array where each element, Jv{i} is 
% a 3xn symbolic matrix
Jv = cell(n,1);

z = cell(n,1);
zskew = cell(n,1);
o = cell(n,1);

% homogeneous transformations
for i=1:n
  Ti{i} = [cos(q(i)), -sin(q(i))*cos(alpha(i)),  sin(q(i))*sin(alpha(i)), a(i)*cos(q(i)); ...
           sin(q(i)),  cos(q(i))*cos(alpha(i)), -cos(q(i))*sin(alpha(i)), a(i)*sin(q(i)); ...
           0,          sin(alpha(i)),            cos(alpha(i)),           d(i);...
           0,          0,                        0,                       1];
       
   if i == 1
     Tn =  Ti{i};
     z{i} = Tn(1:3,3);    % z0 = z{1}
     o{i} = [ 0; 0; 0];   % o0 = o{1}
   else
     Tn = Ti{i-1} * Ti{i};
     z{i} = Tn(1:3,3);
     o{i} = Tn(1:3,4);
   end
end

for i=1:n 
    zskew{i} =[ 0    -z{i}(3) z{i}(2);
               z{i}(3)   0    -z{i}(1);
              -z{i}(2) z{i}(1) 0 ];
          
    % linear velocity Jacobian
    Jv{i} = zskew{i} * (o{n}-o{i});
    
    % angular velocity Jacobian
    Jw{i} = z{i};
end


%% Lab 2 - Kinetic and Potential Energy

% the inertia matrix
% D = 

% kinetic energy
% KE = 

% potential energy
% PE = 



