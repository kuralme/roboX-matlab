% Project #2 - Trajectory Tracking with a Mobile Robot
clc; clear all; close all;
syms x y theta l v w real

% x_dot = g(x)*u
u = [v; w]
g = [cos(theta) 0; sin(theta) 0; 0 1];
h = [x+l*cos(theta);y+l*sin(theta)];

gradh = [gradient(h(1),x) gradient(h(1),y) gradient(h(1),theta);...
         gradient(h(2),x) gradient(h(2),y) gradient(h(2),theta)];
     
% Lie derivative of g
lgh = gradh*g

% Rate of change of output
% y_dot = lfh*x + lgh*u
y_dot = lgh*u

%% Control design
% y_des = desired_path(t);
% u = (y_dot_des - k(y_des-y))/lgh;

% simulate system
t_stop = 18;
[t, robot_path] = ode45(@controller, [0 t_stop], [0 0 0]');

% compute maximum error while painting
[max_error, max_idx] = compute_trajectory_error(t, robot_path);
disp(['Maximum error while painting: ' num2str(max_error) ' m']);

% plot results
visualize_robot_path(robot_path);