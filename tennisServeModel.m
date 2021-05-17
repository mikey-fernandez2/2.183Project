% 2.183 Project
% 05/05/2021
% Calculate the motion for the 3-link arm during a serve

%%
clc; close all; clear all;

%% Parameters
global p th_eq timeVec % parameters

load('EQTrajectory.mat', 'th_eq', 'timeVec')

% set parameter values
% Reference:
% https://link.springer.com/content/pdf/10.1007%2F978-3-319-14418-4_147.pdf (Winter Values)
w = 65;                                         % kg, total body mass
m1 = .028*w; m2 = 0.016*w; m3 = 0.006*w;        % kg, mass of each link
l1 = 0.286; l2 = 0.269; l3 = 0.08;              % m, link lengths
r1 = 0.484*l1; r2 = 0.439*l2; r3 = 0.506*l3;    % m, distance to center of mass of each link
rg1 = 0.322*l1; rg2 = 0.303*l2; rg3 = 0.297*l3; % m, radius of gyration of each link 
Ic1 = m1*rg1^2; Ic2 = m2*rg2^2; Ic3 = m3*rg3^2; % kg-m^2, moment of inertia about mass center
g = 9.81;                                       % m/s^2, gravity constant
k1 = 0; k2 = 0; k3 = 0;                         % N-m/rad, joint stiffness constants
b1 = 0; b2 = 0; b3 = 0;                         % N-m-s/rad, joint damping constants

l_max = l1 + l2 + l3;                           % m, maximum arm reach

p = [m1 m2 m3 Ic1 Ic2 Ic3 l1 l2 l3 r1 r2 r3 g k1 k2 k3 b1 b2 b3]';

%% Calculate and plot motions
th0 = 0.9*th_eq(:, 1);
% th0 = [0; 0; 0];
om0 = [0; 0; 0];
z0 = [th0; om0];

freq = 100;
tSpan = 0:1/freq:timeVec(end);

[tOut, yOut] = ode45(@stateEqs, tSpan, z0);
th1 = yOut(:, 1); th2 = yOut(:, 2); th3 = yOut(:, 3);
om1 = yOut(:, 4); om2 = yOut(:, 5); om3 = yOut(:, 6);

figure
for i = 1:length(tOut)
    pos_links = keypoints_tennisServe(yOut(i, :)', p);
    xCoords = [0 pos_links(1, :)];
    yCoords = [0 pos_links(2, :)];
    
    plot(xCoords, yCoords)
    xlabel('x (m)')
    ylabel('y (m)')
    title(['t = ', num2str(tOut(i), '%.3f'), ' s']) 
    xlim([-l_max l_max]*1.1)
    ylim([-l_max l_max]*1.1)
    
    pause(0.001)
end

%% Functions
function dxdt = stateEqs(t, z)
% z = [th1; th2; th3; om1; om2; om3]
    global p th_eq timeVec
    
    dxdt = zeros(6, 1);
    
    % get input force, torque (SET ME AS FUNCTIONS!)
    F = [0; 0];
%     T = [0; 0; 0];
    T = eqPoint(t, z, p, th_eq, timeVec);
    u = [F; T];
    
    % get velocities (the last three states)
    om = z(4:6);
   
    % get inertia matrix and "rest of" equations of motion
    A = A_tennisServe(z, p);
    b = b_tennisServe(z, u, p);
    
    % Then A\b solves for the acceleration at this time step
    acc = A\b;

    % get the time derivative of the state vector
    dxdt(1:3) = om;
    dxdt(4:6) = acc;
end