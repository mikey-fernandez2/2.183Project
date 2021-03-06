% 2.183 Project
% 05/05/2021
% Calculate the motion for the 3-link arm during a serve

%%
clc; close all; clear all;

%% Parameters
global p th_eq timeVec pos vel acc CONTROLLER % parameters

CONTROLLER = true; % false for EQ Point, true for Computed Torque

load('EQTrajectory_new.mat', 'th_eq', 'timeVec')
load('ComputedTorqueTrajectory.mat', 'pos', 'vel', 'acc')

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
k1 = 2; k2 = 2; k3 = 2;                         % N-m/rad, joint stiffness constants
b1 = 0.1; b2 = 0.1; b3 = 0.1;                         % N-m-s/rad, joint damping constants
l_max = l1 + l2 + l3;                           % m, maximum arm reach

p = [m1 m2 m3 Ic1 Ic2 Ic3 l1 l2 l3 r1 r2 r3 g k1 k2 k3 b1 b2 b3]';

%% Calculate and plot motions
if ~CONTROLLER
    th0 = 0.9*th_eq(:, 1);
else
    th0 = pos(:, 1);
end

om0 = [0; 0; 0];
z0 = [th0; om0];

if ~CONTROLLER
    freq = 60;
    tSpan = 0:1/freq:timeVec(end); %only get 4s
else
    freq = 60;
    tSpan = 0:1/freq:(length(pos) - 1)/freq;
end

[tOut, yOut] = ode45(@stateEqs, tSpan, z0);
th1 = yOut(:, 1); th2 = yOut(:, 2); th3 = yOut(:, 3);
om1 = yOut(:, 4); om2 = yOut(:, 5); om3 = yOut(:, 6);

z = [th1'; th2'; th3'; om1'; om2'; om3'];
posEE = position_endEffector(z, p);
velEE = velocity_endEffector(z, p);
speedEE = vecnorm(velEE);

%%
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

% Plot Joint Trajectories
figure
plot(tOut, th1, 'b')
hold on
plot(tOut, th2, 'r')
plot(tOut, th3, 'g')
legend({'Shoulder', 'Elbow', 'Wrist'});
title('Joint Trajectories')
xlabel('Time (s)')
ylabel('\theta (rad)')

% Plot Hand Trajectory
figure
hold on
plot(posEE(1, :), posEE(2, :), 'b');
plot(posEE(1, 1), posEE(2, 1), '*r')
plot(posEE(1, end), posEE(2, 2), '*g')
legend({'Trajectory', 'Start', 'End'})
title('Hand Trajectory');
xlabel('x (m)')
ylabel('y (m)')
xlim([-l_max l_max]*1.1)
ylim([-l_max l_max]*1.1)

% Plot Hand Velocity
figure
plot(tOut, speedEE);
title('Hand Speed');
xlabel('Time (s)')
ylabel('Speed (m/s)')

%% Functions
function dxdt = stateEqs(t, z)
% z = [th1; th2; th3; om1; om2; om3]
    global p th_eq timeVec pos vel acc CONTROLLER
    
    dxdt = zeros(6, 1);
    
    % get input force, torque
    F = [0; 0];
    
    if ~CONTROLLER
        T = eqPoint(t, z, p, th_eq, timeVec);
    else
        T = computedTorque(t, p, pos, vel, acc);
    end
    
    u = [F; T];
    
    % get velocities (the last three states)
    om = z(4:6);
   
    % get inertia matrix and "rest of" equations of motion
    A = A_tennisServe(z, p);
    b = b_tennisServe(z, u, p);
    
    % Then A\b solves for the acceleration at this time step
    alpha = A\b;

    % get the time derivative of the state vector
    dxdt(1:3) = om;
    dxdt(4:6) = alpha;
end
