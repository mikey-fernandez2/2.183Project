% 2.183 Project
% 05/05/2021
% Calculate the Equations of Motion for the 3-link arm

%%
clc; close all; clear all;

%% Parameters
global r1 r2 r3 l1 l2 l3 m1 m2 m3 I1 I2 I3 Ic1 Ic2 Ic3 g K B % parameters

% set parameter values
l1 = 1; l2 = 1; l3 = 1;                                     % m, link lengths
r1 = l1/2; r2 = l2/2; r3 = l3/2;                            % m, distance to center of mass of each link
m1 = 1; m2 = 1; m3 = 1;                                     % kg, mass of each link
Ic1 = 1/12*m1*l1^2; Ic2 = 1/12*m2*l2^2; Ic3 = 1/12*m3*l3^2; % kg-m^2, moment of inertia about mass center
I1 = Ic1 + m1*r1^2; I2 = Ic2 + m2*r2^2; I3 = Ic3 + m3*r3^2; % kg-m^2, moment of inertia about each joint
g = 9.81;                                                   % m/s^2, gravity constant
k1 = 0; k2 = 0; k3 = 0;                                     % N-m/rad, joint stiffness constants
b1 = 0; b2 = 0; b3 = 0;                                     % N-m-s/rad, joint damping constants

K = diag([k1 k2 k3]);
B = diag([b1 b2 b3]);

%% Calculate and plot motions
th0 = [0; 0; 0];
om0 = [0; 0; 0];
x0 = [th0; om0];

freq = 1000;
tSpan = 0:1/freq:3;

[tOut, yOut] = ode45(@stateEqs, tSpan, x0);
th1 = yOut(:, 1); th2 = yOut(:, 2); th3 = yOut(:, 3);
om1 = yOut(:, 4); om2 = yOut(:, 5); om3 = yOut(:, 6);

% get time paths for ends of each link
x1 = l1*cos(th1);                  y1 = l1*sin(th1);
x2 = x1 + l2*cos(th1 + th2);       y2 = y1 + l2*sin(th1 + th2);
x3 = x2 + l3*cos(th1 + th2 + th3); y3 = y2 + l3*sin(th1 + th2 + th3);

figure
for i = 1:length(tOut)
    xCoords = [0 x1(i) x2(i) x3(i)];
    yCoords = [0 y1(i) y2(i) y3(i)];
    
    plot(xCoords, yCoords)
    xlabel('x (m)')
    ylabel('y (m)')
    title(['t = ', num2str(tOut(i), '%.3f'), ' s']) 
    xlim([-3 3])
    ylim([-3 3])
    
    pause(0.01)
end

%% Functions
function dxdt = stateEqs(t, x)
    global r1 r2 r3 l1 l2 l3 m1 m2 m3 I1 I2 I3 g K B
    
    % extract states
    th1 = x(1); om1 = x(4);
    th2 = x(2); om2 = x(5);
    th3 = x(3); om3 = x(6);
    th = [th1; th2; th3];
    om = [om1; om2; om3];
    
    % get input force, torque
    F = [0; 0];
    T = [0; 0; 0];
    
    % get configuration dependent matrices
    I = [I1 + I2 + I3 + m2*l2^2 + m3*l1^2 + m3*l2^2 + m2*l1*r2*cos(th2) + m3*l1*l2*cos(th2) + m3*l1*r3*cos(th2 + th3) + m3*l2*r3*cos(th2) I2 + I3 + m3*l2^2 + m2*l1*r2*cos(th2) + m3*l1*l2*cos(th2) + m3*l1*r3*cos(th2 + th3) + 2*m3*l2*r3*cos(th3) I3 + m3*l1*r3*cos(th2 + th3) + m3*l2*r3*cos(th3);
         I2 + I3 + m3*l2^2 + m2*l1*r2*cos(th2) + m3*l1*l2*cos(th2) + m3*l1*r3*cos(th2 + th3) + 2*m3*l2*r3*cos(th3)                        I2 + I3 + m3*l2^2 + m3*l2*r3*cos(th3)                                                                     I3 + m3*l2*r3*cos(th3);
         I3 + m3*l1*r3*cos(th2 + th3) + m3*l2*r3*cos(th3)                                                                                 I3 + m3*l2*r3*cos(th3)                                                                                    I3]; 

    C = [(0)                                                                                                                       (-(m2*l1*r2*sin(th2) + m3*l1*l2*sin(th2) + m3*l1*r3*sin(th2 + th3) + m3*l2*r3*sin(th2))*om1 - (m2*l1*r2*sin(th2) + m3*l1*l2*sin(th2) + m3*l1*r3*sin(th2 + th3))*om2 - (2*m3*l1*r3*sin(th2 + th3) + 2*m3*l2*r3*sin(th3))*om3) (-(m3*l1*r3*sin(th2 + th3))*om1 - (m3*l1*r3*sin(th2 + th3) + m3*l2*r3*sin(th3))*om3);
         (1/2*(m2*l1*r2*sin(th2) + m3*l1*l2*sin(th2) + m3*l1*r3*sin(th2 + th3) + m3*l2*r3*sin(th2))*om1 + 2*m3*l2*r3*sin(th3)*om2) (0)                                                                                                                                                                                                                          (-(m3*l2*r3*sin(th3))*om1 - m3*l2*r3*cos(th3)*om2 - m3*l2*r3*sin(th3)*om3);
         (1/2*(m3*l1*r3*sin(th2 + th3))*om1 - 2*m3*l2*r3*sin(th3)*om2)                                                             (1/2*(m3*l2*r3*sin(th3))*om2)                                                                                                                                                                                                (0)];

    G = [g*(m1*r1*cos(th1) + m2*(l1*cos(th1) + r2*cos(th1 + th2)) + m3*(l1*cos(th1) + l2*cos(th1 + th2) + r3*cos(th1 + th2 + th3)));
         g*(m2*r2*cos(th1 + th2) + m3*(l2*cos(th1 + th2) + r3*cos(th1 + th2 + th3)));
         g*(m3*r3*cos(th1 + th2 + th3))];

    Je = [-l1*sin(th1) - l2*sin(th1 + th2) - l3*sin(th1 + th2 + th3), -l2*sin(th1 + th2) - l3*sin(th1 + th2 + th3), -l3*sin(th1 + th2 + th3);
           l1*cos(th1) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3),  l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3),  l3*cos(th1 + th2 + th3)];

    % get the time derivative of the state vector
    dxdt = zeros(6, 1);
    dxdt(1:3) = x(4:6);
    dxdt(4:6) = I\(T + Je'*F - C*om - G - K*th - B*om);
end