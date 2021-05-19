% plot ground truth
close all;

w = 65;                                         % kg, total body mass
m1 = .028*w; m2 = 0.016*w; m3 = 0.006*w;        % kg, mass of each link
l1 = 0.286; l2 = 0.269; l3 = 0.08;              % m, link lengths
r1 = 0.484*l1; r2 = 0.439*l2; r3 = 0.506*l3;    % m, distance to center of mass of each link
rg1 = 0.322*l1; rg2 = 0.303*l2; rg3 = 0.297*l3; % m, radius of gyration of each link 
Ic1 = m1*rg1^2; Ic2 = m2*rg2^2; Ic3 = m3*rg3^2; % kg-m^2, moment of inertia about mass center
g = 9.81;                                       % m/s^2, gravity constant
k1 = 10; k2 = 10; k3 = 10;                         % N-m/rad, joint stiffness constants
b1 = .1; b2 = 0.1; b3 = 0.1;                         % N-m-s/rad, joint damping constants
l_max = l1 + l2 + l3;                           % m, maximum arm reach

p = [m1 m2 m3 Ic1 Ic2 Ic3 l1 l2 l3 r1 r2 r3 g k1 k2 k3 b1 b2 b3]';

freq = 60;


load('ComputedTorqueTrajectory.mat', 'pos', 'vel', 'acc') % load trajectory

framesToTake = 1333:240+1333;
th = pos(:, framesToTake); th1 = th(1, :); th2 = th(2, :); th3 = th(3, :);
om = zeros(size(th));
states = [th; om];

posEE = position_endEffector(states, p);
velEE = velocity_endEffector(states, p);

figure
for i = 1:length(th)
    pos_links = keypoints_tennisServe(states(:, i), p);
    xCoords = [0 pos_links(1, :)];
    yCoords = [0 pos_links(2, :)];
    
    plot(xCoords, yCoords)
    xlabel('x (m)')
    ylabel('y (m)')
    title(['t = ', num2str(i/60, '%.3f'), ' s']) 
    xlim([-l_max l_max]*1.1)
    ylim([-l_max l_max]*1.1)
    
    pause(0.001)
end
figTimeVec = 0:1/freq:4;
% Plot Joint Trajectories
figure
plot(figTimeVec, th1, 'b')
hold on
plot(figTimeVec, th2, 'r')
plot(figTimeVec, th3, 'g')
title("Joint Trajectories")

% Plot Hand Trajectory
figure
plot(posEE(1, :), posEE(2, :))
title("Hand Trajectory");

% Plot Hand Velocity
figure
plot(figTimeVec, velEE)
title("Hand Velocity");