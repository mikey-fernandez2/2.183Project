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

% framesToTake = 1333:238+1333;
th = pos(:, :); th1 = th(1, :); th2 = th(2, :); th3 = th(3, :);
om = diff(th, 1, 2); om = [om om(:, end)];
om(:, 1) = smooth(om(:, 1), 3); om(:, 2) = smooth(om(:, 2), 3); om(:, 3) = smooth(om(:, 3), 3);

states = [th; om];

posEE = position_endEffector(states, p);
velEE = velocity_endEffector(states, p);
speedEE = sqrt(velEE(1, :).^2 + velEE(2, :).^2);
tOut - 0:1/freq:length(th)/freq;

figure
for i = 1:length(th)
    pos_links = keypoints_tennisServe(states(:, i), p);
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