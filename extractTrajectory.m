% function extractTrajectory(fileName)
% EXTRACTTRAJECTORY gets the angular position, velocity, and acceleration
% from fileName

% fileName = "MoCap.csv";
fileName = 'Direction2894.mat';

% put testingStuff here
load('EQTrajectory_new.mat')

frameRate = 60;
frameTime = 1/60;
timeVec = 0:frameTime:frameTime*(length(pos) - 1);

pos = th_eq;
vel = diff(th_eq, 1, 2)/frameTime; vel = [vel vel(:, end)];
% acc = diff(th_eq, 2, 2)./frameTime^2; acc = [acc(:, 1) acc acc(:, end)];

vel(1, :) = smooth(vel(1, :), 3);
vel(2, :) = smooth(vel(2, :), 3);
vel(3, :) = smooth(vel(3, :), 3);

acc = diff(vel, 1, 2)/frameTime; acc = [acc acc(:, end)];

acc(1, :) = smooth(acc(1, :), 30);
acc(2, :) = smooth(acc(2, :), 30);
acc(3, :) = smooth(acc(3, :), 30);

figure
subplot(311)
plot(timeVec, th_eq.')
ylabel('\theta (rad)')
subplot(312)
plot(timeVec, vel.')
ylabel('\omega (rad/s)')
subplot(313)
plot(timeVec, acc.')
ylabel('\alpha (ra/s^2)')
xlabel('Time (s)')
legend({'Shoulder', 'Elbow', 'Wrist'})
sgtitle('Desired Joint Kinematics')

save('ComputedTorqueTrajectory.mat', 'pos', 'vel', 'acc')
gi

% % raw = readtable(fileName);
% % data = raw;
% % shoulderAng = raw.shoulder_angle;
% % elbowAng = raw.elbow_angle;
% % wristAng = raw.wrist_angle;
% 
% % loading the data
% data  = load(fileName); % ex: 'Direction2896.mat'
% data = data.Direction2894;
% frameRate = 60; % FPS
% frameTime = 1/frameRate; % s
% 
% % establishing frames
% frame1 = table2array(data);
% frame1(end,:) = [];
% frame2 = table2array(data);
% frame2(1,:) = [];
% 
% data = table2array(data);
% data = data(1333:240+1333, :);
% shoulder_x = data(:, 1); shoulder_y = data(:, 2);
% elbow_x = data(:, 3); elbow_y = data(:, 4);
% wrist_x = data(:, 5); wrist_y = data(:, 6);
% 
% shoulder_ang = atan2(shoulder_y, shoulder_x);
% elbow_ang = atan2(elbow_y, elbow_x);
% wrist_ang = atan2(wrist_y, wrist_x);
% 
% th_1 = atan2(shoulder_ang - 0, 1 + 0.*shoulder_ang);
% th_2 = atan2(elbow_ang - shoulder_ang, 1 + elbow_ang.*shoulder_ang);
% th_3 = atan2(wrist_ang - elbow_ang, 1 + wrist_ang.*elbow_ang);
% 
% figure
% hold on
% plot(th_1)
% plot(th_2)
% plot(th_3)
% 
% %calculating theta
% multiplyframes12 = frame1.*frame2;
% dotproductframes12 = [multiplyframes12(:,1)+multiplyframes12(:,2) ...
%                      multiplyframes12(:,3)+multiplyframes12(:,4) ...
%                      multiplyframes12(:,5)+multiplyframes12(:,6)];
% 
% theta12 = acos(dotproductframes12); % distance in radians from frame 1 to 2
% 
% % problem: arccos when it goes from 0 to pi
% % can replace arccos with arctan sometimes
% 
% shoulderAng = theta12(:,1);
% elbowAng = theta12(:,2);
% wristAng = theta12(:,3);
% 
% shoulderVel = diff(shoulderAng)/frameTime; shoulderVel = [shoulderVel; shoulderVel(end)];
% elbowVel = diff(elbowAng)/frameTime; elbowVel = [elbowVel; elbowVel(end)];
% wristVel = diff(wristAng)/frameTime; wristVel = [wristVel; wristVel(end)];
% 
% shoulderAcc = diff(shoulderAng, 2)/frameTime^2; shoulderAcc = [shoulderAcc(1); shoulderAcc; shoulderAcc(end)];
% elbowAcc = diff(elbowAng, 2)/frameTime^2; elbowAcc = [elbowAcc(1); elbowAcc; elbowAcc(end)];
% wristAcc = diff(wristAng, 2)/frameTime^2; wristAcc = [wristAcc(1); wristAcc; wristAcc(end)];
% 
% pos = [shoulderAng'; elbowAng'; wristAng'];
% vel = [shoulderVel'; elbowVel'; wristVel'];
% acc = [shoulderAcc'; elbowAcc'; wristAcc'];
% 
% pos = pos(:, 1333:240+1333);
% vel = vel(:, 1333:240+1333);
% acc = acc(:, 1333:240+1333);
% 
% figure
% plot(pos.')
% 
% figure
% plot(vel.')
% 
% figure
% plot(acc.')


% save('ComputedTorqueTrajectory.mat', 'pos', 'vel', 'acc')

% end