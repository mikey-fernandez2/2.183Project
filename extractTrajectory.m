function extractTrajectory(fileName)
% EXTRACTTRAJECTORY gets the angular position, velocity, and acceleration
% from fileName

% fileName = "MoCap.csv";

%raw = readtable(fileName);
% shoulderAng = raw.shoulder_angle;
% elbowAng = raw.elbow_angle;
% wristAng = raw.wrist_angle;

% loading the data
data  = load(fileName); % ex: 'Direction2896.mat'
data = data.Direction2896;
frameRate = 60; % FPS
frameTime = 1/frameRate; % s

% establishing frames
frame1 = table2array(data);
frame1(end,:) = [];
frame2 = table2array(data);
frame2(1,:) = [];

%calculating theta
multiplyframes12 = frame1.*frame2;
dotproductframes12 = [multiplyframes12(:,1)+multiplyframes12(:,2) ...
                     multiplyframes12(:,3)+multiplyframes12(:,4) ...
                     multiplyframes12(:,5)+multiplyframes12(:,6)];

theta12 = acos(dotproductframes12); % distance in radians from frame 1 to 2

shoulderAng = theta12(:,1);
elbowAng = theta12(:,2);
wristAng = theta12(:,3);

shoulderVel = diff(shoulderAng)/frameTime; shoulderVel = [shoulderVel; shoulderVel(end)];
elbowVel = diff(elbowAng)/frameTime; elbowVel = [elbowVel; elbowVel(end)];
wristVel = diff(wristAng)/frameTime; wristVel = [wristVel; wristVel(end)];

shoulderAcc = diff(shoulderAng, 2)/frameTime^2; shoulderAcc = [shoulderAcc(1); shoulderAcc; shoulderAcc(end)];
elbowAcc = diff(elbowAng, 2)/frameTime^2; elbowAcc = [elbowAcc(1); elbowAcc; elbowAcc(end)];
wristAcc = diff(wristAng, 2)/frameTime^2; wristAcc = [wristAcc(1); wristAcc; wristAcc(end)];

pos = [shoulderAng'; elbowAng'; wristAng'];
vel = [shoulderVel'; elbowVel'; wristVel'];
acc = [shoulderAcc'; elbowAcc'; wristAcc'];

save('ComputedTorqueTrajectory.mat', 'pos', 'vel', 'acc')

end