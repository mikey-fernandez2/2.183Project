function extractTrajectory(fileName)
% EXTRACTTRAJECTORY gets the angular position, velocity, and acceleration
% from fileName

% fileName = "MoCap.csv";

raw = readtable(fileName);
frameRate = 60; % FPS
frameTime = 1/frameRate; % s

shoulderAng = raw.shoulder_angle;
elbowAng = raw.elbow_angle;
wristAng = raw.wrist_angle;

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

