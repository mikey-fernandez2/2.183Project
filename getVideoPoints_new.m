%function getVideoPoints(fileName)
% Get points from a video and convert them to joint trajectories
close all; clc;
 
    fileName = "MoCap.csv";

    raw = readtable(fileName);
    freq = 60; % FPS

    frames = 1333:14:240+1333; %size(raw,1);
    time = (frames(end) - frames(1))/freq;
    numPoints = time/(1/freq); % specify number of points
    
    % Extract frames from raw mocap data.
    shoulder_X = raw.shoulder_x(frames); shoulder_Y = raw.shoulder_y(frames);
    elbow_X = raw.elbow_x(frames); elbow_Y = raw.elbow_y(frames);
    wrist_X = raw.wrist_x(frames); wrist_Y = raw.wrist_y(frames);
    hand_X = raw.racket_x(frames); hand_Y = raw.racket_y(frames);
    
    upperArmDir = [elbow_X - shoulder_X, elbow_Y - shoulder_Y];
    forearmDir = [wrist_X - elbow_X, wrist_Y - elbow_Y];
    handDir = [hand_X - wrist_X, hand_Y - wrist_Y];
    
    th1_abs = -atan2(upperArmDir(:, 2), upperArmDir(:, 1)); %th1_abs(th1_abs < 0) = 2*pi + th1_abs(th1_abs < 0);%ind1 = find(th1_abs > 0, 1, 'first'); th1_abs(1:ind1 - 1) = 2*pi + th1_abs(1:ind1 - 1);
    th2_abs = -atan2(forearmDir(:, 2), forearmDir(:, 1)); %th2_abs(th2_abs < 0) = 2*pi + th2_abs(th2_abs < 0);
    %th3_abs = -atan2(handDir(:, 2), handDir(:, 1)); %th3_abs(th3_abs < 0) = 2*pi + th3_abs(th3_abs < 0);
    th3_abs = [0,0,0,0,0,pi/4,pi/2,pi/2,pi/2,pi/4,pi/4,0,0,0,0,0,0,0];


    th1_interp = spline(frames, th1_abs, frames(1):frames(end));
    th2_interp = spline(frames, th2_abs, frames(1):frames(end));
    th3_interp = spline(frames, th3_abs, frames(1):frames(end));
    th1_interp = resample(th1_interp, fix(numPoints), length(th1_interp));
    th2_interp = resample(th2_interp, fix(numPoints), length(th2_interp));
    th3_interp = resample(th3_interp, fix(numPoints), length(th3_interp));
    
    th1_eq = th1_interp;
    th2_eq = th2_interp - th1_interp;
    th3_eq = th3_interp;
 
    timeVec = 0:1/freq:time;
    timeVec = timeVec(1:length(th1_interp));
    
    th_eq = [th1_eq; th2_eq; th3_eq];
    
    figure
    hold on
    plot(th1_interp)
    plot(th2_interp)
    plot(th3_interp)
    %xlim([1333 frames(end)])
    legend({'Upper Arm', 'Forearm', 'Hand'})
    title('Interpolated Limb Trajectories, fitted with a Smoothing Spline');
    
    figure
    hold on
    plot(th1_eq)
    plot(th2_eq)
    plot(th3_eq)
    legend({'Shoulder', 'Elbow', 'Wrist'})
    title('Joint Equilibrium Trajectories');
    
    save('EQTrajectory_new.mat', 'th_eq', 'timeVec');
%end
