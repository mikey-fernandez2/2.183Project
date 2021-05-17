% function getVideoPoints(fileName)
% Get points from a video and convert them to joint trajectories
close all; clc;
fileName = "tennis_serve_video.mov";
    video = VideoReader(fileName); % 2.11 seconds
    time = video.Duration; % video length, sec
    videoFrames = video.NumFrames; % 128 frames
    frames = 1:10:videoFrames - 56; % get points from 8 frames

    videoPoints = [];
    counter = 1;
    
    for i = frames
       currentFrame = read(video, i);
       imshow(currentFrame);
       
       [rows, cols, colors] = size(currentFrame);
       
       x = []; y = [];
       while(length(x) ~= 4 || length(y) ~= 4)
        [x, y] = getpts;
       end
       
       coords = [x (rows - y + 1)];
       videoPoints(:, :, counter) = coords;
       counter = counter + 1;
    end
    
    close all;
    
    % Fit the data to a curve, interpolate points & calculate joint angles.

    % Separate coordinates for each point tracked.
    shoulder = squeeze(videoPoints(1, :, :));
    shoulder_X = shoulder(1, :)'; shoulder_Y = shoulder(2, :)';

    elbow = squeeze(videoPoints(2, :, :));
    elbow_X = elbow(1, :)'; elbow_Y = elbow(2, :)';
    
    wrist = squeeze(videoPoints(3, :, :));
    wrist_X = wrist(1, :)'; wrist_Y = wrist(2, :)';
    
    hand = squeeze(videoPoints(4, :, :));
    hand_X = hand(1, :)'; hand_Y = hand(2, :)';

    freq = 100;
    numPoints = time/(1/freq); % specify number of points

    shoulderTraj = fit(shoulder_X, shoulder_Y, 'smoothingspline');
    shoulder_interp_x = linspace(shoulder_X(1), shoulder_X(end), numPoints)';
    shoulder_interp_y = feval(shoulderTraj, shoulder_interp_x);
    
    elbowTraj = fit(elbow_X, elbow_Y, 'smoothingspline');
    elbow_interp_x = linspace(elbow_X(1), elbow_X(end), numPoints)'; 
    elbow_interp_y = feval(elbowTraj, elbow_interp_x);

    wristTraj = fit(wrist_X, wrist_Y, 'smoothingspline');
    wrist_interp_x = linspace(wrist_X(1), wrist_X(end), numPoints)'; 
    wrist_interp_y = feval(wristTraj, wrist_interp_x);
    
    handTraj = fit(hand_X, hand_Y, 'smoothingspline');
    hand_interp_x = linspace(hand_X(1), hand_X(end), numPoints)'; 
    hand_interp_y = feval(handTraj, hand_interp_x);
    
    upperArmDir = [elbow_interp_x - shoulder_interp_x, elbow_interp_y - shoulder_interp_y];
    forearmDir = [wrist_interp_x - elbow_interp_x, wrist_interp_y - elbow_interp_y];
    handDir = [hand_interp_x - wrist_interp_x, hand_interp_y - wrist_interp_y];
    
    th1_abs = atan2(upperArmDir(:, 2), upperArmDir(:, 1));
    th2_abs = atan2(forearmDir(:, 2), forearmDir(:, 1));
    th3_abs = atan2(handDir(:, 2), handDir(:, 1));
    
    th1_eq = th1_abs;
    th2_eq = th2_abs - th1_abs;
    th3_eq = th3_abs - th2_abs;

    figure
    plot(shoulderTraj, shoulder_X, shoulder_Y);
    title("Shoulder")
    xlim([0 cols]); ylim([0 rows])

    figure
    plot(elbowTraj, elbow_X, elbow_Y);
    title("Elbow")
    xlim([0 cols]); ylim([0 rows])

    figure
    plot(wristTraj, wrist_X, wrist_Y);
    title("Wrist")
    xlim([0 cols]); ylim([0 rows])
   
    figure
    plot(handTraj, hand_X, hand_Y);
    title("Hand")
    xlim([0 cols]); ylim([0 rows])
    
    th_eq = [th1_eq'; th2_eq'; th3_eq'];
    
    figure
    hold on
    plot(th1_eq)
    plot(th2_eq)
    plot(th3_eq)
    legend({'1', '2', '3'})
    
    timeVec = 0:1/freq:time;
    timeVec = timeVec(1:length(th_eq));
    save('EQTrajectory.mat', 'th_eq', 'timeVec');
% end
