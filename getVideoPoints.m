%function getVideoPoints(fileName)
% Get points from a video and convert them to joint trajectories
close all; clc;
 fileName = "tennis_serve_video.mov";
    video = VideoReader(fileName); % 2.11 seconds
    time = video.Duration; % video length, sec
    videoFrames = video.NumFrames; % 127 frames
    frames = 1:14:videoFrames; % get points from 9 frames

    videoPoints = [];
    counter = 1;
    
%     for i = frames
%        currentFrame = read(video, i);
%        imshow(currentFrame);
%        
%        [rows, cols, colors] = size(currentFrame);
%        
%        x = []; y = [];
%        while(length(x) ~= 4 || length(y) ~= 4)
%         [x, y] = getpts;
%        end
%        
%        coords = [x (rows - y + 1)];
%        videoPoints(:, :, counter) = coords;
%        counter = counter + 1;
%     end
   
    close all;
    
    load('EQTrajectory.mat', 'videoPoints');
    
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
    
    upperArmDir = [elbow_X - shoulder_X, elbow_Y - shoulder_Y];
    forearmDir = [wrist_X - elbow_X, wrist_Y - elbow_Y];
    handDir = [hand_X - wrist_X, hand_Y - wrist_Y];
    
    th1_abs = atan2(upperArmDir(:, 2), upperArmDir(:, 1)); th1_abs(th1_abs < 0) = 2*pi + th1_abs(th1_abs < 0);
    th2_abs = atan2(forearmDir(:, 2), forearmDir(:, 1)); th2_abs(th2_abs < 0) = 2*pi + th2_abs(th2_abs < 0);
    th3_abs = atan2(handDir(:, 2), handDir(:, 1)); th3_abs(th3_abs < 0) = 2*pi + th3_abs(th3_abs < 0);

    freq = 100;
    numPoints = time/(1/freq); % specify number of points
   

    th1_interp = spline(frames, th1_abs, 1:frames(end));
    th2_interp = spline(frames, th2_abs, 1:frames(end));
    th3_interp = spline(frames, th3_abs, 1:frames(end));
    th1_interp = resample(th1_interp, fix(numPoints), length(th1_interp));
    th2_interp = resample(th2_interp, fix(numPoints), length(th2_interp));
    th3_interp = resample(th3_interp, fix(numPoints), length(th3_interp));
  
    timeVec = 0:1/freq:time;
    timeVec = timeVec(1:length(th1_interp));
    
%     shoulder_interp_x = []; shoulder_interp_y = [];
%     elbow_interp_x = []; elbow_interp_y = [];
%     wrist_interp_x = []; wrist_interp_y = [];
%     hand_interp_x = []; hand_interp_y = [];
%     numPer = floor(numPoints/(length(shoulder) - 1));
%     for i = 1:length(shoulder) - 1
%         if i == length(shoulder) - 1
%             num = numPoints - numPer*(length(shoulder) - 2);
%         else
%             num = numPer;
%         end
%         
%         shoulder_interp_x = [shoulder_interp_x linspace(shoulder_X(i), shoulder_X(i + 1), num)];
%         elbow_interp_x = [elbow_interp_x linspace(elbow_X(i), elbow_X(i + 1), num)];
%         wrist_interp_x = [wrist_interp_x linspace(wrist_X(i), wrist_X(i + 1), num)];
%         hand_interp_x = [hand_interp_x linspace(hand_X(i), hand_Y(i + 1), num)];
%         
%         shoulder_interp_y = [shoulder_interp_y linspace(shoulder_Y(i), shoulder_Y(i + 1), num)];
%         elbow_interp_y = [elbow_interp_y linspace(elbow_Y(i), elbow_Y(i + 1), num)];
%         wrist_interp_y = [wrist_interp_y linspace(wrist_Y(i), wrist_Y(i + 1), num)];
%         hand_interp_y = [hand_interp_y linspace(hand_Y(i), hand_Y(i + 1), num)];
%     end
    
%     shoulderTraj = fit(shoulder_X, shoulder_Y, 'pchipinterp');
%     shoulder_interp_x = linspace(shoulder_X(1), shoulder_X(end), numPoints)';
%     shoulder_interp_y = makima(shoulder_X, shoulder_Y, shoulder_interp_x);
%     shoulder_interp_y = feval(shoulderTraj, shoulder_interp_x);
    
%     elbowTraj = fit(elbow_X, elbow_Y, 'pchipinterp');
%     elbow_interp_x = linspace(elbow_X(1), elbow_X(end), numPoints)'; 
%     elbow_interp_y = feval(elbowTraj, elbow_interp_x);

%     wristTraj = fit(wrist_X, wrist_Y, 'pchipinterp');
%     wrist_interp_x = linspace(wrist_X(1), wrist_X(end), numPoints)'; 
%     wrist_interp_y = feval(wristTraj, wrist_interp_x);
    
%     handTraj = fit(hand_X, hand_Y, 'pchipinterp');
%     hand_interp_x = linspace(hand_X(1), hand_X(end), numPoints)'; 
%     hand_interp_y = feval(handTraj, hand_interp_x);
    
%     upperArmDir = [elbow_interp_x' - shoulder_interp_x', elbow_interp_y' - shoulder_interp_y'];
%     forearmDir = [wrist_interp_x' - elbow_interp_x', wrist_interp_y' - elbow_interp_y'];
%     handDir = [hand_interp_x' - wrist_interp_x', hand_interp_y' - wrist_interp_y'];
%     
%     th1_abs = atan2(upperArmDir(:, 2), upperArmDir(:, 1));
%     th2_abs = atan2(forearmDir(:, 2), forearmDir(:, 1));
%     th3_abs = atan2(handDir(:, 2), handDir(:, 1));
    
%     th1_abs = atan(upperArmDir(:, 2)./upperArmDir(:, 1));
%     th2_abs = atan(forearmDir(:, 2)./forearmDir(:, 1));
%     th3_abs = atan(handDir(:, 2)./handDir(:, 1));
    
    th1_eq = th1_interp;
    th2_eq = th2_interp - th1_interp;
    th3_eq = th3_interp - th2_interp;

%     figure
%     plot(shoulderTraj, shoulder_X, shoulder_Y);
%     title("Shoulder")
%     xlim([0 cols]); ylim([0 rows])
% 
%     figure
%     plot(elbowTraj, elbow_X, elbow_Y);
%     title("Elbow")
%     xlim([0 cols]); ylim([0 rows])
% 
%     figure
%     plot(wristTraj, wrist_X, wrist_Y);
%     title("Wrist")
%     xlim([0 cols]); ylim([0 rows])
%    
%     figure
%     plot(handTraj, hand_X, hand_Y);
%     title("Hand")
%     xlim([0 cols]); ylim([0 rows])
%     
    th_eq = [th1_eq; th2_eq; th3_eq];
    
    figure
    hold on
    plot(th1_interp)
    plot(th2_interp)
    plot(th3_interp)
    legend({'Upper Arm', 'Forearm', 'Hand'})
    title('Interpolated Limb Trajectories, fitted with a Smoothing Spline');
    
    figure
    hold on
    plot(th1_eq)
    plot(th2_eq)
    plot(th3_eq)
    legend({'Shoulder', 'Elbow', 'Wrist'})
    title('Joint Equilibrium Trajectories');
    
    save('EQTrajectory.mat', 'th_eq', 'timeVec', 'videoPoints');
%end

