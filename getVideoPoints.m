
function th_eq = getVideoPoints(t, fileName)
% Extrapolate 8 3x2 joint motion trajectory coordinates from video.
    %clc; close all;
    video = VideoReader(fileName); % 2.11 seconds
    videoFrames = video.NumFrames; % 128 frames
    frame = 1:10:videoFrames-56; % get 8 points
    
    videoPoints = [];
    counter = 1;
   for i=frame
       currentFrame = read(video, i);
       imshow(currentFrame);
       [rows, cols, colors] = size(currentFrame);
       [x,y] = getpts;
       if(length(x) ~= 10 || length(y) ~= 10)
        [x, y] = getpts;
       end
       coords = [x (rows-y+1)];
       videoPoints(:,:,counter)=coords;
       counter = counter + 1;
   end 
  % Fit the data to a curve, interpolate points & calculate joint angles.
  
  % Separate coordinates for each joint.
  shoulder = videoPoints(1,:,:); 
  shoulder_X = reshape(shoulder(:,1,:), [8 ,1]); 
  shoulder_Y = reshape(shoulder(:,2,:), [8,1]);
  elbow = videoPoints(2,:,:); 
  elbow_X = reshape(elbow(:,1,:), [8,1]); 
  elbow_Y = reshape(elbow(:,2,:),[8,1]);
  wrist = videoPoints(3,:,:); 
  wrist_X = reshape(wrist(:,1,:),[8,1]); 
  wrist_Y = reshape(wrist(:,2,:),[8,1]);
  
  freq = 100;
  sampleRate = t/(1/freq); % specify number of points 
  %sampleRate = 300;
  figure; title("Shoulder");
  SplineFit = fit(shoulder_X,shoulder_Y, 'smoothingspline');
  plot(SplineFit, shoulder_X, shoulder_Y);
  interpolated_x = linspace(shoulder_X(1), shoulder_X(end),sampleRate);
  interpolated_y = feval(SplineFit, interpolated_x);
  th1_eq = atan(interpolated_y./interpolated_x');
  figure; title("Elbow");
  SplineFit2 = fit(elbow_X,elbow_Y, 'smoothingspline');
  plot(SplineFit2, elbow_X, elbow_Y);
  interpolated_x2 = linspace(elbow_X(1), elbow_X(end),sampleRate); 
  interpolated_y2 = feval(SplineFit2, interpolated_x2);
  th2_eq = atan(interpolated_y2./interpolated_x2');
  figure; title("Wrist");
  SplineFit3 = fit(wrist_X,wrist_Y, 'smoothingspline');
  plot(SplineFit3, wrist_X, wrist_Y);
  interpolated_x3 = linspace(wrist_X(1), wrist_X(end),sampleRate); 
  interpolated_y3 = feval(SplineFit3, interpolated_x3);
  th3_eq = atan(interpolated_y3./interpolated_x3');
  
  th_eq = [th1_eq, th2_eq, th3_eq];
  
end
