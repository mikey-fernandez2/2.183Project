function nOut = computedTorque(t, p, posTraj, velTraj, accTraj)
% calculates the torque necessary at time t to execute the desired
% kinematics

frameRate = 60; % FPS
timeVec = 0:1/frameRate:length(posTraj)/frameRate; % s

[~, closestIndex] = min(abs(t - timeVec)); % get the desired time

desPos = posTraj(:, closestIndex);
desVel = velTraj(:, closestIndex);
desAcc = accTraj(:, closestIndex);

desState = [desPos; desVel];

I = A_tennisServe(desState, p);
Corr = Corr_Terms(desState, p);
Grav = Grav_Terms(desState, p);

nOut = I*desAcc + Corr + Grav;

end