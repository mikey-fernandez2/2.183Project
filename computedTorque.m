function nOut = computedTorque(t, p, posTraj, velTraj, accTraj)
% calculates the torque necessary at time t to execute the desired
% kinematics

frameRate = 60; % FPS
timeVec = 0:1/frameRate:(length(posTraj)-1)/frameRate; % s

[~, closestIndex] = min(abs(t - timeVec)); % get the desired time
% timeVec(end)
% length(timeVec)
% disp(closestIndex)
desPos = posTraj(:, closestIndex);
desVel = velTraj(:, closestIndex);
desAcc = accTraj(:, closestIndex);

desState = [desPos; desVel];
% change p here, these p = [m1 m2 m3 Ic1 Ic2 Ic3 l1 l2 l3 r1 r2 r3 g k1 k2 k3 b1 b2 b3]';

I = A_tennisServe(desState, p);
Corr = Corr_Terms(desState, p);
Grav = Grav_Terms(desState, p);
Damping = Damping_Matrix(desState, p);
Stiffness = Stiffness_Matrix(desState, p);

nOut = I*desAcc + Corr + Grav + Damping*desVel + Stiffness*desPos;
% nOut = I*desAcc + Corr + Grav + Stiffness*desPos;


end