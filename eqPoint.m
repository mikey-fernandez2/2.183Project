% This function outputs joint torque vector from equilibrium point model.
function nOut = eqPoint(t, z, p, th_eq, timeVec)
    % Compute Equilibrium-Point Motion
    % z = [th1; th2; th3; om1; om2; om3];
   
    % Get desired/equilibrium joint angles
%     th_eq = getVideoPoints(t, "tennis_serve_video.mov"); % works with t=3s right now, (300 samples)
%     load('EQTrajectory.mat', 'th_eq', 'timeVec')
    
    th_1 = z(1); th_2 = z(2); th_3 = z(3);
    th = [th_1; th_2; th_3];
    
    om_1 = z(4); om_2 = z(5); om_3 = z(6);
    om = [om_1; om_2; om_3];
    
    % Tunable Parameters
    wnHz = 3; % natural frequency of upper arm (Hz)
    wn = 2*pi*wnHz;
    Ic1 = p(4); Ic2 = p(5); Ic3 = p(6);
    % get joint stiffnesses by assuming natural frequency
    k_shoulder = wn^2*Ic1;
    k_elbow = wn^2*Ic2;
    k_wrist = wn^2*Ic3;
    k=[k_shoulder, k_elbow, k_wrist];
    % get damping coefficients from critically damped damping ratio
    b_shoulder = 2*sqrt(k_shoulder/Ic1);
    b_elbow = 2*sqrt(k_elbow/Ic2);
    b_wrist = 2*sqrt(k_wrist/Ic3);
    b=[b_shoulder, b_elbow, b_wrist]; 
    K = diag(k);
    B = diag(b);
    
    [~, closestIndex] = min(abs(t - timeVec));
    
    % Equilibrium-Point Model
    nOut = K*(th_eq(:, closestIndex) - th) - B*(om); % nOut is the resultant joint torque vector
end
