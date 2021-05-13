% This function outputs joint torque vector from equilibrium point model.
function nOut = eqPoint(t,z)
    % Compute Equilibrium-Point Motion
    % z = [th1; th2; th3; om1; om2; om3];
   
    % Get desired/equilibrium joint angles
    th_eq = getVideoPoints(3, "tennis_serve_video.mov"); % t=3s
    
    th_act1 = z(1); th_act2 = z(2); th_act3 = z(3);
    th_act =[th_act1, th_act2, th_act3];
    
    om_act1 = z(4); om_act2 = z(5); om_act3 = z(6);
    om_act = [om_act1, om_act2, om_act3];
    
    % Tunable Parameters
    w = 7; % natural frequency of upper arm (Hz)
    % get joint stiffnesses by assuming natural frequency
    k_shoulder = w^2*m1;
    k_elbow = w^2*m2;
    k_wrist = w^2*m3;
    k=[k_shoulder, k_elbow, k_wrist];
    % get damping coefficients from critically damped damping ratio
    b_shoulder = 2*sqrt(k_shoulder/m1);
    b_elbow = 2*sqrt(k_elbow/m2);
    b_wrist = 2*sqrt(k_wrist/m3);
    b=[b_shoulder, b_elbow, b_wrist]; 
    K = diag(k);
    B = diag(b);
    
    % Equilibrium-Point Model
    nOut = K*(th_eq(t) - th_act(t))' - B*(om_act(t))'; % nOut is the resultant joint torque vector
end
