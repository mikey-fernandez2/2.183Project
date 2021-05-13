
% This script outputs joint torque vector from equilibrium point model.

    % Compute Equilibrium-Point Motion
    % z = [th1; th2; th3; om1; om2; om3];
   
    % Get desired/equilibrium joint angles
    th_eq = getVideoPoints(3, "tennis_serve_video.mov"); % t=3s
    %th_eq = [th1_eq, th2_eq, th3_eq];
    
    th_act1 = th1(1:300);
    th_act2 = th2(1:300);
    th_act3 = th3(1:300);
    th_act =[th_act1, th_act2, th_act3];
    
    om_act1 = om1(1:300);
    om_act2 = om2(1:300);
    om_act3 = om3(1:300);
    om_act = [om_act1, om_act2, om_act3];
    
    % Tunable Parameters
    k=[1,2,3];
    b=[1,2,3];
    K = diag(k);
    B = diag(b);
    
    % Equilibrium-Point Model
    nOut = K*(th_eq - th_act)' - B*(om_act)'; % nOut is the resultant joint torque vector
