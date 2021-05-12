
function nOut = eqPoint(t,z)
    % Compute Equilibrium-Point Motion
    % z = [th1; th2; th3; om1; om2; om3];
    
    % Get desired/equilibrium joint angles
    th_eq = getVideoPoints("tennis_serve_video.mov");
   
    th_act1 = z(1);
    th_act2 = z(2);
    th_act3 = z(3);
    th_act =[th_act1;th_act2;th_act3];
    
    om_act1 = z(4);
    om_act2 = z(5);
    om_act3 = z(6);
    om_act = [om_act1; om_act2; om_act3];
    
    % Equilibrium-Point Model
    nOut = K.*(th_eq - th_act) - B.*(om_act); % nOut is the resultant joint torque vector
    
end
