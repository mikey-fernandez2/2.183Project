% 2.183 Project
% 05/10/2021
% Derive the equations of motion for the 3 link arm

clear; clc; close all;

name = 'tennisServe';

%% System setup
% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
syms t th1 th2 th3 dth1 dth2 dth3 ddth1 ddth2 ddth3 real
syms m1 m2 m3 Ic1 Ic2 Ic3 l1 l2 l3 r1 r2 r3 g k1 k2 k3 b1 b2 b3 real
syms F_x F_y Tau1 Tau2 Tau3 real

% Group them
q   = [th1; th2; th3];              % generalized coordinates: theta_1, theta_2, theta_3
dq  = [dth1; dth2; dth3];           % first time derivatives
ddq = [ddth1; ddth2; ddth3];        % second time derivatives
u   = [F_x; F_y; Tau1; Tau2; Tau3]; % inputs - racket force, joint torques

% parameters vector
p = [m1 m2 m3 Ic1 Ic2 Ic3 l1 l2 l3 r1 r2 r3 g k1 k2 k3 b1 b2 b3]';

% Generate Unit Vectors and Derivatives
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = cross(ihat,jhat);

xhat = [1; 0; 0];
yhat = [0; 1; 0];

% Link orientations
e1hat = cos(th1)*ihat + sin(th1)*jhat;                         % orientation of link 1
e2hat = cos(th1 + th2)*ihat + sin(th1 + th2)*jhat;             % orientation of link 2
e3hat = cos(th1 + th2 + th3)*ihat + sin(th1 + th2 + th3)*jhat; % orientation of link 3

ddt = @(r) jacobian(r, [q; dq])*[dq; ddq]; % a handy anonymous function for taking time derivatives

% Positions of link ends and centers of mass
pos1 = l1*e1hat;        % position of end of link 1
pos2 = pos1 + l2*e2hat; % position of end of link 2
pos3 = pos2 + l3*e3hat; % position of end of link 3

pos_c1 = r1*e1hat;        % link 1 center of mass
pos_c2 = pos1 + r2*e2hat; % link 2 center of mass
pos_c3 = pos2 + r3*e3hat; % link 3 center of mass

% Velocities of ends of links
v1 = ddt(pos1);
v2 = ddt(pos2);
v3 = ddt(pos3);
a3 = ddt(v3);

% Velocities of centers of mass
v_c1 = ddt(pos_c1);
v_c2 = ddt(pos_c2);
v_c3 = ddt(pos_c3);

% Angular velocities of links
omega1 = dth1;
omega2 = omega1 + dth2;
omega3 = omega2 + dth3;

%% Calculate Energy Terms
% Calculate Kinetic Energy, Potential Energy, and Generalized Forces
F2Q = @(F, r) simplify(jacobian(r, q)'*(F));    % force contributions to generalized forces
M2Q = @(M, w) simplify(jacobian(w, dq)'*(M));   % moment contributions to generalized forces

% Kinetic Energies
T1 = (1/2)*m1*dot(v_c1, v_c1) + (1/2)*Ic1*omega1^2;
T2 = (1/2)*m2*dot(v_c2, v_c2) + (1/2)*Ic2*omega2^2;
T3 = (1/2)*m3*dot(v_c3, v_c3) + (1/2)*Ic3*omega3^2;
T = simplify(T1 + T2 + T3); % total kinetic energy

% Potential Energies
Ug1 = m1*g*pos_c1(2); Ue1 = (1/2)*k1*th1^2;
Ug2 = m2*g*pos_c2(2); Ue2 = (1/2)*k2*th2^2;
Ug3 = m3*g*pos_c3(2); Ue3 = (1/2)*k3*th3^2;
Ug = Ug1 + Ug2 + Ug3; % gravitational potential energy
Ue = Ue1 + Ue2 + Ue3; % elastic potential energy
U = Ug + Ue; % total potential energy

% Generalized forces/torques
Q_F = F2Q(F_x*ihat + F_y*jhat, pos3);
Q_T1 = M2Q(Tau1*khat - b1*dth1*khat, dth1*khat);
Q_T2 = M2Q(Tau2*khat - b2*dth2*khat, dth2*khat); 
Q_T3 = M2Q(Tau3*khat - b3*dth3*khat, dth3*khat);
Q_T = Q_T1 + Q_T2 + Q_T3;
Q = Q_T + Q_F; % total generalized forces

% Assemble the array of cartesian coordinates of the key points
% (x, y) position of end of each link
keypoints = [pos1(1:2) pos2(1:2) pos3(1:2)];

%% All the work is done!  Just turn the crank...
% Derive Energy Function and Equations of Motion
E = T + U; % Total energy
L = T - U; % Lagrangian
eom = simplify(ddt(jacobian(L, dq).') - jacobian(L, q).' - Q);

% Rearrange Equations of Motion
A = simplify(jacobian(eom, ddq)); % inertia matrix
b = simplify(A*ddq - eom); % non-acceleration terms

% Equations of motion are
% eom: (inertia matrix)*ddq + (coriolis/centripetal terms) + (damping matrix)*dq + (gravitational terms) + (stiffness matrix)*q - (applied forces/torques) = 0
% Mass_Joint_Sp = subs(A, [Ic1 + m1*r1^2, Ic2 + m2*r2^2, Ic3 + m3*r3^2], [I1, I2, I3]);
Inertia_Matrix = simplify(A);
Grav_Terms = simplify(jacobian(Ug, q)');
Stiffness_Matrix = simplify(jacobian(jacobian(Ue, q)', q)');
Damping_Matrix = simplify(jacobian(-Q, dq)');
Corr_Terms = simplify(eom + Q - Grav_Terms - Stiffness_Matrix*q - A*ddq);

% Compute endpoint jacobian
J = jacobian(pos3, q);

% Compute ddt(J)
dJ = reshape(ddt(J(:)), size(J));

% Write Energy Function and Equations of Motion
z    = [q; dq];      % state vector
pos3 = pos3(1:2);    % racket position (x, y)
v3   = v3(1:2);      % racket velocity (vx, yv)
a3   = a3(1:2);      % racket acceleration (ax, ay)
z2   = [q; dq; ddq]; % expanded state vector
J    = J(1:2, 1:3);
dJ   = dJ(1:2, 1:3);

%% Generate MATLAB functions to use these symbolic values throughout our code
matlabFunction(Inertia_Matrix, 'file', ['A_' name], 'vars', {z p});
matlabFunction(b, 'file', ['b_' name], 'vars', {z u p});
matlabFunction(E, 'file',['energy_' name], 'vars', {z p});
matlabFunction(pos3, 'file', ['position_endEffector'], 'vars', {z p});
matlabFunction(v3, 'file', ['velocity_endEffector'], 'vars', {z p});
matlabFunction(a3, 'file', ['acceleration_endEffector'], 'vars', {z2 p});
matlabFunction(J, 'file', ['jacobian_endEffector'], 'vars', {z p});
matlabFunction(dJ, 'file', ['jacobian_dot_endEffector'], 'vars', {z p});

matlabFunction(Grav_Terms, 'file', ['Grav_Terms'] , 'vars', {z p});
matlabFunction(Corr_Terms, 'file', ['Corr_Terms'] , 'vars', {z p});
matlabFunction(Stiffness_Matrix, 'file', ['Stiffness_Matrix'], 'vars', {z p});
matlabFunction(Damping_Matrix, 'file', ['Damping_Matrix'], 'vars', {z p});
matlabFunction(keypoints, 'file', ['keypoints_' name], 'vars', {z p});