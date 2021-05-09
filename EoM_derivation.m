syms r1 r2 r3 l1 l2 l3 m1 m2 m3 I1 I2 I3 Ic1 Ic2 Ic3 g real % parameters
syms th1 th2 th3 om1 om2 om3 real % generalized coordinates and their derivatives

%% Kinematic Equations
xc1 = r1*cos(th1);
yc1 = r1*sin(th1);

xc2 = l1*cos(th1) + r2*cos(th1 + th2);
yc2 = l1*sin(th1) + r2*sin(th1 + th2);

xc3 = l1*cos(th1) + l2*cos(th1 + th2) + r3*cos(th1 + th2 + th3);
yc3 = l1*sin(th1) + l2*sin(th1 + th2) + r3*sin(th1 + th2 + th3);

xe = l1*cos(th1) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3);
ye = l1*sin(th1) + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3);

% Jacobians
J1 = jacobian([xc1; yc1], [th1]);
J2 = jacobian([xc2; yc2], [th1 th2]);
J3 = jacobian([xc3; yc3], [th1 th2 th3]);
Je = jacobian([xe; ye], [th1 th2 th3]);

% Velocities
vc1 = J1*om1;
vc2 = J2*[om1; om2];
vc3 = J3*[om1; om2; om3];

% Kinetic Energies
T1 = simplify(1/2*m1*(vc1'*vc1) + 1/2*Ic1*om1^2);
T2 = simplify(1/2*m2*(vc2'*vc2) + 1/2*Ic2*(om1 + om2)^2);
T3 = simplify(1/2*m3*(vc3'*vc3) + 1/2*Ic3*(om1 + om2 + om3)^2);
T = T1 + T2 + T3;

% Potential Energies
V1 = simplify(m1*g*yc1 + 1/2*k1*th1^2);
V2 = simplify(m2*g*yc2 + 1/2*k2*th2^2);
V3 = simplify(m3*g*yc3 + 1/2*k3*th3^2);
V = V1 + V2 + V3;

% Lagrangian
L = simplify(T - V);
pretty(collect(L, [om1^2 om2^2 om3^2 om1*om2 om1*om3 om2*om3 th1^2 th2^2 th3^2 g]))