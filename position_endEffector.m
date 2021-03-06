function pos3 = position_endEffector(in1,in2)
%POSITION_ENDEFFECTOR
%    POS3 = POSITION_ENDEFFECTOR(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    10-May-2021 20:09:49

l1 = in2(7,:);
l2 = in2(8,:);
l3 = in2(9,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
t2 = th1+th2;
t3 = t2+th3;
pos3 = [l2.*cos(t2)+l3.*cos(t3)+l1.*cos(th1);l2.*sin(t2)+l3.*sin(t3)+l1.*sin(th1)];
