function Damping_Matrix = Damping_Matrix(in1,in2)
%DAMPING_MATRIX
%    DAMPING_MATRIX = DAMPING_MATRIX(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    10-May-2021 20:09:50

b1 = in2(17,:);
b2 = in2(18,:);
b3 = in2(19,:);
Damping_Matrix = reshape([b1,0.0,0.0,0.0,b2,0.0,0.0,0.0,b3],[3,3]);
