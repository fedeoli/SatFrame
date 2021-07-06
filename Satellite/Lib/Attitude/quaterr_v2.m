function q_err = quaterr_v2(q1, q2)

%   QuaternionError_V2_1.m
%   Made by Sapienza Gn Lab
%
%   Computes the error between a pair of quaternion vectors.
%
%   INPUT
%   q1: quaternion vector
%   q2: 
%
%   OUTPUT
%   DesiredAttitude: matrix (3 x N) containing the euler angles representing the desired attitude for each deputy based on the firing direction (N is the 
%                    number of deputy satellites)
%
%   VERSION
%   20191125 V2_1:
%   -  First Release

q_err = quatmultiply(quatconj(q2),q1);
% q_err = [q_err(2:4), q_err(1)];

end