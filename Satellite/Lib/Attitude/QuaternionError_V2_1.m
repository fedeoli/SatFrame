function q_err = QuaternionError_V2_1(q1, q2)

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


q_ref_rior = [q2(2:4), q2(1)];
q_rior = [q1(2:4), q1(1)];

q_err_rior = [q_ref_rior(4) q_ref_rior(3) -q_ref_rior(2) -q_ref_rior(1);
              -q_ref_rior(3) q_ref_rior(4) q_ref_rior(1) -q_ref_rior(2);
              q_ref_rior(2) -q_ref_rior(1) q_ref_rior(4) -q_ref_rior(3);
              q_ref_rior(1) q_ref_rior(2) q_ref_rior(3) q_ref_rior(4)]*q_rior';

q_err = [q_err_rior(4), q_err_rior(1:3)'];

end