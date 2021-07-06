function x_ref = Pendulum_V1_1(t, params)

%   Pendulum_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Computes the Pendulum trajectory (position and velocity) at time t.
%
%   INPUT
%   t: time instant
%   params: structure containing the following fields
%       - params.n = chief's mean motion
%       - params.yRef = along-track (y) distance between chief and deputy
%       - params.zRef = cross-track (z) amplitude of the pendulum motion
%
%   OUTPUT
%   x_ref: Array (6x1) containing the Pendulum trajectory position and velocity.
%
%   VERSION
%   20190326 V1_1:
%   - First release

n = params.n;
x_ref = [0; params.yRef; params.zRef*cos(n*t); 0; 0; -params.zRef*n*sin(n*t)];


end


