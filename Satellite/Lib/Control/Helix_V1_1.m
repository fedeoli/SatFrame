function x_ref = Helix_V1_1(t, params)

%   Helix_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Computes the Helix trajectory (position and velocity) at time t.
%
%   INPUT
%   t: time instant
%   params: structure containing the following fields
%       - params.n = chief's mean motion
%       - params.xRef = radial (x) amplitude of the Helix orbit
%       - params.yRef = along-track (y) amplitude of the Helix orbit
%       - params.zRef = cross-track (z) amplitude of the Helix orbit
%
%   OUTPUT
%   x_ref: Array (6x1) containing the Helix trajectory position and velocity.
%
%   VERSION
%   20190326 V1_1:
%   - First release

n = params.n;
x_ref = [params.xRef*cos(n*t); -params.yRef*sin(n*t); params.zRef*cos(n*t); -params.xRef*n*sin(n*t); -params.yRef*n*cos(n*t); -params.zRef*n*sin(n*t)];


end


