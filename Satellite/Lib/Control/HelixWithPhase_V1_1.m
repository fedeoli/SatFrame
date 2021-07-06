function x_ref = HelixWithPhase_V1_1(t, params)

%   HelixWithPhase_V1_1.m
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
%       - params.Phase = phase angle of helix starting point with respect to radial axis (x-axis) [deg]
%
%   OUTPUT
%   x_ref: Array (6x1) containing the Helix trajectory position and velocity.
%
%   VERSION
%   20190404 V1_1:
%   - First release

n = params.n;
alpha = params.Phase*pi/180;
x_ref = [params.xRef*cos(n*t + alpha); -params.yRef*sin(n*t + alpha); params.zRef*cos(n*t + alpha); -params.xRef*n*sin(n*t + alpha); -params.yRef*n*cos(n*t + alpha); -params.zRef*n*sin(n*t + alpha)];


end


