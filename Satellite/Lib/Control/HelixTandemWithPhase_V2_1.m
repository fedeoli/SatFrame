function x_ref = HelixTandemWithPhase_V2_1(t, params)

%   HelixTandemWithPhase_V2_1.m
%   Made by Sapienza Gn Lab
%
%   Computes the Helix trajectory (position and velocity) at time t. The out-of-plane position is set as in the Tandem formation, where z = 0 @t = 0.
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
%   20200902 V2_1:
%   - First release

n = params.n;
alpha = params.Phase*pi/180;
x_ref = [params.xRef*cos(n*t + alpha); -params.yRef*sin(n*t + alpha); params.zRef*sin(n*t + alpha); -params.xRef*n*sin(n*t + alpha); -params.yRef*n*cos(n*t + alpha); params.zRef*n*cos(n*t + alpha)];


end


