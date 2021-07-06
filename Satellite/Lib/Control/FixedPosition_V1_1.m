function x_ref = FixedPosition_V1_1(t, params)

%   FixedPosition_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Computes the wanted Fixed Position (position and velocity) at time t.
%
%   INPUT
%   t: time instant
%   params: structure containing the following fields
%       - params.xRef = radial (x) position of the deputy satellite
%       - params.yRef = along-track (y) position of the deputy satellite
%       - params.zRef = cross-track (z) position of the deputy satellite
%
%   OUTPUT
%   x_ref: Array (6x1) containing the Fixed position and velocity.
%
%   VERSION
%   20190326 V1_1:
%   - First release


x_ref = [params.xRef; params.yRef; params.zRef; 0; 0; 0];


end


