function x_ref = Out_of_plane_V1_1(t, params)

%   LeaderFollower_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Computes the Leader/Follower trajectory (position and velocity) at time t.
%
%   INPUT
%   t: time instant
%   params: structure containing the following fields
%       - params.yRef = along-track distance between leader and follower
%
%   OUTPUT
%   x_ref: Array (6x1) containing the Leader/Follower trajectory position and velocity.
%
%   VERSION
%   20190326 V1_1:
%   - First release


x_ref = [0; 0; params.zRef; 0; 0; 0];


end


