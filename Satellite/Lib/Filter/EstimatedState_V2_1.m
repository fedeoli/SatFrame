function [deputy_rel_measured, P] = EstimatedState_V2_1(deputy_rel)

%   EstimatedState_V2_1.m
%   Made by Sapienza Gn Lab
%
%   Takes as input the relative state of N satellites and computes the relative state and covariance as estimated by the TV observer in the worst case scenario.
%
%   INPUT
%   deputy_rel: Array (6 x N), with N equal to the number of deputy satellites, containing each deputy's position and velocity relative components,
%               expressed in LVLH reference frame.
%
%   OUTPUT
%   deputy_rel_measured: Array (6 x N), with N equal to the number of deputy satellites, containing each deputy's position and velocity relative components,
%                        expressed in LVLH reference frame, as measured by GPS.
%   P: Covariance matrix (6*N x 6*N) associated to the estimated state vector.
%
%   VERSION
%   20200908 V2_1:
%   -  First Release


% Number of deputy satellites
N = size(deputy_rel, 2);

% State error 1-sigma covariance (ref. TV observer worst case scenario)
std = [1; 1; 1; 0.001; 0.001; 0.001]*1e-3;

% Repeat the covariance for the number of deputy
covN = repmat(std, [1,N]);

% Compute the Gaussian noise based on the prescribed covariances
noise = covN.*randn(6,N);

% Add the noise to the true state
deputy_rel_measured = deputy_rel + noise;

% Covariance Matrix
P = diag(repmat(std.^2, [N,1]));

end