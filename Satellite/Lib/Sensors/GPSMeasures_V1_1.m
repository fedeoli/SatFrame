function deputy_rel_measured = GPSMeasures_V1_1(deputy_rel)

%   GPSMeasures_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Takes as input the relative position and velocity of N satellites and computes their relative state as measured by GPS. The noise added to the
%   state is computed as Gaussian noise, multiplied by the covariances provided as a reference in 11/03/2019 e-mail by Luca Nardecchia.
%
%   INPUT
%   deputy_rel: Array (6 x N), with N equal to the number of deputy satellites, containing each deputy's position and velocity relative components,
%               expressed in LVLH reference frame.
%
%   OUTPUT
%   deputy_rel_measured: Array (6 x N), with N equal to the number of deputy satellites, containing each deputy's position and velocity relative components,
%                        expressed in LVLH reference frame, as measured by GPS.
%
%   VERSION
%   20190312 V1_1:
%   -  First Release


% Number of deputy satellites
N = size(deputy_rel, 2);

% GPD error 1-sigma covariance (ref. mail 20190311 from Nardecchia Luca)
cov = [2; 2; 2; 0.1; 0.1; 0.1]*1e-3;

% Repeat the covariance for the number of deputy
covN = repmat(cov, [1,N]);

% Compute the Gaussian noise based on the prescribed covariances
noise = covN.*randn(6,N);

% Add the noise to the true state
deputy_rel_measured = deputy_rel + noise;

end