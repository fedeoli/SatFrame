function [RAAN_drift, inPlane_drift] = J2drift_V1_1(satellite_MOE, params)

%   J2drift_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Computes both the out-of-plane and in-plane drifts caused by Earth's J2 perturbation on a satellite whose Mean Orbital Elements (MOE) are known.
%
%   INPUT
%   satellite_MOE: Array containing satellite's Mean Orbital Elements (MOE). Can be either (6 x 1) or (1 x 6).
%   params: parameter's structure containing the following fields:
%       - params.J2: Earth's J2 coefficient
%       - params.Re: Earth's mean equatorial radius
%       - params.mi: Earth's gravitational constant
%
%   OUTPUT
%   RAAN_drift: Out-of-plane drift
%   inplane_drift: In-plane drift
%
%   VERSION
%   20190304 V1_1:
%   -  First Release


% Extraction of constants from "params" structure
J2 = params.J2;
Re = params.Re;
mi = params.mi;

% Extraction of satellite's MOE
a_mean = satellite_MOE(1);
e_mean = satellite_MOE(2);
i_mean = satellite_MOE(3);

% Satellite's constant calculation
n_mean = sqrt(mi/a_mean^3);
p_mean = a_mean*(1 - e_mean^2);

% Drifts' calculation
RAAN_drift = -3/2*J2*n_mean*(Re/p_mean)^2*cos(i_mean);                                      % Out-of-plane drift
om_drift = 3/4*J2*n_mean*(Re/p_mean)^2*(5*cos(i_mean)^2 - 1);
M_drift = n_mean + 3/4*J2*n_mean*(Re/p_mean)^2*sqrt(1 - e_mean^2)*(3*cos(i_mean)^2 - 1);
inPlane_drift = om_drift + M_drift;                                                         % In-plane drift


end