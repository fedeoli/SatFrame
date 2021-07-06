function [differential_inplaneDrift, differential_outofplaneDrift] = DifferentialDriftPerOrbit_V1_1(satellites_iner, params)

%   DifferentialDriftPerOrbit_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Computes the differential drift between chief and deputy caused by J2 perturbation.
%
%   INPUT
%   satellites_iner: Array (6*N x 1) containing satellite's inertial position and velocity. N is the number of satellites, whcih must be at least 2
%                    (the chief and one deputy). 
%   params: parameter's structure containing the following fields:
%       - params.mi: Earth's gravitational constant
%
%   OUTPUT
%   differential_inplaneDrift: Out-of-plane differential drift per orbit
%   differential_outofplaneDrift: In-plane differential drift per orbit
%
%   VERSION
%   20190304 V1_1:
%   -  First Release


% Extraction of constants from "params" structure
mi = params.mi;

% Conversion from inertial position and velocity to Osculating Orbital Parameters (OOE)
chief_OOE = rv2coe_V1_1(satellites_iner(1:3), satellites_iner(4:6), mi);
deputy_OOE = rv2coe_V1_1(satellites_iner(7:9), satellites_iner(10:12), mi);

% Conversion from OOE to MOE
chief_MOE  = ooe2moe_V1_1(chief_OOE, params);
deputy_MOE  = ooe2moe_V1_1(deputy_OOE, params);

% Compute chief and deputy's drifts
[chief_drift_outofplane, chief_drift_inplane] = J2drift_V1_1(chief_MOE, params);
[deputy_drift_outofplane, deputy_drift_inplane] = J2drift_V1_1(deputy_MOE, params);
n = sqrt(mi/chief_MOE(1)^3);

% Differential drift per orbit
differential_inplaneDrift = (deputy_drift_inplane - chief_drift_inplane + (deputy_drift_outofplane - chief_drift_outofplane)*cos(chief_MOE(3)))*chief_MOE(1)*2*pi/n;
differential_outofplaneDrift = (deputy_drift_outofplane - chief_drift_outofplane)*sin(chief_MOE(3))*chief_MOE(1)*2*pi/n;


end