function [inplane_DeltaV, outofplane_DeltaV] = DeltaVPerOrbit_V1_1(satellites_iner, params)

%   DeltaVPerOrbit_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Computes the out-of-plane and in-plane Delta V needed to compensate for in-plane drift caused by J2 perturbation.
%
%   INPUT
%   satellites_iner: Array (6*N x 1) containing satellite's inertial position and velocity. N is the number of satellites, whcih must be at least 2
%                    (the chief and one deputy). 
%   params: parameter's structure containing the following fields:
%       - params.mi: Earth's gravitational constant
%
%   OUTPUT
%   outofplane_DeltaV: Out-of-plane Delta V
%   inplane_DeltaV: In-plane Delta V
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

% Compute out-of-plane Delta V
outofplane_DeltaV = OutOfPlaneDeltaV_V1_1(chief_MOE, deputy_MOE, params);

% Compute in-plane Delta V
inplane_DeltaV = InPlaneDeltaV_V1_1(chief_MOE, deputy_MOE, params);

end

