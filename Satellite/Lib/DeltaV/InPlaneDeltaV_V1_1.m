function inplane_DeltaV = InPlaneDeltaV_V1_1(chief_MOE, deputy_MOE, params)

%   InPlaneDeltaV_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Computes the in-plane Delta V needed to compensate for in-plane drift caused by J2 perturbation.
%
%   INPUT
%   chief_MOE: Array containing chief's Mean Orbital Elements (MOE). Can be either (6 x 1) or (1 x 6).
%   deputy_MOE: Array containing deputy's Mean Orbital Elements (MOE). Can be either (6 x 1) or (1 x 6).
%   params: parameter's structure containing the following fields:
%       - params.J2: Earth's J2 coefficient
%       - params.Re: Earth's mean equatorial radius
%       - params.mi: Earth's gravitational constant
%
%   OUTPUT
%   inplane_DeltaV: In-plane Delta V
%
%   VERSION
%   20190304 V1_1:
%   -  First Release


% Extraction of constants from "params" structure
J2 = params.J2;
Re = params.Re;
mi = params.mi;

% Extraction of chief's MOE
a_c = chief_MOE(1);
e_c = chief_MOE(2); 
incl_c = chief_MOE(3);

% Chief's constant calculation
n_c = sqrt(mi/a_c^3);
p_c = a_c*(1 - e_c^2);

% Chief's drifts
om_drift_chief = 3/4*J2*n_c*(Re/p_c)^2*(5*cos(incl_c)^2 - 1);
M_drift_chief = n_c + 3/4*J2*n_c*(Re/p_c)^2*sqrt(1 - e_c^2)*(3*cos(incl_c)^2 - 1);

% Extraction of deputy's MOE
a_d = deputy_MOE(1);
e_d = deputy_MOE(2); 
incl_d  = deputy_MOE(3);

% Deputy's constant calculation
n_d = sqrt(mi/a_d^3);
p_d = a_d*(1 - e_d^2);

% Deputy's drifts
om_drift_deputy = 3/4*J2*n_d*(Re/p_d)^2*(5*cos(incl_d)^2 - 1);
M_drift_deputy = n_d + 3/4*J2*n_d*(Re/p_d)^2*sqrt(1 - e_d^2)*(3*cos(incl_d)^2 - 1);

% In plane differential drift
dtheta_dot = (M_drift_deputy + om_drift_deputy) - (om_drift_chief + M_drift_chief);

% In plane Delta V
inplane_DeltaV = a_c/3*sqrt((1 - e_c)/(1 + e_c))*dtheta_dot;

end