function [outofplane_DV] = OutOfPlaneDeltaV_V1_1(chief_MOE, deputy_MOE, params)

%   OutOfPlaneDeltaV_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Computes the out-of-plane Delta V needed to compensate for out-of-plane drift caused by J2 perturbation.
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
%   outofplane_DV: Out-of-plane Delta V
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
f_c =  chief_MOE(6);

% Chief's constant calculation
n_c = sqrt(mi/a_c^3);
p_c = a_c*(1 - e_c^2);
eta_c = sqrt(1-e_c^2);
r_c = a_c*eta_c^2/(1+e_c*cos(f_c));
h_c = sqrt(mi*p_c);

% Chief's out-of-plane drift
outofplaneDrift_chief = -3/2*J2*n_c*(Re/p_c)^2*cos(incl_c);

% Extraction of deputy's MOE
a_d = deputy_MOE(1);
e_d = deputy_MOE(2); 
incl_d  = deputy_MOE(3);
n_d = sqrt(mi/a_d^3);
p_d = a_d*(1 - e_d^2);

% Deputy's out-of-plane drift
outofplaneDrift_deputy = -3/2*J2*n_d*(Re/p_d)^2*cos(incl_d);

% Out-of-plane Delta V
outofplane_DV = h_c/r_c*(outofplaneDrift_deputy - outofplaneDrift_chief)*sin(incl_c)*2*pi/n_c;

end