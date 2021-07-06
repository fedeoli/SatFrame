function deputy_J2invariantMOE = J2invariantCOE_V1_1(chief_MOE, deputy_dcoe, params)

%   J2invariantCOE_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Computes deputy's J2 invariant Mean Orbital Elements (MOE) from chief's MOE and deputy's differential orbital elements.
%
%   INPUT
%   chief_MOE: Array containing chief's Mean Orbital Elements (MOE). Can be either (6 x 1) or (1 x 6).
%   deputy_dcoe: Array containing deputy's differential orbital elements. The differential orbital elements must be in this order [d_a, d_e, d_incl, d_om, d_OM, d_f].
%   Note that the sixth element must be The array can be either (6 x 1) or (1 x 6).
%   params: parameter's structure containing the following fields:
%       - params.J2: Earth's J2 coefficient
%       - params.Re: Earth's mean equatorial radius
%
%   OUTPUT
%   deputy_J2invariantDCOE: Array (1 x 6) containing deputy's J2 invariant, Mean Orbital Elements (MOE).
%
%   VERSION
%   20190304 V1_1:
%   -  First Release


% Extraction of constans from "params" structure
J2 = params.J2;
Re = params.Re;

% Extraction of chief's MOE
a_chief_mean = chief_MOE(1);
e_chief_mean = chief_MOE(2);
i_chief_mean = chief_MOE(3);
om_chief_mean = chief_MOE(4);
OM_chief_mean = chief_MOE(5);
f_chief_mean = chief_MOE(6);

% Extraction of deputy's differential orbital elements
d_e = deputy_dcoe(2);
d_om = deputy_dcoe(4);
d_OM = deputy_dcoe(5);
d_f = deputy_dcoe(6);

% Four out of six deputy's MOE remain unchanged
e_deputy_mean = e_chief_mean + d_e;
om_deputy_mean = om_chief_mean + d_om;
OM_deputy_mean = OM_chief_mean + d_OM;
f_deputy = f_chief_mean + d_f;

% Compute deputy's J2 invariant inclination
eta_chief = sqrt(1 - e_chief_mean^2);
eta_deputy = sqrt(1 - e_deputy_mean^2);
d_eta = eta_deputy - eta_chief;
d_incl = -4*d_eta/(eta_chief*tan(i_chief_mean));    %J2 invariant condition on inclination
i_deputy_mean = i_chief_mean + d_incl;

% Compute deputy's J2 invariant semi-major axis
L = sqrt(a_chief_mean/Re);
D = J2/(4*L^4*eta_chief^5)*(4 + 3*eta_chief)*(1 +5*cos(i_chief_mean)^2);
d_L = D*L*d_eta;
L = L + d_L;
a_deputy_mean = L^2*Re;

% Allocation of deputy's J2 invariant MOE
deputy_J2invariantMOE = [a_deputy_mean, e_deputy_mean, i_deputy_mean, om_deputy_mean, OM_deputy_mean, f_deputy];

end