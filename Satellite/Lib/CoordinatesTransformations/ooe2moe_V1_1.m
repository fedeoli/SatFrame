function moe = ooe2moe_V1_1(ooe, params)

%   ooe2moe_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Transforms satellite's Mean Orbital Elements (MOE) into Osculating Orbital Elements (OOE).
%
%   INPUT
%   ooe: Array containing satellite's Osculating Orbital Elements (OOE)
%   params: parameter's structure containing the following fields:
%       - params.J2: Earth's J2 coefficient
%       - params.Re: Earth's mean equatorial radius
%
%   OUTPUT
%   moe: Array (1 x 6) containing satellite's Mean Orbital Elements (MOE).
%
%   VERSION
%   20190301 V1_1:
%   -  First Release


% Extraction of constants from params structure
J2 = params.J2;
Re = params.Re;

% Extraction of OOE
a = ooe(1);
e = ooe(2); 
inc = ooe(3);
om =  ooe(4);
OM =  ooe(5);
an =  ooe(6);

E = 2*atan(sqrt((1 - e)/(1 + e))*tan(an/2));    % eccentric anomaly
M = E - e*sin(E);                               % mean anomaly
eta = sqrt(1-e^2);
r = a*eta^2/(1+e*cos(an));                      % satellite's radius


%%% The plus/minus sign in "gamma_2" expression is the only difference in mapping from mean to osc or viceversa %%%
gamma_2 = -J2/2*(Re/a)^2;
gamma_2p = gamma_2/eta^4;

a_trasf = a + a*gamma_2*((3*cos(inc)^2-1)*((a/r)^3 - 1/eta^3) + 3*(1-cos(inc)^2)*(a/r)^3*cos(2*om + 2*an));

de1 = gamma_2p/8*e*eta^2*(1 - 11*cos(inc)^2 - 40*cos(inc)^4/(1 - 5*cos(inc)^2))*cos(2*om);

de  = de1 + eta^2/2*(gamma_2*((3*cos(inc)^2 - 1)/eta^6*(e*eta + e/(1+eta) + 3*cos(an)...
    + 3*e*cos(an)^2 + e^2*cos(an)^3)  + 3*(1 - cos(inc)^2)/eta^6*(e...
    + 3*cos(an) + 3*e*cos(an)^2 + e^2*cos(an)^3)*cos(2*om + 2*an))...
    - gamma_2p*(1-cos(inc)^2)*(3*cos(2*om + an) + cos(2*om + 3*an)));

d_i = -e*de1/(eta^2*tan(inc)) + gamma_2p/2*cos(inc)*sqrt(1-cos(inc)^2)*(3*cos(2*om + 2*an) ...
    +3*e*cos(2*om + an) + e*cos(2*om + 3*an));

sum_ang = M + om + OM + gamma_2p/8*eta^3*(1 - 11*cos(inc)^2 - 40*cos(inc)^4/(1 - 5*cos(inc)^2))...
    -gamma_2p/16*(2 + e^2 - 11*(2 + 3*e^2)*cos(inc)^2 ...
-40*(2 + 5*e^2)*cos(inc)^4/(1 - 5*cos(inc)^2) - 400*e^2*cos(inc)^6/(1 - 5*cos(inc)^2)^2)...
    +gamma_2p/4*(-6*(1 - 5*cos(inc)^2)*(an - M + e*sin(an))...
    + (3 - 5*cos(inc)^2)*(3*sin(2*om + 2*an) + 3*e*sin(2*om +an)...
    +e*sin(2*om + 3*an)))...
    -gamma_2p/8*e^2*cos(inc)*(11 + 80*cos(inc)^2/(1 - 5*cos(inc)^2) + 200*cos(inc)^4/(1 - 5*cos(inc)^2)^2)...
    -gamma_2p/2*cos(inc)*(6*(an - M + e*sin(an))...
    -3*sin(2*om + 2*an) - 3*e*sin(2*om +an) - e*sin(2*om + 3*an));

edM = gamma_2p/8*e*eta^3*(1 - 11*cos(inc)^2 - 40*cos(inc)^4/(1 - 5*cos(inc)^2))...
    - gamma_2p/4*eta^3*(2*(3*cos(inc)^2 - 1)*((a*eta/r)^2 + a/r +1)*sin(an)...
    +3*(1 - cos(inc)^2)*((-(a*eta/r)^2 -a/r +1)*sin(2*om+an)...
    +((a*eta/r)^2 + a/r + 1/3)*sin(2*om + 3*an)));

d_OM = -gamma_2p/8*e^2*cos(inc)*(11 + 80*cos(inc)^2/(1 - 5*cos(inc)^2) + 200* cos(inc)^4/(1 - 5*cos(inc)^2)^2) ...
    -gamma_2p/2*cos(inc)*(6*(an - M + e*sin(an)) - 3*sin(2*om + 2*an) ...
-3*e*sin(2*om + an) - e*sin(2*om + 3*an));

d1 = (e + de)*sin(M) + (edM)*cos(M);
d2 = (e + de)*cos(M) - (edM)*sin(M);
e_trasf = sqrt(d1^2 + d2^2);

M_trasf = atan(d1/d2);


d3 = (sin(inc/2) + cos(inc/2)*d_i/2)*sin(OM) + sin(inc/2)*d_OM*cos(OM);
d4 = (sin(inc/2) + cos(inc/2)*d_i/2)*cos(OM) - sin(inc/2)*d_OM*sin(OM);

OM_trasf = atan(d3/d4);
om_trasf = sum_ang - M_trasf - OM_trasf;
i_trasf = 2*asin(sqrt(d3^2 + d4^2));

E_trasf = EccentricAnomaly_V1_1(M_trasf,e_trasf,0);
an_trasf = 2*atan(sqrt((1 + e_trasf)/(1 - e_trasf))*tan(E_trasf/2));
moe = [a_trasf, e_trasf, i_trasf, om_trasf, OM_trasf, an_trasf];

end
