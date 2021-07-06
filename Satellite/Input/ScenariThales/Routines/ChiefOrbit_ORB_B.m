%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   CHIEF ORBIT B
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Type: Multi-SSO, low altitude, prograde
%
% Repeat Cycle: 5 nodal days = 4 days 22h 0m 11.11sec
% Cycle length: 14*5 + 4 = 74 orbits
% a = 6935.136 [km]
% i	= 46.60684 [deg]
% e	= 0.000783 
% Arg of perigee = 90 [deg]
% RAAN = 0 [deg]



% Chief's Mean Orbital Elements (MOE)
a = 6935.136;                                                   % chief's orbit's mean semimajor axis [km]
ecc = 0.000783;                                                 % chief's orbit's mean eccentricity
i = 46.60684*pi/180;                                            % chief's orbit's mean inclination [rad]
om = 90*pi/180;                                                 % chief's orbit's mean argument of the perigee [rad]
RAAN = 0*pi/180;                                                % chief's orbit's mean right ascension of the ascending node [rad]
f_0 = 0*pi/180;                                                 % chief's mean true anomaly at time 0 [rad]
n = sqrt(mi/(a^3));                                             % chief's orbit's mean motion [rad/s]
T = 2*pi/n;


chief_MOE = [a, ecc, i, om, RAAN, f_0];                                 % array with chief's mean orbital elements [A, E, I, ARG, RAAN, TH]
params.n = n;

% Transformation of chief's Mean Orbital Elements (MOE) to Osculating Orbital Elements (OOE)

chief_OOE = moe2ooe_V1_1(chief_MOE, params);
params.chief_OOE = chief_OOE;

% Conversion from chief's Osculating Orbital Elements to inertial coordinates expressed in ECI reference frame and storing into satellites_iner_ECI
satellites_iner_ECI = coe2rv_V1_1(chief_OOE(1:6), mi);