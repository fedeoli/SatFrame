%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   CHIEF Terra-SAR-X 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % TERRASAR-X
% % 1 31698U 07026A   20245.70697917  .00000423  00000-0  23320-4 0  9994
% % 2 31698  97.4461 251.6490 0001668  88.1771  13.3778 15.19145970732878
% 
% e_terrasar = 0.0001668;
% incl_terrasar = 97.4461*pi/180;
% raan_terrasar = 251.6490*pi/180;
% argp_terrasar = 88.1771*pi/180;
% f0_terrasar = 2*pi/180;
% T_terrasar = 86400/15.19145970732878;
% a_terrasar = (mi*T_terrasar^2/(4*pi^2))^(1/3);

% Chief's Mean Orbital Elements (MOE)
ecc = 0.0001668;                                                        % chief's orbit's mean eccentricity
i = 97.4461*pi/180;                                                     % chief's orbit's mean inclination [rad]
om = 88.1771*pi/180;                                                    % chief's orbit's mean argument of the perigee [rad]
RAAN = 251.6490*pi/180;                                                 % chief's orbit's mean right ascension of the ascending node [rad]
f_0 = 0*pi/180;                                                         % chief's mean true anomaly at time 0 [rad]
T = 86400/15.19145970732878;
n = 2*pi/T;                                                             % chief's orbit's mean motion [rad/s]
a = (mi/(n^2))^(1/3);                                                   % chief's orbit's mean semimajor axis [km]

chief_MOE = [a, ecc, i, om, RAAN, f_0];                                 % array with chief's mean orbital elements [A, E, I, ARG, RAAN, TH]
params.n = n;

% Transformation of chief's Mean Orbital Elements (MOE) to Osculating Orbital Elements (OOE)

chief_OOE = moe2ooe_V1_1(chief_MOE, params);
params.chief_OOE = chief_OOE;

% Conversion from chief's Osculating Orbital Elements to inertial coordinates expressed in ECI reference frame and storing into satellites_iner_ECI
satellites_iner_ECI = coe2rv_V1_1(chief_OOE(1:6), mi);