function deputy_iner_ECI = shape2iner_V1_1(deputy_shape, chief_iner_ECI, chief_coe, params)

%   shape2iner_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Transforms a single deputy satellite's coordinates from shape of the closed trajectory (rho1, rho2, rho3, alpha0, beta0) to inertial coordinates
%   
%   INPUT
%   deputy_shape: array containing deputy's shape parameters (rho1, rho2, rho3, alpha0, beta0). Can be either (1 x 5) or (5 x 1).
%   chief_iner_ECI: array (6 x 1) containing chief's inertial position and velocity.
%   chief_coe: array containing chief's orbital elements. Can be either (1 x 6) or (6 x 1).
%   params: structure containing the following fields
%       .mi = Earth's planetary constant
%
%   OUTPUT
%   deputy_iner_ECI: array (6 x 1) containing deputy's inertial position and velocity


% Extraction of contants from "params" structure
mi = params.mi;

% Extraction of chief's orbital elements
a = chief_coe(1);
e = chief_coe(2);
inc = chief_coe(3);
raan = chief_coe(5);
om = chief_coe(4);
f0 = chief_coe(6);

% Chief's orbital constants
p_c = a*(1 - e^2);
r_c = p_c/(1 + e*cos(f0));
h_c = sqrt(mi*p_c);
f0_dot = h_c/r_c^2;

% Extraction of deputy's shape parameters
rho1 = deputy_shape(1);
rho2 = deputy_shape(2);
rho3 = deputy_shape(3);
alpha0 = deputy_shape(4);
beta0 = deputy_shape(5);

% Computation of deputy's relative orbital position and velocity
x0 = rho1*sin(f0 + alpha0);                                                                                                 % x component of relative orbital position
y0 = 2*rho1*cos(f0 + alpha0)*((1 + (e/2)*cos(f0))/(1 + e*cos(f0))) + rho2/(1 + e*cos(f0));                                  % y component of relative orbital position
z0 = rho3*sin(f0 +  beta0)/(1 + e*cos(f0));                                                                                 % z component of relative orbital position
chief_pos_ECI(1:3) = chief_iner_ECI(1:3);                                                                                   % chief's inertial position vector expressed in ECI reference frame
deputy_relpos_ECI(1:3) = LVLH2ECI_V1_1([x0, y0, z0], inc, raan, om + f0);                                                   % deputy's relative position expressed in ECI reference frame
deputy_inerpos_ECI = chief_pos_ECI + deputy_relpos_ECI(1:3);                                                                % deputy's inertial position expressed in ECI reference frame
x0_dot = rho1*cos(f0 + alpha0)*f0_dot;                                                                                      % x component of deputy's relative orbital velocity
z0_dot = rho3*cos(f0 + beta0)*f0_dot/(1 + e*cos(f0)) - rho3*sin(f0 +  beta0)*(-e*sin(f0)*f0_dot)/(1 + e*cos(f0)^2);         % z component of deputy's relative orbital velocity
chief_vr = sqrt(mi/p_c)*e*sin(f0);                                                                                          % chief's radial velocity
vx = chief_vr + x0_dot - f0_dot*y0;                                                                                         % x component of deputy's inertial velocity
vz = z0_dot;                                                                                                                % z component of deputy's inertial velocity
y0_dot = sqrt(2*(-mi/(2*a) + mi/norm(deputy_inerpos_ECI)) - vx^2 - vz^2) - (f0_dot*r_c + f0_dot*x0);                        % Second component of the relative velocity computed with the Energy Matching approach

% From relative to inertial
deputy_rel_LVLH = [x0, y0, z0, x0_dot, y0_dot, z0_dot];                                                                     % deputy's relative coordinates expressed in LVLH reference frame
deputy_iner_ECI = rel2iner_V1_1(deputy_rel_LVLH, chief_iner_ECI, chief_coe, params);                                        % deputy's inertial coordinates expressed in ECI reference frame

end





