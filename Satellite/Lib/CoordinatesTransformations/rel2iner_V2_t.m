function deputy_iner_ECI = rel2iner_V2_t(deputy_rel_LVLH, chief_iner_ECI, chief_coe, params)

%   rel2iner_V2_t.m
%   Made by Sapienza Gn Lab
%
%   Transforms a single deputy satellite's relative coordinates into inertial coordinates.
%
%   INPUT
%   deputy_rel_LVLH: array (1 x 6) containing deputy's relative position and velocity expressed in LVLH.
%   chief_iner_ECI: array containing chief's inertial position and velocity. Can be either (1 x 6) or (6 x 1).
%   chief_coe: array containing chief's orbital elements. Can be either (1 x 6) or (6 x 1).
%   params: structure containing the following fields:
%       - params.mi: Earth's planetary constant [km^3/s^2]
%       - params.Re: Earth's equatorial mean radius [km]
%       - params.J2: Earth's J2 coefficient
%       - params.omega_e: Earth's rotational velocity [rad/s]
%       - params.CDAF: satellites' balistic coefficients
%
%   OUTPUT
%   deputy_iner_ECI = array (6 x 1) containing deputy's inertial position and velocity.
%
%   20200109 V2_1:
%   - Adaptated to the new definition of satellites drag properties (used only at t = 0)
%
%   20200109 V2_t:
%   - Used for t > 0 in SingleAxisThruster


global fh_c r_c h_c

% Extracting constants from "params" structure
mi = params.mi;

% Chief's orbital elements
a = chief_coe(1);
e = chief_coe(2);
incl = chief_coe(3);
argp = chief_coe(4);
raan = chief_coe(5);
f = chief_coe(6);
th = argp + f;

% The orbital reference frame is the chief orbit's osculating reference frame, which also takes into account perturbations
chief_pos_iner(1:3) = chief_iner_ECI(1:3);                                                                                  % chief's inertial position
chief_vel_iner(1:3) = chief_iner_ECI(4:6);                                                                                  % chief's inertial velocity
deputy_rel_ECI(1:3) = LVLH2ECI_V1_1(deputy_rel_LVLH(1:3), incl, raan, th);                                                  % deputy's relative coordinates expressed in ECI reference frame
deputy_relpos_ECI = chief_pos_iner + deputy_rel_ECI(1:3);                                                                   % deputy's inertial position expressed in ECI reference frame

p_c = a*(1 - e^2);                                                                                                          % chief's semilatum rectum
r_c = p_c/(1 + e*cos(f));                                                                                                   % chief's position module
h_c = sqrt(mi*p_c);                                                                                                         % chief's orbital angular momentum

% Deputy's relative velocity computation
chief_omega_LVLH = [fh_c*r_c/h_c, 0, h_c/r_c^2];                                                                            % chief's orbital angular velocity in LVLH
deputy_relvel_LVLH = deputy_rel_LVLH(4:6) + cross(chief_omega_LVLH, deputy_rel_LVLH(1:3));                                  % deputy's relative velocity (expressed in LVLH reference frame)
deputy_relvel_ECI = chief_vel_iner' + LVLH2ECI_V1_1(deputy_relvel_LVLH, incl, raan, th);                                     % deputy's relative velocity (expressed in ECI reference frame)

% Allocation of the output
deputy_iner_ECI(1:3,1) = deputy_relpos_ECI;
deputy_iner_ECI(4:6,1) = deputy_relvel_ECI;

end