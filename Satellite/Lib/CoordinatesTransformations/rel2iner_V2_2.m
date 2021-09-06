function [deputy_iner_ECI, params] = rel2iner_V2_2(deputy_rel_LVLH, chief_iner_ECI, chief_coe, params)

%   rel2iner_V2_2.m
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
%   - Adaptated to the new definition of satellites drag properties
%
%   20200523 V2_2:
%   - The chief's cross section allocation name in the 'sat' structure has been changed from 'A' to 'MeanCrossSection'

% Extracting constants from "params" structure
mi = params.mi;
Re = params.Re;
J2 = params.J2;
omega_e = params.omega_e;
BC_c = params.sat(1).MeanCrossSection*params.sat(1).CD/params.sat(1).M;

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

%%%%%%%%%%%% TRUE %%%%%%%%%%%%%%%%
deputy_relpos_ECI = chief_pos_iner + deputy_rel_ECI(1:3);                                                                   % deputy's inertial position expressed in ECI reference frame
%%%%%%%%%%% TEST %%%%%%%%%%%%%%%%
% deputy_relpos_ECI = chief_pos_iner + deputy_rel_LVLH(1:3);

p_c = a*(1 - e^2);                                                                                                          % chief's semilatum rectum
params.r_c = p_c/(1 + e*cos(f));                                                                                                   % chief's position module
params.h_c = sqrt(mi*p_c);                                                                                                         % chief's orbital angular momentum

% J2 perturbation
f2r_c = -3/2*J2*mi*Re^2/(params.r_c^4)*(1 - (3*(sin(incl))^2*(sin(th))^2));                                                        % chief's J2 radial component
f2th_c = -3/2*J2*mi*Re^2/(params.r_c^4)*(sin(incl))^2*sin(2*th);                                                                   % chief's J2 tangential component
f2h_c = -3*J2*mi*Re^2/(params.r_c^4)*sin(incl)*cos(incl)*sin(th);                                                                  % chief's J2 out-of-plane component
fj2_c = LVLH2ECI_V1_1([f2r_c, f2th_c, f2h_c], incl, raan, th);                                                              % chief's J2 perturbation

% Drag perturbation
v_rel = [chief_vel_iner(1) + chief_pos_iner(2)*omega_e, chief_vel_iner(2) - chief_pos_iner(1)*omega_e, chief_vel_iner(3)];  % chief's relative velocity wrt to Earth's rotation
h_chief = norm(chief_iner_ECI(1:3)) - Re;                                                                                   % chief's altitude
rho_chief = ExponentialAtmDensity_V1_1(h_chief);                                                                               % atmospheric density at chief's altitude
RD = -(1/2)*BC_c*rho_chief*norm(v_rel)^2;                                                                                   % drag acceleration module [s^-2]
fd_c = RD*v_rel./norm(v_rel);                                                                                               % Drag's acceleration components in ECI reference frame

% Total perturbation (sum of Drag and J2)
pert_tot_ECI = [(fj2_c(1) + fd_c(1)), (fj2_c(2) + fd_c(2)), (fj2_c(3) + fd_c(3))];                                          % sum of the perturbations acting on the chief (expressed in ECI reference frame)
pert_tot_LVLH = ECI2LVLH_V1_1(pert_tot_ECI, incl, raan, th);                                                                % sum of the perturbations acting on the chief (expressed in LVLH reference frame)
params.fh_c = pert_tot_LVLH(3);                                                                                                    % out-of-plane component of the total perturbation acting on the chief

% Deputy's relative velocity computation
chief_omega_LVLH = [params.fh_c*params.r_c/params.h_c, 0, params.h_c/params.r_c^2];                                                                            % chief's orbital angular velocity in LVLH
deputy_relvel_LVLH = deputy_rel_LVLH(4:6) + cross(chief_omega_LVLH, deputy_rel_LVLH(1:3));                                  % deputy's relative velocity (expressed in LVLH reference frame)
deputy_relvel_ECI = chief_vel_iner' + LVLH2ECI_V1_1(deputy_relvel_LVLH, incl, raan, th);                                     % deputy's relative velocity (expressed in ECI reference frame)

% define for C code
deputy_iner_ECI = zeros(6,1);

% Allocation of the output
deputy_iner_ECI(1:3,1) = deputy_relpos_ECI;
deputy_iner_ECI(4:6,1) = deputy_relvel_ECI;

end