function deputy_rel_LVLH = iner2rel_V1_1(chief_iner_ECI, deputy_iner_ECI)

%   iner2rel_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Transforms satellites inertial coordinates (expressed in ECI reference frame) into relative orbital coordinates (expressed in LVLH reference
%   frame), relatively to a chief satellite, whose position and velocity must be the first six components of the input "satellites_iner"
%
%   INPUT
%   chief_iner_ECI: Array (6 x 1) containing chief's inertial coordinates expressed in ECI reference frame
%   deputy_iner_ECI: Array (6 x 1) containing deputy's inertial coordinates expressed in ECI reference frame
%
%   OUTPUT
%   deputy_rel_LVLH: Array (6 x 1) containing deputy's relative coordinates exoressed in LVLH reference frame
%
%   VERSION
%   20190301 V1_1:
%   -  First Release


global fh_c r_c h_c

% Extraction of chief's and deputy's position and velocity vectors
chief_pos(1:3) = chief_iner_ECI(1:3);                                                           
chief_vel(1:3) = chief_iner_ECI(4:6);
deputy_pos(1:3) = deputy_iner_ECI(1:3);
deputy_vel(1:3) = deputy_iner_ECI(4:6);

% Chief's classical orbital elements
coe(1:6) = rv2coe_V1_1(chief_pos, chief_vel);

% Deputy's relative position expressed in LVLH reference frame
deputy_rel_LVLH(1:3) = ECI2LVLH_V1_1(deputy_pos-chief_pos, coe(3), coe(5), coe(4)+coe(6));

% Chief's angular velocity in LVLH reference frame
chief_omega_LVLH(1) = fh_c*r_c/h_c;
chief_omega_LVLH(2) = 0;
chief_omega_LVLH(3) = h_c/r_c^2;

% Deputy's inertial velocity expressed in LVLH reference frame
deputy_vel_LVLH = ECI2LVLH_V1_1((deputy_vel-chief_vel), coe(3), coe(5), coe(4)+coe(6));

% Deputy's relative velocity expressed in LVLH reference frame
deputy_rel_LVLH(4:6) = deputy_vel_LVLH' - cross(chief_omega_LVLH, deputy_rel_LVLH(1:3));

end