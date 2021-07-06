function deputy_rel_LVLH = AbsECI2RelHill_V1_1(satellites_iner_ECI, mi)

%   AbsECI2RelHill_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Transforms satellites inertial coordinates (expressed in ECI reference frame) into relative orbital coordinates (expressed in LVLH reference
%   frame), relatively to a chief satellite, whose position and velocity must be the first six components of the input "satellites_iner"
%
%   INPUT
%   satellites_iner: Array (6*N x 1), with N equal to the number of satellites. It is built in the following way: [pos1; vel1; pos2; vel2; pos3; vel3;
%                    .... ], where the position and velocities are must be expressed in ECI reference frame. The first six components (pos1; vel1) are 
%                    the inertial coordinates of the chief satellite. 
%   mi: Earth's planetary constant [km^3/s^2]
%
%   OUTPUT
%   deputy_rel: Array (6 x N), with N equal to the number of deputy satellites. The i-th column contains the i-th deputy's position and velocity
%               expressed in LVLH reference frame
%
%   VERSION
%   20190301 V1_1:
%   -  First Release


global fh_c r_c h_c

n_sat = length(satellites_iner_ECI)/6;                                                                                                                      % Number of satellites
satellite_pos_ECI(:,1) = satellites_iner_ECI(1:3);                                                                                                          % Satellite's inertial position vector expressed in ECI (chief)
satellite_vel_ECI(:,1) = satellites_iner_ECI(4:6);                                                                                                          % Satellite's inertial velocity vector expressed in ECI (chief)
chief_omega_LVLH = [fh_c*r_c/h_c; 0; h_c/r_c^2];                                                                                                            % Chief's angular velocity expressed in LVLH

% Arrays initializatin before for cicle
vel_LVLH = zeros(3, n_sat);
relvel_LVLH = zeros(3, n_sat);
relpos_LVLH = zeros(3, n_sat);
deputy_rel_LVLH = zeros(6, n_sat-1);

for i = 2:n_sat
    
    satellite_pos_ECI(:,i) = satellites_iner_ECI(6*(i-1)+1 : 6*(i-1)+3);                                                                                    % Satellite's inertial position vector expressed in ECI
    satellite_vel_ECI(:,i) = satellites_iner_ECI(6*(i-1)+4 : 6*(i-1)+6);                                                                                    % Satellite's inertial velocity vector expressed in ECI
    coe(6*(i-1)+1 : 6*(i-1)+6) = rv2coe_V1_1(satellite_pos_ECI(:,i), satellite_vel_ECI(:,i), mi);                                                           % Satellite's classical orbital elements array
    relpos_LVLH(:,i) = ECI2LVLH_V1_1(satellite_pos_ECI(:,i) - satellite_pos_ECI(:,1), coe(6*(i-1)+3), coe(6*(i-1)+5), coe(6*(i-1)+4) + coe(6*(i-1)+6));     % i-th deputy's relative position expressed in LVLH
    vel_LVLH(:,i) = ECI2LVLH_V1_1(satellite_vel_ECI(:,i) - satellite_vel_ECI(:,1), coe(6*(i-1)+3), coe(6*(i-1)+5), coe(6*(i-1)+4) + coe(6*(i-1)+6));        % i-th deputy's velocity expressed in LVLH     
    relvel_LVLH(:,i) = vel_LVLH(:,i) - cross(chief_omega_LVLH, relpos_LVLH(:,i));                                                                           % i-th deputy's relative velocity expressed in LVLH
    deputy_rel_LVLH(:,i-1) = [relpos_LVLH(:,i); relvel_LVLH(:,i)];                                                                                          % i-th deputy's relative position and velocity expressed in LVLH
    
end

end