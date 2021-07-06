function deputy_rel_LVLH = AbsECI2RelHill_V1_2(satellites_iner_ECI, mi, params)

n_sat = length(satellites_iner_ECI)/6;                                                                                                                      % Number of satellites
satellite_pos_ECI(:,1) = satellites_iner_ECI(1:3);                                                                                                          % Satellite's inertial position vector expressed in ECI (chief)
satellite_vel_ECI(:,1) = satellites_iner_ECI(4:6);                                                                                                          % Satellite's inertial velocity vector expressed in ECI (chief)
coe = rv2coe_V1_1(satellite_pos_ECI(:,1), satellite_vel_ECI(:,1), mi);
chief_omega_LVLH = [params.fh_c*params.r_c/params.h_c; 0; params.h_c/params.r_c^2];                                                                                                            % Chief's angular velocity expressed in LVLH

% Arrays initializatin before for cicle
vel_LVLH = zeros(3, n_sat);
relvel_LVLH = zeros(3, n_sat);
relpos_LVLH = zeros(3, n_sat);
deputy_rel_LVLH = zeros(6, n_sat-1);

for i = 2:n_sat
    
    satellite_pos_ECI(:,i) = satellites_iner_ECI(6*(i-1)+1 : 6*(i-1)+3);                                                                                    % Satellite's inertial position vector expressed in ECI
    satellite_vel_ECI(:,i) = satellites_iner_ECI(6*(i-1)+4 : 6*(i-1)+6);                                                                                    % Satellite's inertial velocity vector expressed in ECI
    relpos_LVLH(:,i) = ECI2LVLH_V1_1(satellite_pos_ECI(:,i) - satellite_pos_ECI(:,1), coe(3), coe(5), coe(4) + coe(6));     % i-th deputy's relative position expressed in LVLH
    vel_LVLH(:,i) = ECI2LVLH_V1_1(satellite_vel_ECI(:,i) - satellite_vel_ECI(:,1), coe(3), coe(5), coe(4) + coe(6));        % i-th deputy's velocity expressed in LVLH     
    relvel_LVLH(:,i) = vel_LVLH(:,i) - cross(chief_omega_LVLH, relpos_LVLH(:,i));                                                                           % i-th deputy's relative velocity expressed in LVLH
    deputy_rel_LVLH(:,i-1) = [relpos_LVLH(:,i); relvel_LVLH(:,i)];                                                                                          % i-th deputy's relative position and velocity expressed in LVLH
    
end

end