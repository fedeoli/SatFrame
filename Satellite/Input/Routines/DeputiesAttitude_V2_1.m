for i = 1:N_deputy
    
    % Compute deputy's attitude and angular velocity
    eul_Hill2Body = [10, 0, 0]*pi/180;
    q_Hill2Body = eul2quat(eul_Hill2Body, 'ZYX');
    R_Hill2Body = quat2dcm(q_Hill2Body);
    % omega_Body2Hill_Body = [0.2, 0.1, 0.4]*pi/180;
    omega_Body2Hill_Hill = [0, 0, 0]*pi/180;
    omega_Body2Hill_Body = R_Hill2Body*omega_Body2Hill_Hill';
    R_Hill2Body = quat2dcm(q_Hill2Body);
    R_ECI2Body = R_Hill2Body*R_ECI2Hill;
    q_ECI2Body = dcm2quat(R_ECI2Body);
    omega_Body2ECI_Body = omega_Body2Hill_Body + R_ECI2Body*omega_Hill2ECI_ECI;
    satellites_attitude(1 + 7*i : 7*(i+1), 1) = [q_ECI2Body' ; omega_Body2ECI_Body];
    
    
end

params.SatellitesAttitude = satellites_attitude(:,1);