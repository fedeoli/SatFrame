%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 CHIEF ATTITUDE PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Compute Hill's attitude and angular velocity wrt ECI
vect_r = satellites_iner_ECI(1:3);
vect_v = satellites_iner_ECI(4:6);
omega_Hill2ECI_ECI = OmegaHill2ECI(vect_r, vect_v, params.mi);
R_ECI2Hill = RECI2Hill(vect_r, vect_v);

% Compute chief's attitude and angular velocity
eul_Hill2Body = [10, 0, 0]*pi/180;
q_Hill2Body = eul2quat(eul_Hill2Body, 'ZYX');
omega_Body2Hill_Body = [0.2, 0.1, 0.4]*pi/180*0;
R_Hill2Body = quat2dcm(q_Hill2Body);
R_ECI2Body = R_Hill2Body*R_ECI2Hill;
q_ECI2Body = dcm2quat(R_ECI2Body);
omega_Body2ECI_Body = omega_Body2Hill_Body' + R_ECI2Body*omega_Hill2ECI_ECI;
satellites_attitude(1:7,1) = [q_ECI2Body'; omega_Body2ECI_Body];
