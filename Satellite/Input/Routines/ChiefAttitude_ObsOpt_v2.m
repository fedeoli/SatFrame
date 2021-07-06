%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 CHIEF ATTITUDE PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if params.Attitude
    
    global DynOpt
    
    % Compute Hill's attitude and angular velocity wrt ECI
    vect_r = satellites_iner_ECI(1:3);
    vect_v = satellites_iner_ECI(4:6);
    %%%%% TEST %%%%%
%     omega_Hill2ECI_ECI = [0;0;0];
    omega_Hill2ECI_ECI = OmegaHill2ECI(vect_r, vect_v, params.mi);
    R_ECI2Hill = RECI2Hill(vect_r, vect_v);
    
    % Compute chief's attitude and angular velocity
    
    %%%%% TEST CONTROL %%%%
%     eul_Hill2Body = [0, 0, 0]*180/pi;
    eul_Hill2Body = 1*(DynOpt.target_attitude');
    %%%%%%%%%%%%%%%%%%%%%%
    
    
    %%%% conversions %%%%
    q_ECI2Body = eul2quat(eul_Hill2Body);
    R_ECI2Body = quat2dcm(q_ECI2Body);
    omega_Body2Hill_Body = [5, 5, 5]*pi/180;
    
    omega_Body2ECI_Body = omega_Body2Hill_Body' + R_ECI2Body*omega_Hill2ECI_ECI;
    satellites_attitude(1:7,1) = [q_ECI2Body'; omega_Body2ECI_Body];
    
end