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
    eul_Hill2Body = [0, 0, 0]*180/pi;
%     eul_Hill2Body = 1*(DynOpt.target_attitude')*180/pi;
    %%%%%%%%%%%%%%%%%%%%%%%
    q_Hill2Body = RotationConversion_V2_1('EA321toQ', eul_Hill2Body);
    omega_Body2Hill_Body = [5, 5, 5]*pi/180;
    R_Hill2Body = RotationConversion_V2_1('QtoDCM', q_Hill2Body);
    R_ECI2Body = R_Hill2Body*R_ECI2Hill;
    
    %%%%% TEST CONTROL %%%%
%     q_ECI2Body = RotationConversion_V2_1('EA321toQ', eul_Hill2Body);
    q_ECI2Body = RotationConversion_V2_1('DCMtoQ', R_ECI2Body);
    q_ECI2Body = ConvertQuat_V2_1(q_ECI2Body, 'ScalarTo1');
    %%%%%%%%%%%%%%%%%%%%%%%
    
    omega_Body2ECI_Body = omega_Body2Hill_Body' + R_ECI2Body*omega_Hill2ECI_ECI;
    satellites_attitude(1:7,1) = [q_ECI2Body'; omega_Body2ECI_Body];
    
end