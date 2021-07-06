function [params, satellites_attitude] = ChiefAttitude_ObsOpt_v2_function(params,DynOpt,satellites_iner_ECI)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                 CHIEF ATTITUDE PARAMETERS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if params.Attitude

        % Compute Hill's attitude and angular velocity wrt ECI
        vect_r = satellites_iner_ECI(1:3);
        vect_v = satellites_iner_ECI(4:6);
        
        omega_Hill2ECI_ECI = OmegaHill2ECI(vect_r, vect_v, params.mi);
        
        R_ECI2Hill = RECI2Hill(vect_r, vect_v);

        eul_Hill2Body = [0, 0, 0]*180/pi;


        %%%% conversions %%%%
        q_ECI2Body = eul2quat(eul_Hill2Body);
        R_ECI2Body = quat2dcm(q_ECI2Body);
        
        randstart = DynOpt.randstart*2*randn(1,3);
        omega_Body2Hill_Body = ([5, 5, 5]+randstart)*pi/180;
        

        omega_Body2ECI_Body = omega_Body2Hill_Body' + R_ECI2Body*omega_Hill2ECI_ECI;
        satellites_attitude(1:7,1) = [q_ECI2Body'; omega_Body2ECI_Body];
        
        params.omega_Hill2ECI_ECI = omega_Hill2ECI_ECI;
        params.R_ECI2Hill = R_ECI2Hill;
    end
end