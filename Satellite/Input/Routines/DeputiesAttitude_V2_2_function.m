function [params, satellites_attitude] = DeputiesAttitude_V2_2_function(params,DynOpt,satellites_attitude,N_deputy)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   DEPUTIES ATTITUDE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if params.Attitude
    
    eul_Hill2Body = [10, 0, 0];
    
    for i = 1:N_deputy
        
        R_ECI2Hill = params.R_ECI2Hill;
        omega_Hill2ECI_ECI = params.omega_Hill2ECI_ECI;
        
        % Compute deputy's attitude and angular velocity
        q_Hill2Body = RotationConversion_V2_1('EA321toQ', eul_Hill2Body);
        R_Hill2Body = RotationConversion_V2_1('QtoDCM', q_Hill2Body);
        randstart = DynOpt.randstart*2*randn(1,3);
        omega_Body2Hill_Hill = (params.Omega0.*[1, 1, 1]+randstart);
        omega_Body2Hill_Body = R_Hill2Body*omega_Body2Hill_Hill';
        R_Hill2Body = RotationConversion_V2_1('QtoDCM', q_Hill2Body);
        
        R_ECI2Body = R_Hill2Body*R_ECI2Hill;
        q_ECI2Body = RotationConversion_V2_1('DCMtoQ', R_ECI2Body);
        q_ECI2Body = ConvertQuat_V2_1(q_ECI2Body, 'ScalarTo1');
        omega_Body2ECI_Body = omega_Body2Hill_Body + R_ECI2Body*omega_Hill2ECI_ECI;
        satellites_attitude(1 + 7*i : 7*(i+1), 1) = [q_ECI2Body' ; omega_Body2ECI_Body];

    end
    
    params.SatellitesAttitude = satellites_attitude(:,1);
    
end