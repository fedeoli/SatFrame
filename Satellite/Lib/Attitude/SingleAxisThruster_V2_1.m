function [u_out, satellites_iner_ECI, params] = SingleAxisThruster_V2_1(t, u, satellites_iner_ECI, deputy_rel_LVLH, satellites_attitude, params)

%   SingleAxisThruster_V2_1.m
%   Made by Sapienza Gn Lab
%
%   Computes the desired attitude based on the firing direction for each deputy satellite.
%
%   INPUT
%   t: time [s]
%   u: matrix (3 x N) containing the orbital control for each deputy (N is the number of deputy satellites)
%   satellites_iner_ECI: array (6*(N+1) x 1) containing the inertial position and velocity of each satellite (N is the number of deputy satellites)
%   deputy_rel_LVLH: array (6 x N), with N equal to the number of deputy satellites, containing each deputy's position and velocity relative components,
%                    expressed in LVLH reference frame
%   satellites_attitude: array (7*(N+1) x 1) containing the current attitude and angular velocities of each satellite (N is the number of deputy satellites)
%   params: structure containing the following fields
%       - params.Ndeputy = number of deputy satellites
%       - params.mi = Earth's planetary constant [km^3*s^-2]
%       - params.ControlStrategy = flag for the selection of the Control Strategy to be adopted (either 'Impulsive' or 'Continuous')
%       - params.DesiredAttitude = matrix (3 x N) containing the euler angles representing the desired attitude for each deputy based on the firing direction (N is the 
%                    number of deputy satellites)
%       - params.att_threshold = maximum attitude error allowable for orbital control application [deg]
%       - params.control_dir_Hill = matrix (3 x N) containing the firing direction for each of the N deputy satellites
%       - params.Burn_executed = 
%
%   OUTPUT
%   u_out: matrix (3 x N) containing the control to be actuated by the N deputy satellites
%   satellites_iner_ECI: array (6*(N+1) x 1) containing the modified inertial position and velocity of all the satellites. This output variable is 
%                        modified only if the impulsive strategy is adopted (Delta V is added to velocity components)
%   params: structure in which the following fields are modified (this happens only if the impulsive control strategy is used)
%       - params.check_attitude: 
%       - params.WaitingTime:
%       - params.Burn_executed: 
%
%   VERSION
%   20191125 V2_1:
%   -  First Release

% Extract the number of deputies
N_deputy = params.Ndeputy;

% Extract Earth's planetary constant
mi = params.mi;

% Initialize output control vector
u_out = zeros(3, N_deputy);

% Compute ECI2Hill rotation matrix
vect_r = satellites_iner_ECI(1:3);
vect_v = satellites_iner_ECI(4:6);
R_ECI2Hill = RECI2Hill(vect_r, vect_v);

if  strcmpi(params.ControlStrategy,  'Continuous')
    
    for j = 1:N_deputy
        
        % Compute deputy's attitude quaternion vector wrt Hill reference frame
        quat_ECI = satellites_attitude(1 + 7*j: 4 + 7*j)';
        R_ECI2Body = quat2dcm(quat_ECI);
        R_Hill2Body = R_ECI2Body*R_ECI2Hill';
        quat_Hill = dcm2quat(R_Hill2Body);
        
        % Extract desired attitude and convert to quaternions
        quat_ref = eul2quat(params.DesiredAttitude(:,j)', 'ZYX');
        
        % Compute the quaternion error between current and desired attitude for j-th deputy
        q_err = QuaternionError_V2_1(quat_Hill,quat_ref);
        
        % Check if the error is lower than "params.att_threshold" threshold
        if 2*abs(acos(abs(q_err(1))))*180/pi < params.att_threshold
            
            u_intensity = norm(u(:,j));
            R_Hill2Dep = quat2dcm(quat_Hill);
            R_Dep2Hill = R_Hill2Dep';
            u_out(:,j) = R_Dep2Hill*[u_intensity; 0; 0];
            
        else
            
            u_out(:,j) = u(:,j)*0;
            
        end
        
    end
    
elseif strcmpi(params.ControlStrategy,  'Impulsive')
    
    params.check_attitude = 1;
    
    for j = 1:N_deputy
        
        quat_ECI = satellites_attitude(1 + 7*j: 4 + 7*j)';
        R_ECI2Body = quat2dcm(quat_ECI);
        R_Hill2Body = R_ECI2Body*R_ECI2Hill';
        quat_Hill = dcm2quat(R_Hill2Body);
        quat_ref = eul2quat(params.DesiredAttitude(:,j)','ZYX');
        q_err = QuaternionError_V2_1(quat_Hill,quat_ref);
        params.check_attitude = params.check_attitude*(abs(acos(abs(q_err(1))))*180/pi < params.att_threshold);
        
    end
    
    if (params.check_attitude == 1) && (params.Burn_executed == 0)
        
        for j = 1:N_deputy
            
            quat_ECI = satellites_attitude(1 + 7*j: 4 + 7*j)';
            R_ECI2Body = quat2dcm(quat_ECI);
            R_Hill2Body = R_ECI2Body*R_ECI2Hill';
            quat_Hill = dcm2quat(R_Hill2Body);
            u_intensity = norm(u(:,j));
            R_Hill2Dep = quat2dcm(quat_Hill);
            R_Dep2Hill = R_Hill2Dep';
            
            % %                 % Correct the j-th deputy relative state by adding DeltaV to velocities components
            DV = params.DV(:,round(t/params.time_step),j);
            
            DV_applied = R_Dep2Hill*[norm(DV);0;0];
            
            deputy_rel_LVLH(:,j) = deputy_rel_LVLH(:,j) + [zeros(3,1); DV_applied];
            %      % Compute chief COE
            chief_coe = rv2coe_V1_1(satellites_iner_ECI(1:3), satellites_iner_ECI(4:6), mi);
            %      % Correct j-th deputy inertial coordinates
            satellites_iner_ECI(1 + 6*j : 6 + 6*j, 1) = rel2iner_V1_1(deputy_rel_LVLH(:,j)', satellites_iner_ECI(1:6)', chief_coe, params);
            params.WaitingTime = t + params.DT_burn;
            params.Burn_executed = 1;
            
            
        end
        
    else
        
        for j = 1:N_deputy
            
            params.DV(:,round(t/params.time_step),j) = [0; 0; 0];
            u_out(:,j) = zeros(3,1);
            
        end
        
    end
    
end

end