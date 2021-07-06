function [u_out, satellites_iner_ECI, params] = SingleAxisThruster_V2_3(t, u, satellites_iner_ECI, deputy_rel_LVLH, satellites_attitude, params)

%   SingleAxisThruster_V2_3.m
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
%       - params.ParkingTime = time to be waited from all the deputies until the orbital control can be switched on [s]
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
%   - First Release
%
%   20200109 V2_2:
%   - Inclusion of multiple waiting times for diferent deputies
%
%   20200110 V2_3:
%   - The function has been modified to take into account that "params.deputy" has been chaged to "params.sat"
%   - Solved an issue for which the waiting time was updated even if the orbital control had not been switched on yet.

% Extract the number of deputies
N_deputy = params.Ndeputy;

% Initialize output control vector
u_out = zeros(3, N_deputy);

% Compute ECI2Hill rotation matrix
vect_r = satellites_iner_ECI(1:3);
vect_v = satellites_iner_ECI(4:6);
    
    if  strcmpi(params.ControlStrategy, 'Continuous') && t >= params.ParkingTime
        
        for j = 1:N_deputy
            
            if params.u_module(j+1) > 0 && params.Attitude
                
                % Compute deputy's attitude quaternion vector wrt Hill reference frame
                quat_ECI = satellites_attitude(1 + 7*j: 4 + 7*j)';
                R_ECI2Body = quat2dcm(quat_ECI);
                R_ECI2Hill = RECI2Hill(vect_r, vect_v);
                R_Hill2Body = R_ECI2Body*R_ECI2Hill';
                quat_Hill = dcm2quat(R_Hill2Body);
                
                % Extract desired attitude and convert to quaternions
                quat_ref = eul2quat(params.DesiredAttitude(:,j+1)', 'ZYX');
                
                % Compute the quaternion error between current and desired attitude for j-th deputy
                q_err = QuaternionError_V2_1(quat_Hill, quat_ref);
                
                params.sat(j+1).check_attitude = abs(acos(abs(q_err(1))))*180/pi < params.sat(j+1).att_threshold;
                
                % Check if the error is lower than "params.att_threshold" threshold
                if params.sat(j+1).check_attitude == 1
                    
                    params.sat(j+1).FiringAttitude = params.DesiredAttitude(:,j+1);
                    u_intensity = norm(u(:,j));
                    
                    if params.RealThruster
                        
                        if u_intensity > (params.sat(j+1).Thruster.MaxThrust)/params.sat(j+1).M
                            
                            u_intensity = (params.sat(j+1).Thruster.MaxThrust)/params.sat(j+1).M;
                            
                        end
                        
                        if params.PWM
                            
                            u_nominal = (params.sat(j+1).Thruster.MaxThrust)/params.sat(j+1).M;
                            u_real = (params.sat(j+1).Thruster.MaxThrust + params.sat(j+1).Thruster.Resolution/3*randn)/params.sat(j+1).M;
                            u_intensity  = PWM_V2_1(t, u_intensity, u_nominal, u_real, params.dutyPeriod);
                            
                        else
                            
                            u_intensity = u_intensity + params.sat(j+1).Thruster.Resolution/3*randn/params.sat(j+1).M;
                            
                        end
                        
                    end
                    
                    u_out(:,j) = R_Hill2Body'*[u_intensity; 0; 0];
                    
                end
                
            elseif params.Attitude == 0
                
                u_intensity = norm(u(:,j));
                
                if params.RealThruster
                    
                    if u_intensity > (params.sat(j+1).Thruster.MaxThrust)/params.sat(j+1).M
                        
                        u_intensity = (params.sat(j+1).Thruster.MaxThrust)/params.sat(j+1).M;
                        
                    end
                    
                    if params.PWM
                        
                        u_nominal = (params.sat(j+1).Thruster.MaxThrust)/params.sat(j+1).M;
                        u_real = (params.sat(j+1).Thruster.MaxThrust + params.sat(j+1).Thruster.Resolution/3*randn)/params.sat(j+1).M;
                        u_intensity  = PWM_V2_1(t, u_intensity, u_nominal, u_real, params.dutyPeriod);
                        
                    else
                        
                        u_intensity = u_intensity + params.sat(j+1).Thruster.Resolution/3*randn/params.sat(j+1).M;
                        
                    end
                    
                end
                
                u_out(:,j) = (u_intensity/norm(u(:,j)))*u(:,j);
                
            end
            
        end
        
    elseif strcmpi(params.ControlStrategy,  'Impulsive') && t >= params.ParkingTime
        
        for j = 1:N_deputy
            
            if ( params.u_module(j+1) > 0 || params.sat(j+1).IsBurning ) && params.Attitude
                
                quat_ECI = satellites_attitude(1 + 7*j: 4 + 7*j)';
                R_ECI2Body = quat2dcm(quat_ECI);
                R_ECI2Hill = RECI2Hill(vect_r, vect_v);
                R_Hill2Body = R_ECI2Body*R_ECI2Hill';
                quat_Hill = dcm2quat(R_Hill2Body);
                quat_ref = eul2quat(params.DesiredAttitude(:,j+1)', 'ZYX');
                q_err = QuaternionError_V2_1(quat_Hill, quat_ref);
                params.sat(j+1).FiringAttitude = params.DesiredAttitude(:,j+1);
                params.sat(j+1).check_attitude = abs(acos(abs(q_err(1))))*180/pi < params.sat(j+1).att_threshold;
                
                if params.sat(j+1).check_attitude == 1 && params.sat(j+1).Burn_executed == 0
                    
                    params.sat(j+1).WaitingTime = t + params.DT_burn;
                    
                    if params.RealThruster
                        
                        if params.sat(j+1).FiniteBurnTime > 0
                            
                            params.sat(j+1).IsBurning = 1;
                            params.sat(j+1).FiniteBurnTime = params.sat(j+1).FiniteBurnTime - params.time_step;
                            u_real = (params.sat(j+1).Thruster.MaxThrust + params.sat(j+1).Thruster.Resolution/3*randn)/params.sat(j+1).M;
                            u_out(:,j) = R_Hill2Body'*[u_real; 0; 0];
                            
                        else
                            
                            params.sat(j+1).Burn_executed = 1;
                            params.sat(j+1).IsBurning = 0;
                            
                        end
                        
                    else
                        
                        % Correct the j-th deputy relative state by adding DeltaV to velocities components
                        DV = params.DV(:, round(t/params.time_step), j);
                        DV_applied = R_Hill2Body'*[norm(DV); 0; 0];
                        deputy_rel_LVLH(:,j) = deputy_rel_LVLH(:,j) + [zeros(3,1); DV_applied];
                        
                        % Compute chief COE
                        chief_coe = rv2coe_V1_1(satellites_iner_ECI(1:3), satellites_iner_ECI(4:6), params.mi);
                        
                        % Correct j-th deputy inertial coordinates
                        satellites_iner_ECI(1 + 6*j : 6 + 6*j, 1) = rel2iner_V2_2(deputy_rel_LVLH(:,j)', satellites_iner_ECI(1:6)', chief_coe, params);
                        params.sat(j+1).Burn_executed = 1;
                        params.sat(j+1).IsBurning = 0;
                        
                    end
                    
                elseif ( params.u_module(j+1) > 0 || params.sat(j+1).IsBurning ) && params.Attitude == 0
                    
                    params.DV(:, round(t/params.time_step), j) = [0; 0; 0];
                    
                end
                
            elseif params.Attitude == 0 && params.RealThruster && params.sat(j+1).Burn_executed == 0
                
                params.sat(j+1).WaitingTime = t + params.DT_burn;
                
                if params.sat(j+1).FiniteBurnTime > 0
                    
                    DV = norm(params.DV(:, round(t/params.time_step), j));
                    params.sat(j+1).FiniteBurnTime = params.sat(j+1).FiniteBurnTime - params.time_step;
                    u_real = (params.sat(j+1).Thruster.MaxThrust + params.sat(j+1).Thruster.Resolution/3*randn)/params.sat(j+1).M;
                    
                    if DV > 0
                        
                        u_direction = params.DV(:, round(t/params.time_step), j)/DV;
                        params.u_direction(:,j) = u_direction;
                        
                    end
                    
                    u_out(:,j) = u_real*params.u_direction(:,j);
                    
                else
                    
                    params.sat(j+1).Burn_executed = 1;
                    
                end
                
            else
                
                params.DV(:, round(t/params.time_step), j) = [0; 0; 0];
                
            end
            
        end
        
    end

end