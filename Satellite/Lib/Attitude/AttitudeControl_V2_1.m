function tau = AttitudeControl_V2_1(satellites_attitude, satellites_iner_ECI, params)

%   AttitudeControl_V2_1.m
%   Made by Sapienza Gn Lab
%
%   Computes the attitude control torque to be applied to each deputy to make it track (reach) the desired attitude.
%
%   INPUT
%   satellites_attitude: array (7*(N+1) x 1) containing the current attitude and angular velocities of each satellite (N is the number of deputy satellites)
%   satellites_iner_ECI: Array (6*(N+1) x 1) containing the satellites' (chief + deputies) inertial coordinates (N is the number of deputy
%                        satellites).
%   params: structure containing the following fields
%       - params.Ndeputy = number of deputy satellites
%       - params.control_dir_Hill = matrix (3 x N) containing the orbital control of each deputy
%       - params.DesiredAttitude = matrix (3 x N) containing the euler angles representing the desired attitude for each deputy based on the firing direction (N is the
%                    number of deputy satellites)
%       - params.mi = Earth's gravitational constant [km^3*s^-2]
%       - params.kp = Proportional attitude control gain
%       - params.kd = Derivative attitude control gain
%
%   OUTPUT
%   tau: matrix (3 x (N+1)) containing the attitude torque to be applied to each satellite (N is the number of deputy satellites). The torque of the
%        chief is always set to zero.
%
%   VERSION
%   20191125 V2_1:
%   -  First Release

% Extract the number of deputy satellites
Ndeputy = params.Ndeputy;

% Initialize the output matrix
tau = zeros(3, Ndeputy + 1);

% Compute ECI2Hill rotation matrix and angular velocity
vect_r = satellites_iner_ECI(1:3);
vect_v = satellites_iner_ECI(4:6);
R_ECI2Hill = RECI2Hill(vect_r, vect_v);
omega_Hill2ECI_ECI = OmegaHill2ECI(vect_r, vect_v, params.mi);

% Compute the attitude torque for each satellite
for i = 1:Ndeputy
    
    if norm(params.control_dir_Hill(:,i)) > 0 % if the control is switched on -> compute the attitude torque
        
        % Extract the desired attitude for each deputy
        DesiredAttitude = params.DesiredAttitude;
        
        % Convert the desired attitude to dcm representation
        R_Hill2Body = angle2dcm(DesiredAttitude(1,i), DesiredAttitude(2,i), DesiredAttitude(3,i), 'ZYX');
        
        % Refer the desired attitude wrt ECI reference frame and convert to quaternions
        R_ECI2Body = R_Hill2Body*R_ECI2Hill;
        q_ref = dcm2quat(R_ECI2Body);
        
        % Extract the deputy current attitude and angular velocity
        q = satellites_attitude(1 + 7*i: 4 + 7*i)';
        omega_Body2ECI_Body = satellites_attitude(5 + 7*i: 7 + 7*i);
        
        % Compute the deputy current angular velocity wrt to Hill reference frame
        omega_Body2Hill_Body = omega_Body2ECI_Body - R_ECI2Body*omega_Hill2ECI_ECI;
        
        % Compute the quaternion error between current and desired attitudes
        q_err = QuaternionError_V2_1(q, q_ref);
        
        % Compute PD attitude control torque
        tau(:,i+1) =  -sign(q_err(1))*params.kp*q_err(2:4)' - params.kd*omega_Body2Hill_Body;
        
    end
    
end

end