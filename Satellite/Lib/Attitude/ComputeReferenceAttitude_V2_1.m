function DesiredAttitude = ComputeReferenceAttitude_V2_1(satellites_attitude, satellites_iner_ECI, params)

%   ComputeReferenceAttitude_V2_1.m
%   Made by Sapienza Gn Lab
%
%   Computes the desired attitude based on the firing direction for each deputy satellite.
%
%   INPUT
%   params: structure containing the following fields
%       - params.Ndeputy = number of deputy satellites
%       - params.control_dir_Hill = matrix (3 x N) containing the firing direction for each of the N deputy satellites
%   satellites_attitude: array (7*(N+1) x 1) containing the current attitude and angular velocities of each satellite (N is the number of deputy satellites)
%   satellites_iner_ECI: array (6*(N+1) x 1) containing the inertial position and velocity of each satellite (N is the number of deputy satellites)
%
%   OUTPUT
%   DesiredAttitude: matrix (3 x N) containing the euler angles representing the desired attitude for each deputy based on the firing direction (N is the 
%                    number of deputy satellites)
%
%   VERSION
%   20191125 V2_1:
%   -  First Release

% Extract the number of deputy satellite
Ndeputy = params.Ndeputy ;

% Initialize the desired attitude matrix
DesiredAttitude = zeros(3, Ndeputy);

% Compute ECI2Hill rotation matrix
vect_r = satellites_iner_ECI(1:3);
vect_v = satellites_iner_ECI(4:6);
R_ECI2Hill = RECI2Hill(vect_r, vect_v);
    
for i = 1:Ndeputy
    
    % Compute i-th deputy current real attitude wrt Hill reference frame
    quat_ECI2Body = satellites_attitude(1 + 7*i: 4 + 7*i)';
    R_ECI2Body = quat2dcm(quat_ECI2Body);
    R_Hill2Body = R_ECI2Body*R_ECI2Hill';
    quat_Hill2Body = dcm2quat(R_Hill2Body);
    ang_dep = quat2eul(quat_Hill2Body , 'ZYX');
    
    % Extract desired firing direction
    FiringDirection = params.control_dir_Hill(:,i);
    
    % Compute Euler angles of the desired firing direction wrt Hill reference frame
        
    phi = -asin(FiringDirection(3));
    psi = atan2(FiringDirection(2), FiringDirection(1));
    
    if psi < 0
        
        psi = psi + 2*pi;
        
    end
    
    % Since the rotation around the firing direction is not constrained, use the current deputy theta
    theta = ang_dep(3);
    
    % Desired attitude for i-th deputy expressed in Euler angles
    DesiredAttitude(1:3,i) = [psi, phi, theta];
    
end

end