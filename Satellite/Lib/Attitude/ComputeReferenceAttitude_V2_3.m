function params = ComputeReferenceAttitude_V2_3(t, params)

%   ComputeReferenceAttitude_V2_3.m
%   Made by Sapienza Gn Lab
%
%   Computes the desired attitude to be followed by each deputy either based on the firing direction (if orbital control is on) or based on the required mission
%   profile (if the orbital control is on and the strategy is impulsive).
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
%   - First Release
%
%   20200113 V2_2:
%   - With Impulsive control strategy: between two subsequent impulses the deputies are required to perform an imposed attitude maneuver, based on the
%     required mission profile.
%
%   20200525 V2_3:
%   - The firing attitude (first branch of the if/else) is computed also if t < params.ParkingTime & params.CollisionFlag = 1, i.e. if a collision occurs before
%     the end of the parking time.

% Extract the number of deputy satellite
Ndeputy = params.Ndeputy;

% Initialize the desired attitude matrix
DesiredAttitude = zeros(3, Ndeputy + 1);

for i = 1:Ndeputy + 1
    
    if ( t > params.ParkingTime && i > 1 && ( (strcmpi(params.ControlStrategy, 'Continuous') && params.u_module(i) > 0) || (strcmpi(params.ControlStrategy, 'Impulsive') && params.sat(i).Burn_executed == 0 && params.sat(i).IsBurning == 0 && params.u_module(i) > 0) ) ) || ( t <= params.ParkingTime && params.CollisionFlag == 1 && params.sat(i).CA_Maneuvering == 1 )
        
        % Extract desired firing direction
        FiringDirection = params.control_dir_Hill(:,i);
        
        % Compute Euler angles of the desired firing direction wrt Hill reference frame
        
        phi = -asin(FiringDirection(3));
        psi = atan2(FiringDirection(2), FiringDirection(1));
        
        if psi < 0
            
            psi = psi + 2*pi;
            
        end
        
        % Since the rotation around the firing direction is not constrained, use the current deputy theta
        theta = 0;
        
        % Desired attitude for i-th deputy expressed in Euler angles
        DesiredAttitude(:,i) = [psi; phi; theta];
        
    elseif t > params.ParkingTime && i > 1 && ( strcmpi(params.ControlStrategy, 'Impulsive') && params.sat(i).Burn_executed == 0 && params.sat(i).IsBurning == 1 )
        
        DesiredAttitude(:,i) = params.sat(i).FiringAttitude;
        
    elseif ( t > params.ParkingTime && i > 1 && params.sat(i).CanManeuverAttitude == 1 && ( strcmpi(params.ControlStrategy, 'Impulsive') && ( params.sat(i).Burn_executed == 1 || ( params.sat(i).Burn_executed == 0 && params.sat(i).IsBurning == 0 && params.u_module(i) == 0 ) ) ) ) || ( i == 1 && params.sat(1).CanManeuverAttitude == 1 )
        
        switch params.sat(i).MissionAttitude
            
            case 'Nadir pointing x'
                
                DesiredAttitude(:,i) = NadirPointing_V2_1('X');
                
            case 'Nadir pointing y'
                
                DesiredAttitude(:,i) = NadirPointing_V2_1('Y');
                
            case 'Nadir pointing z'
                
                DesiredAttitude(:,i) = NadirPointing_V2_1('Z');
                
            case 'No maneuver'
                
                params.sat(i).CanManeuverAttitude = 0;
                
            case 'Hold attitude'
                
                DesiredAttitude(:,i) = params.sat(i).FiringAttitude;
                
        end
        
    elseif t > params.ParkingTime && i > 1 && ( strcmpi(params.ControlStrategy, 'Continuous') && isempty(params.sat(i).FiringAttitude) == 0 )
        
        DesiredAttitude(:,i) = params.sat(i).FiringAttitude;
        
    end
    
end

params.DesiredAttitude = DesiredAttitude;

end