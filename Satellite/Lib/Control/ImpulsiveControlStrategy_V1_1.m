function [DV, satellites_iner_ECI] = ImpulsiveControlStrategy_V1_1(satellites_iner_ECI, deputy_rel_LVLH, t, params)

%   ImpulsiveControlStrategy_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Computes the optimal Delta V to be exerted by multiple deputy satellite in order to follow the respective reference trajectories and computes their inertial 
%   states accordingly. The optimization process is carried out by means of linear programming optimization.
%
%   INPUT
%   satellites_iner_ECI: Array (6*(N+1) x 1) containing the satellites' (chief + deputies) inertial coordinates. N is the number of deputies.
%   deputy_rel_LVLH: Array (6 x N), with N equal to the number of deputy satellites, containing each deputy's position and velocity relative components,
%                    expressed in LVLH reference frame.
%   t: time instant in which the control has to be evaluate
%   params: structure containing the following fields
%       - params.N_burn_horizon = Number of burns
%       - params.toll = tollerance vector on the accuracy of the constraints for the optimization
%       - params.mi = Earth's planetary constant [km^3/s^2]
%       - params.n = chief's orbit mean motion [rad/s]
%
%   OUTPUT
%   DV: Array (3 x N) containing the optimal Delta V for each deputy.
%   satellites_iner_ECI: Array (6*(N+1) x 1) containing the satellites inertial coordinates corrected through the addition of the optimal Delta V.
%
%   VERSION
%   20190412 V1_1:
%   -  First Release


% Number of deputy satellite calculation
Ndeputy = size(deputy_rel_LVLH, 2);

% Extraction of constants from the "params" structure
N_burn_horizon = params.N_burn_horizon; 
toll = params.toll;
mi = params.mi;
n = params.n; 

% Hill dynamics matrix computation
A_hcw = [0 0 0 1 0 0;
         0 0 0 0 1 0;
         0 0 0 0 0 1;
         3*n^2 0 0 0 2*n 0;
         0 0 0 -2*n 0 0;
         0 0 -n^2 0 0 0];
     
% State Transition Matrix
PHI = expm(A_hcw*params.DT_burn);

% Initialize the output
DV = zeros(3,Ndeputy);
    
% Iterate over number of deputies in order to find the optimal Delta V for each deputy
for j = 1:Ndeputy
    
    % COmpute the reference trajectory
    x_des = params.deputy(j).TrajectoryProfile(t, params.deputy(j));
    
    % Compute the optimal Delta V through linear programming optimization
    DV_keeping = linprog_keeping_V1_1(N_burn_horizon, PHI, x_des, deputy_rel_LVLH(:,j), toll);
    
    % Store the Delta V
    DV(:,j) = DV_keeping(1:3,1) - DV_keeping(4:6,1);
    
    % Correct the j-th deputy relative state by adding DeltaV to velocities components
    deputy_rel_LVLH(:,j) = deputy_rel_LVLH(:,j) + [zeros(3,1); DV(:,j)];
    
    % Compute chief COE
    chief_coe = rv2coe_V1_1(satellites_iner_ECI(1:3), satellites_iner_ECI(4:6), mi);
    
    % Correct j-th deputy inertial coordinates
    satellites_iner_ECI(1 + 6*j : 6 + 6*j, 1) = rel2iner_V1_1(deputy_rel_LVLH(:,j)', satellites_iner_ECI(1:6)', chief_coe, params);
    
end

end
