function DV = ImpulsiveControlStrategySingle_V2_2(deputy_rel_LVLH, t, params, deputy)

%   ImpulsiveControlStrategySingle_V2_2.m
%   Made by Sapienza Gn Lab
%
%   Computes the optimal Delta V to be exerted by a single deputy satellite in order to follow the inputted reference trajectory and computes its inertial 
%   state accordingly. The optimization process is carried out by means of linear programming optimization.
%
%   INPUT
%   deputy_rel_LVLH: Array (6 x 1), containing the deputy's position and velocity relative components, expressed in LVLH reference frame.
%   t: time instant in which the control has to be evaluated
%   params: structure containing the following fields
%       - params.N_burn_horizon = Number of burns
%       - params.toll = tollerance vector on the accuracy of the constraints for the optimization
%       - params.mi = Earth's planetary constant [km^3/s^2]
%       - params.n = chief's orbit mean motion [rad/s]
%
%   OUTPUT
%   DV: Array (3 x 1) containing the optimal Delta V.
%
%   VERSION
%   20200109 V2_1:
%   - First release.


% Extraction of constants from the "params" structure
N_burn_horizon = params.N_burn_horizon; 
toll = params.toll;
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

% Compute the reference trajectory
x_des = deputy.TrajectoryProfile(t, deputy);

% Compute Dv upper boundary
if isfield(deputy.Thruster, 'MaximumBurningTime') && params.RealThruster == 1
    
    DvMax = deputy.Thruster.MaxThrust*deputy.Thruster.MaximumBurningTime;
    
else
    
    DvMax = 0.01;
    
end

% Compute the optimal Delta V through linear programming optimization
DV_keeping = linprog_keeping_V2_1(N_burn_horizon, PHI, x_des, deputy_rel_LVLH, toll, DvMax);

% Store the Delta V
DV = DV_keeping(1:3,1) - DV_keeping(4:6,1);


end
