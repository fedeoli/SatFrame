function DV = linprog_keeping_V1_1(N_burn_horizon, PHI, x_des, deputy_rel_LVLH, toll)

%   linprog_keeping_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Computes the optimal Delta V to be exerted by a single deputy in order to follow the reference trajectory. The optimization process is
%   carried out by means of linear programming optimization.
%
%   INPUT
%   N_burn_horizon: Number of burns.
%   PHI: State transition matrix.
%   x_des: reference trajectory to be followed at time "t"
%   deputy_rel_LVLH: deputy state vector
%   toll: array of tolerances on the optimization constraints
%
%   OUTPUT
%   DV: Array (3 x 1) containing the optimal Delta V.
%
%   VERSION
%   20190412 V1_1:
%   -  First Release


A_keeping = [];
b_keeping = [];

for j = 1: N_burn_horizon/2+1
    
    A = zeros(6, 6*N_burn_horizon);
    M = N_burn_horizon + 1 - j;
    
    for i = 1 : M
        
        phi_aux = PHI^(M + 1 - i);
        phi_aux = [phi_aux(1:6, 4:6) -phi_aux(1:6, 4:6)] ;
        A(1:6, 6*(i - 1) + 1 : 6*i) = phi_aux;
        
    end
    
    A = [A; -A];
    A_keeping = [A_keeping; A];
    x_des_i = PHI^(M)*x_des;
    b = [x_des_i - PHI^M*deputy_rel_LVLH + toll; -x_des_i + PHI^M*deputy_rel_LVLH + toll];
    b_keeping = [b_keeping; b];
    
end

lb = zeros(3*N_burn_horizon*2, 1);                      % optimization's lower boundary
ub = 0.01*ones(3*N_burn_horizon*2, 1);                  % optimization's upper boundary
f = ones(1, 6*N_burn_horizon);                          % function to be optimizaed (sum of Delta V)
options = optimoptions('linprog', 'Display', 'off');    % Optimization options

% Linear programming optimization
[DV, ~, exitflag] = linprog(f', A_keeping, b_keeping, [], [], lb, ub, options);

% Warning if optimization does not succeed
if exitflag < 1
    
    disp('WARNING: Optimal solution not found. Selecting zero DV input')
    DV =  zeros(3*N_burn_horizon*2, 1);
    
end

end


