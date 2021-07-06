%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                  ORBITAL CONTROL PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if strcmpi(params.ControlStrategy, 'Continuous') % Continuous Control Parameters
    
    % LQR tuning gain matrices
    params.R = 1e12*eye(3);
    params.Q = diag([1, 1, 1, 1/n^2, 1/n^2, 1/n^2]);
    
    % Mapping control matrix
    params.B = [0 0 0;
        0 0 0;
        0 0 0;
        1 0 0;
        0 1 0;
        0 0 1];
    
    % Time step for Riccati differential equation integration
    params.tRiccati = 20;
    
    % Reshape of the R and B matrices if "twoAxesControl" option has been activated
    if params.twoAxesControl
        
        params.B = [0 0;
            0 0;
            0 0;
            0 0;
            1 0;
            0 1];
        params.R = 1e12*eye(2);
        
    end
    
    if params.PWM
        
        % Duty period of the sawteeth (needed for PWM)
        params.dutyPeriod = 50;
        
    end
    
    params.FiringThreshold = 1e-9;  % Minimum Threshold. If  [km/s^2]
    
elseif strcmpi(params.ControlStrategy, 'Impulsive') % Impulsive Control Parameters
    
    % Parameters for impulsive control strategy
    T_horizon = 2*T;
    params.DT_burn = T_horizon/10;                                  % The impulsive DT will be applied every DT_burn. Suggestion: select odd fractions of the period
    params.N_burn_horizon = floor(T_horizon/params.DT_burn);        % Number of burns
    params.toll = [1e-3; 1e-3; 1e-3; 1e-6; 1e-6; 1e-6];             % Tollerance on the optimization constraints
    
    params.FiringThreshold = 5e-6;  % [km/s]
    
end

if params.Attitude == 0
    
    params.FiringThreshold = 0;
    
end