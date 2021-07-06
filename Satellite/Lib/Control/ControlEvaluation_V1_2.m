function [u, params] = ControlEvaluation_V1_2(deputy_rel_LVLH, t, params)

%   ControlEvaluation_V1_2.m
%   Made by Sapienza Gn Lab
%
%   Takes as input the relative position and velocity of N deputy satellites, the time instant and the required parameters in order to compute the closed-loop 
%   control needed to reach the reference trajectory.
%
%   INPUT
%   deputy_rel_LVLH: Array (6 x N), with N equal to the number of deputy satellites, containing each deputy's position and velocity relative components,
%                    expressed in LVLH reference frame.
%   t: time instant in which the control has to be evaluate
%   params: structure containing the following fields
%       - params.n = chief's orbit mean motion [rad/s]
%       - params.Control = flag for the activation of the control
%       - params.WaitingTime = time instant to which the control activation can be delayed
%       - params.ALQR = flag for activation of the Augmented LQR method
%       - params.FF = flag for the activation of the feedforward term. If the Augmented LQR technique is activated, this parameter is automatically
%                     set to 1
%       - params.ReferenceTrajectory = handle to the function which computes the reference trajectory at time "t"
%       - params.time_step = integration time step [s]
%       - params.tRiccati = Riccati differential equation integration time step [s]
%       - params.R = LQR control tuning matrix
%       - params.Q = LQR state error tuning matrix
%       - params.B = control mapping matrix
%
%   OUTPUT
%   u: Array (3 x N) containing the LQR control needed to track the reference state for each deputy.
%
%   VERSION
%   20190312 V1_1:
%   -  First Release
%
%   20190326 V1_2:
%   - The possibility to compute the control with the Augmented LQR method has been added. Can be activated through the "params.ALQR" flag.
%   - Evaluation of the Pulse Width Modulation (PWM) has been added. Can be activated thtrough the "params.PWM" flag.
%   - The feedforward term (vector "Gc") can be added for the calculation of the control profile through the "params.FF" flag. Please note that, if
%     Augmented LQR has been selected, the feedforward term is included in control evaluation, regardless of the value assigned to "params.FF".
%   - The flag "params.WaitingTime" has been moved from the Main inside this function for the sake of clarity. By setting a certain value, the
%     application of the control can be delayed to that time.
%   - The reference trajectory is now computed inside this function of the sake of clarity. The computation is done through the function whose handle
%     has been assigned to "params.ReferenceTrajectory". Please see the "Analysis" file to set the desired reference trajectory.
%   - The Riccati differential equation is being solved (at first time instant and if the "params.ALQR" flag is activated) inside this function for
%     the sake of clarity.


% Number of deputy satellite calculation
N = size(deputy_rel_LVLH, 2);
        
% Initialization of the control matrix
u = zeros(3, N);

if params.Control == 0
    
    params.WaitingTime = Inf;
    
end
    
    
if t > params.WaitingTime
    
    % Extraction of the required constants from "params" structure
    R = params.R;
    B = params.B;
    
    % Computation of the reference trajectory at time "t"
    x_ref = params.ReferenceTrajectory(t, params);
    
    if params.ALQR  % Control computed by means of the differential Riccati equation
        
        params.FF = 1;  % Include the feedrforward term if Augmented LQR technique is selected
        
        if t - params.time_step == 0    % The Riccati differential equation must be solved only at first time instant
            
            % Integration of the Riccati differential equation
            [K, G] = RiccatiDiff_V1_1(params);
            
            % Matrices K and G must be allocated for usage in future time instants
            params.K = K;
            params.G = G;
            
        end
        
        RiccatiStep = params.tRiccati;
        PrevIndex = floor(t/RiccatiStep) + 1;
        SucIndex = floor(t/RiccatiStep) + 2;
        
        % Store gain matrices in the "params" structure (they don't need to be calculated at each time step)
        K = params.K;
        G = params.G;
        
        if (SucIndex < max(size(K)))
            
            t_prec = (PrevIndex - 1)*RiccatiStep;
            t_suc = (SucIndex - 1)*RiccatiStep;
            Kvet = K(PrevIndex,:);
            K_prec = [Kvet(1:6); Kvet(7:12); Kvet(13:18); Kvet(19:24); Kvet(25:30); Kvet(31:36)];
            Kvet = K(SucIndex,:);
            K_suc = [Kvet(1:6); Kvet(7:12); Kvet(13:18); Kvet(19:24); Kvet(25:30); Kvet(31:36)];
            Kc = (K_suc - K_prec)/(t_suc - t_prec)*t + (K_prec*t_suc - K_suc*t_prec)/(t_suc - t_prec);
            G_prec = G(PrevIndex,:)';
            G_suc = G(SucIndex,:)';
            Gc = (G_suc - G_prec)/(t_suc - t_prec)*t + (G_prec*t_suc - G_suc*t_prec)/(t_suc - t_prec);
            
        else
            
            Kvet = K(PrevIndex,:);
            Kc = [Kvet(1:6); Kvet(7:12); Kvet(13:18); Kvet(19:24); Kvet(25:30); Kvet(31:36)];
            Gc = G(PrevIndex,:)';
            
        end
        
    else    % Control computed by means of the algebraic Riccati equation
        
        % Extraction of the required constants from "params" structure
        n = params.n;
        Q = params.Q;
        
        % Matrix of the linearized dynamics
        A = [0    , 0, 0   , 1   , 0  , 0;
            0    , 0, 0   , 0   , 1  , 0;
            0    , 0, 0   , 0   , 0  , 1;
            3*n^2, 0, 0   , 0   , 2*n, 0;
            0    , 0, 0   , -2*n, 0  , 0;
            0    , 0, -n^2, 0   , 0  , 0];
        
        % Gain Matrix calculation through LQR mathod
        Kc = care(A, B, Q, R);
        
        % Computation of the feedforward gain matrix
        Gc = inv(Kc*B*inv(R)*B' - A')*Q*x_ref;
        
    end
    
    % Computation of the closed-loop control with LQR gain matrix
    for i = 1:N
        
        
        if params.twoAxesControl
            
            if params.FF
                
                u(2:3,i) = R\B'*(Gc - Kc*deputy_rel_LVLH(:,i));
                
            else
                
                u(2:3,i) = -R\B'*Kc*(deputy_rel_LVLH(:,i) - x_ref);
                
            end
            
        else
            
            if params.FF
                
                u(:,i) = R\B'*(Gc - Kc*deputy_rel_LVLH(:,i));
                
            else
                
                u(:,i) = -R\B'*Kc*(deputy_rel_LVLH(:,i) - x_ref);
                
            end
            
        end
        
        if params.PWM   % Sampling of the control by means of the Pulse Width Modulation
            
            u(1,i)  = PWM_V1_1(t, u(1,i), params);
            u(2,i)  = PWM_V1_1(t, u(2,i), params);
            u(3,i)  = PWM_V1_1(t, u(3,i), params);
            
        end
        
        
    end
    
end


end