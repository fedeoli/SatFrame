function [u, satellites_iner_ECI, params] = ControlEvaluation_V2_4(deputy_rel_LVLH, satellites_iner_ECI, t, params)

%   ControlEvaluation_V2_4.m
%   Made by Sapienza Gn Lab
%
%   Takes as input the relative position and velocity of N deputy satellites, the time instant and the required parameters in order to compute the
%   control needed to reach the reference trajectory.
%
%   INPUT
%   deputy_rel_LVLH: Array (6 x N), with N equal to the number of deputy satellites, containing each deputy's position and velocity relative components,
%                    expressed in LVLH reference frame.
%   satellites_iner_ECI: Array (6*(N+1) x 1) containing the satellites' (chief + deputies) inertial coordinates. N is the number of deputies.
%   t: time instant in which the control has to be evaluate
%   params: structure containing the following fields
%       - params.ControlStrategy = flag for the selection of the Control Strategy to be adopted (either 'Impulsive' or 'Continuous')
%       - params.CollisionAvoidance = flag for the activation of the Collision Avoidance control calculation
%       - params.DT_burn = Time interval between burns [s] (needed only for Impulsive Control)
%       - params.n = chief's orbit mean motion [rad/s]
%       - params.Control = flag for the activation of the control
%       - params.ParkingTime = time instant to which the control activation can be delayed
%       - params.WaitingTime = time interval between two subsequent impulses (needed only for Impulsive Control)
%       - params.ALQR = flag for activation of the Augmented LQR method (needed only for Continuous Control)
%       - params.FF = flag for the activation of the feedforward term. If the Augmented LQR technique is activated, this parameter is automatically
%                     set to 1 (needed only for Continuous Control)
%       - params.ReferenceTrajectory = handle to the function which computes the reference trajectory at time "t"
%       - params.tfinal = final time of the simulation [s]
%       - params.time_step = integration time step [s]
%       - params.R = LQR control tuning matrix (needed only for Continuous Control)
%       - params.Q = LQR state error tuning matrix (needed only for Continuous Control)
%       - params.B = control mapping matrix (needed only for Continuous Control)
%       - params.deputy = array of structure of size (Ndeputy x 1). Each structure must contain the following fields:
%           - deputy(j).TrajectoryProfile = handle to the function which computes the reference trajectory at time "t";
%           - all the parameters needed for correct reference trajectory calculation.
%
%   OUTPUT
%   u: Array (3 x N) containing the LQR control needed to track the reference state for each deputy.
%   satellites_iner_ECI: Array (6*(N+1) x 1) containing the satellites' inertial components.
%   params: parameter structure
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
%
%   20190405 V1_3:
%   - Different control calculation for each deputy satellite based on different reference trajectories
%
%   20190419 V1_4:
%   - The computation of the Riccati gain matrices by means of interpolation has been moved inside the function "InterpolateRiccatiGainV1_1" for the
%     sake of clarity.
%   - The "Impulsive Control Strategy" has been introduced. The flag "params.ControlStrategy" now selects either "Impulsive" or "Continuous" control
%     strategy and computes the control accordingly.
%   - The Impulsive Control Strategy approach has been included inside the function "ImpulsiveControlStrategy_V1_1".
%   - The Collision Avoidance control strategy has been added and can be selected by means of the "params.CollisionAvoidance" flag.
%
%   20190923 V1_5:
%   - Minor issues on Delta V allocation corrected
%
%   20191114 V1_6:
%   - Computation and storing of the control direction for desired attitude orientation added
%   - Computation and storing of the desired control module added
%
%   20191125 V2_1:
%   - New version adapted for the inclusion of attitude dynamics control
%
%   20200109 V2_2:
%   - Inclusion of different waiting times for different deputies in the "Impulsive" control strategy
%
%   20200110 V2_3:
%   - Check of the flag "params.Control" modified to account for the multiple waiting times included in V2_2.
%   - "params.ParkingTime" introduced to account for an initial phase of the mission in which no control is applied. It has been separated from
%     "params.WaitingTime" which now only relates to the Impulsive control strategy calculation
%   - The function has been modified to take into account that "params.deputy" has been chaged to "params.sat"
%
%   20200518 V2_4:
%   - params.sat.CA_mode introduced and used to discern whether a sat is in CA mode (and thus cannot control) or not
%   - params.sat.FiniteBrunTime & params.BurningTimes fixed and set to zero when params.RealThruster = 0

% Number of deputy satellite calculation
Ndeputy = size(deputy_rel_LVLH, 2);

% Initialization of the control output
u = zeros(3, Ndeputy);
params.DV(:,round(t/params.time_step),:) = zeros(3, 1, Ndeputy);
params.u_module = zeros(1, Ndeputy + 1);
params.BurningTimes(:,round(t/params.time_step)) = zeros(Ndeputy,1);

% Check if control is chosen to be switched on
if ( params.Control == 1 && t > params.ParkingTime )
    
    % Initialize reference trajectory
    x_ref = zeros(6, Ndeputy);
    
    for i = 1:Ndeputy
        
        % Computation of the reference trajectory at time "t"
        x_ref(:,i) = params.sat(i+1).TrajectoryProfile(t, params.sat(i+1));
        
    end
    
    if strcmpi(params.ControlStrategy, 'Impulsive')
        
        % Initialization of the Delta V output
        params.DV(:,round(t/params.time_step),:) = zeros(3,1,Ndeputy);
                
        for i = 1:Ndeputy
            
            if params.sat(i+1).CA_mode == 0
                
                if (t > params.sat(i+1).WaitingTime) && params.sat(i+1).IsWorking == 1
                    
                    % Compute the Delta V to be applied and the state updated with such Delta V
                    DV = ImpulsiveControlStrategySingle_V2_2(deputy_rel_LVLH(:,i), t, params, params.sat(i+1));
                    
                    % Calculate the burn time needed to exert the computed Delta V by means of the chosen thruster
                    params.sat(i+1).FiniteBurnTime = norm(DV)/(params.sat(i+1).Thruster.MaxThrust/params.sat(i+1).M);
                    
                    % Change the Burn_executed parameter to "0" (not finished burning yet)
                    params.sat(i+1).Burn_executed = 0;
                    
                    % Store firing module
                    params.u_module(i+1) = module_V2_1(DV);
                    
                    if params.u_module(i+1) >= params.FiringThreshold
                        
                        % Store the computed Delta V in the "params" structure
                        params.DV(:, round(t/params.time_step), i) = DV;
                        params.BurningTimes(i, round(t/params.time_step)) = params.sat(i+1).FiniteBurnTime;
                        
                        if params.Attitude
                            
                            params.control_dir_Hill(:,i+1) = DV./params.u_module(i+1);
                            params.sat(i+1).CanManeuverAttitude = 1;
                            
                        elseif params.RealThruster == 0
                            
                            % Correct the j-th deputy relative state by adding DeltaV to velocities components
                            deputy_rel_LVLH(:,i) = deputy_rel_LVLH(:,i) + [zeros(3,1); DV];
                            
                            % Compute chief COE
                            chief_coe = rv2coe_V1_1(satellites_iner_ECI(1:3), satellites_iner_ECI(4:6), params.mi);
                            
                            % Correct j-th deputy inertial coordinates
                            satellites_iner_ECI(1 + 6*i : 6 + 6*i, 1) = rel2iner_V2_t(deputy_rel_LVLH(:,i)', satellites_iner_ECI(1:6)', chief_coe, params);
                            
                            % Update Waiting Time & Burning Time
                            params.sat(i+1).WaitingTime = t + params.DT_burn;
                            params.sat(i+1).FiniteBurnTime = 0;
                            params.BurningTimes(i, round(t/params.time_step)) = 0;
                            
                        end
                        
                    else
                        
                        params.u_module(i+1) = 0;
                        params.control_dir_Hill(:,i+1) = [0; 0; 0];
                        
                    end
                    
                end
                
            end
            
        end
        
    elseif strcmpi(params.ControlStrategy, 'Continuous')
        
        % Extraction of the required constants from "params" structure
        R = params.R;
        B = params.B;
        
        if params.ALQR  % Control computed by means of the differential Riccati equation
            
            params.FF = 1;  % Include the feedrforward term if Augmented LQR technique is selected
            
            if (0 <= t - params.ParkingTime) && (t - params.ParkingTime <= params.time_step)    % The Riccati differential equation must be solved only at first time instant
                
                K = zeros(ceil((params.tfinal + params.time_step)/params.tRiccati), 36, Ndeputy);
                G = zeros(ceil((params.tfinal + params.time_step)/params.tRiccati), 6, Ndeputy);
                
                for j = 1:Ndeputy
                    
                    params.ReferenceTrajectory = params.sat(j+1);
                    
                    % Integration of the Riccati differential equation
                    [K(:,:,j), G(:,:,j)] = RiccatiDiff_V1_2(params);
                    
                end
                
                % Matrices K and G must be stored for usage in future time instants (they don't need to be calculated at each time step)
                params.K = K;
                params.G = G;
                
            end
            
            % Computation of the gain matrices at time "t" by interpolation of the Riccati differential equation solution
            [Kc, Gc] = InterpolateRiccatiGain_V1_1(t, Ndeputy, params);
            
            
        else    % Control computed by means of the algebraic Riccati equation
            
            % Extraction of the required constants from "params" structure
            n = params.n;
            Q = params.Q;
            
            % Matrix of the linearized dynamics
            A = [0   , 0, 0   , 1   , 0  , 0;
                0    , 0, 0   , 0   , 1  , 0;
                0    , 0, 0   , 0   , 0  , 1;
                3*n^2, 0, 0   , 0   , 2*n, 0;
                0    , 0, 0   , -2*n, 0  , 0;
                0    , 0, -n^2, 0   , 0  , 0];
            
            for i = 1:Ndeputy
                
                % Gain Matrix calculation through LQR method
                Kc(:,:,i) = care(A, B, Q, R);
                
                % Computation of the feedforward gain matrix
                Gc(:,i) = inv(Kc(:,:,i)*B*inv(R)*B' - A')*Q*x_ref(:,i);
                
            end
            
        end
        
        % Computation of the closed-loop control with LQR gain matrix
        for i = 1:Ndeputy
            
            if params.sat(i+1).CA_mode == 0
                
                if params.twoAxesControl
                    
                    if params.FF
                        
                        u(2:3,i) = R\B'*(Gc(:,i) - Kc(:,:,i)*deputy_rel_LVLH(:,i));
                        
                    else
                        
                        u(2:3,i) = -R\B'*Kc(:,:,i)*(deputy_rel_LVLH(:,i) - x_ref(:,i));
                        
                    end
                    
                else
                    
                    if params.FF
                        
                        u(:,i) = R\B'*(Gc(:,i) - Kc(:,:,i)*deputy_rel_LVLH(:,i));
                        
                    else
                        
                        u(:,i) = -R\B'*Kc(:,:,i)*(deputy_rel_LVLH(:,i) - x_ref(:,i));
                        
                    end
                    
                end
                
            end
            
        end
        
        % Computation of the Collision Avoidance Potential control for each deputy
        if params.CollisionAvoidance && strcmpi(params.CollisionAvoidanceStrategy, 'Potential')
            
            % Collision avoidance control array storage
            u_CA = zeros(3,Ndeputy);
            
            for i = 1:Ndeputy
                
                % Computation of the Collision Avoidance control for the i-th deputy
                u_CA(:,i) = CollisionAvoidancePotential_V1_1(deputy_rel_LVLH, i);
                
                % Collision Avoidance control addition to maneuver control already computed
                u(:,i) = u(:,i) + u_CA(:,i);
                
            end
            
        end
        
        % Store firing direction
        for i = 1:Ndeputy
            
            params.u_module(i+1) = module_V2_1(u(:,i));
            
            if params.u_module(i+1) > params.FiringThreshold
                
                params.control_dir_Hill(:,i+1) = u(:,i)./params.u_module(i+1);
                
            else
                
                params.u_module(i+1) = 0;
                params.control_dir_Hill(:,i+1) = [0; 0; 0];
                
            end
            
        end
        
    end
    
end

end