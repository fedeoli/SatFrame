function [satellites_iner_ECI, params] = CollisionAvoidanceManeuver_V2_2(t, satellites_iner_ECI, deputy_rel_LVLH, CollisionProb,CollisionProb_vs_chief, params)

% Enter the Collision Avoidance procedure if the flag is selected
if params.CollisionAvoidance == 1 && ( strcmpi(params.CollisionAvoidanceStrategy, 'SafetyImpulse') || strcmpi(params.CollisionAvoidanceStrategy, 'SafetyDrift') || strcmpi(params.CollisionAvoidanceStrategy, 'SafetyParking') )
        
    % Check if the collision is expected
    params = CheckCollision_V2_3(CollisionProb, CollisionProb_vs_chief, params);
        
    if params.CollisionFlag == 1
        
        % Reset Hold Collision Avoidance data (if a collision was in holding mode before or if a burning had not been completed)
        params.CollisionHolding = 0;
        new_CA = num2cell(zeros(1, params.Ndeputy + 1));
        [params.sat.CA_mode] = new_CA{:};
        [params.sat.CA_Maneuvering] = new_CA{:};
        
        % Extract quantities from "params" structure
        n = params.n;
        Dmin = params.MinimumSeparationDistance;
        
        % Initialize the deputy that is going to exert the collision avoidance Delta V
        j = 0;
        
        % Matrix of the linearized dynamics
        A = [0   , 0, 0   , 1   , 0  , 0;
            0    , 0, 0   , 0   , 1  , 0;
            0    , 0, 0   , 0   , 0  , 1;
            3*n^2, 0, 0   , 0   , 2*n, 0;
            0    , 0, 0   , -2*n, 0  , 0;
            0    , 0, -n^2, 0   , 0  , 0];
        
        % Transition Matrix
        phi = expm(A*(params.CollisionTime));
        phi12 = phi(1:3,4:6);
        
        if strcmpi(params.CollisionAvoidanceStrategy, 'SafetyImpulse') || strcmpi(params.CollisionAvoidanceStrategy, 'SafetyDrift')
            
            [eig_vec, eig_val] = eig(phi12'*phi12);
            
            % Compute the Delta V to have a minimum separation distance (ref. Slater, Byram, Williams)
            [lambda_max, ind_max] = max(diag(eig_val));
            DV_vers = eig_vec(:,ind_max)/norm(eig_vec(:,ind_max));
            DV_CA = sqrt(Dmin^2/lambda_max)*DV_vers;
            
            % Select the deputy that is asked to exert the collision avoidance Delta V
            if params.colliding_deputy_chief == 0
                
                if params.sat(params.colliding_deputy(1) + 1).IsWorking == 1 && params.sat(params.colliding_deputy(2) + 1).IsWorking == 1
                    
                    [~, j_min] = min([params.sat(params.colliding_deputy(1) + 1).Burnt_DV, params.sat(params.colliding_deputy(2) + 1).Burnt_DV]);
                    j = params.colliding_deputy(j_min);
                    
                elseif params.sat(params.colliding_deputy(1) + 1).IsWorking == 1
                    
                    j = params.colliding_deputy(1);
                    
                elseif params.sat(params.colliding_deputy(2) + 1).IsWorking == 1
                    
                    j = params.colliding_deputy(2);
                    
                end
                
            else
                
                j = params.colliding_deputy_chief*params.sat(params.colliding_deputy(1) + 1).IsWorking;
                
            end
            
        elseif strcmpi(params.CollisionAvoidanceStrategy, 'SafetyParking')
            
            phi11 = phi(1:3,1:3);
            phi21 = phi(4:6,1:3);
            phi22 = phi(4:6,4:6);
            
            fun = @(x)quadobj(x);
            H1 = [0 1 0]; H2 = [1 0 0];
            
            PHI = phi12'*phi12;
            nonlconstr = @(x)quadconstr(x,PHI,Dmin);
            
            options = optimoptions(@fmincon,'Algorithm','interior-point','SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true, 'HessianFcn',@(x,lambda)quadhess(x,lambda,PHI),...
                'TypicalX',1e-3*ones(3,1),'OptimalityTolerance',1e-9,'MaxIterations',100000,'MaxFunctionEvaluations',100000);
            x0 = 1e-4*ones(3,1);
            Aeq1 = H1*phi22 + 2*n*H2*phi12;
            Aeq2 = H2*phi22 - n/2*H1*phi12;
            Aeq = [Aeq1; Aeq2];
            rho = deputy_rel_LVLH(1:3, params.colliding_deputy(1)) ;
            rho_dot = deputy_rel_LVLH(4:6, params.colliding_deputy(1)) ;
            beq1 = -H1*phi21*rho - H1*phi22*rho_dot -2*n*H2*phi11*rho - 2*n*H2*phi12*rho_dot;
            CoEllipse = 1*n/2;
            beq2 = -H2*phi21*rho - H2*phi22*rho_dot +n/2*H1*phi11*rho + n/2*H1*phi12*rho_dot - CoEllipse;
            beq = [beq1; beq2];
            DV_opt = zeros(3,2);
            DV_opt(:,1) = fmincon(fun, x0, [], [], Aeq, beq, [], [], nonlconstr, options);
            
            if params.colliding_deputy_chief == 0
                
                rho = deputy_rel_LVLH(1:3, params.colliding_deputy(2)) ;
                rho_dot = deputy_rel_LVLH(4:6 ,params.colliding_deputy(2)) ;
                beq1 = -H1*phi21*rho - H1*phi22*rho_dot -2*n*H2*phi11*rho - 2*n*H2*phi12*rho_dot;
                beq2 = -H2*phi21*rho - H2*phi22*rho_dot +n/2*H1*phi11*rho + n/2*H1*phi12*rho_dot - CoEllipse;
                beq = [beq1; beq2];
                x0 = 1e-4*ones(3,1);
                DV_opt(:,2) = fmincon(fun, x0, [], [], Aeq, beq, [], [], nonlconstr, options);
                
                % Select the deputy that is asked to exert the collision avoidance Delta V
                if params.sat(params.colliding_deputy(1) + 1).IsWorking == 1 && params.sat(params.colliding_deputy(2) + 1).IsWorking == 1
                    
                    [~, j_min] = min([norm(DV_opt(:,1)), norm(DV_opt(:,2))]);
                    j = params.colliding_deputy(j_min);
                    DV_CA = DV_opt(:,j_min);
                    
                elseif params.sat(params.colliding_deputy(1) + 1).IsWorking == 1
                    
                    j = params.colliding_deputy(1);
                    DV_CA = DV_opt(:,1);
                    
                elseif params.sat(params.colliding_deputy(2) + 1).IsWorking == 1
                    
                    j = params.colliding_deputy(2);
                    DV_CA = DV_opt(:,2);
                    
                end
                
            else
                
                DV_CA = DV_opt(:,1);
                j = params.colliding_deputy_chief*params.sat(params.colliding_deputy(1) + 1).IsWorking;
                
            end
            
        end
        
        % Apply the Collision Avoidance Delta V (if a deputy has been succesfully selected)
        if j > 0
            
            params.sat(j+1).CA_Maneuvering = 1;
            params.DV(:,round(t/params.time_step),j) = DV_CA;
            params.u_module(j+1) = norm(DV_CA);
            params.sat(j+1).FiniteBurnTime = norm(DV_CA)/(params.sat(j+1).Thruster.MaxThrust/params.sat(j+1).M);
            params.BurningTimes(j, round(t/params.time_step)) = params.sat(j+1).FiniteBurnTime;
            params.sat(j+1).CA_DV = DV_CA;
            params.sat(j+1).Burn_executed = 0;
            
            if params.Attitude
                
                params.control_dir_Hill(:,j+1) = DV_CA./params.u_module(j+1);
                params.sat(j+1).CanManeuverAttitude = 1;
                params.CollisionHolding = 1;

            elseif params.RealThruster == 0
                
                deputy_rel_LVLH(:,j) = deputy_rel_LVLH(:,j) + [zeros(3,1); DV_CA];
                
                % Compute chief COE
                chief_coe = rv2coe_V1_1(satellites_iner_ECI(1:3), satellites_iner_ECI(4:6), params.mi);
                
                % Correct j-th deputy inertial coordinates
                satellites_iner_ECI(1 + 6*j : 6 + 6*j, 1) = rel2iner_V2_t(deputy_rel_LVLH(:,j)', satellites_iner_ECI(1:6)', chief_coe, params);
                
                % Update Burning Time
                params.sat(j+1).FiniteBurnTime = 0;
                params.BurningTimes(j, round(t/params.time_step)) = 0;
                
                % Tell that the burn has been executed
                params.sat(j+1).Burn_executed = 1;
                
            end
            
            if strcmpi(params.CollisionAvoidanceStrategy, 'SafetyImpulse')
                
                CA_ResidualTime = params.sat(j+1).FiniteBurnTime;
                
            else
                
                CA_ResidualTime = params.CA_Time;
                
            end
            
            
            if params.CA_GlobalSafeMode
                
                new_CA = num2cell([0, ones(1, params.Ndeputy)]);
                [params.sat.CA_mode] = new_CA{:};
                new_ParkingTime = num2cell([0, CA_ResidualTime*ones(1, params.Ndeputy)]);
                [params.sat.CA_ResidualTime] = new_ParkingTime{:};
                
            else
                
                params.sat(j+1).CA_mode = 1;
                params.sat(j+1).CA_ResidualTime = CA_ResidualTime;
                
            end
            
        end
       
    elseif params.CollisionHolding
        
        % Compute the index of the satellite maneuvering in CA mode
        CA_Man = [params.sat.CA_Maneuvering];
        idx_CA_Man = find(CA_Man == 1);
        
        DV_CA = params.sat(idx_CA_Man).CA_DV;
        params.DV(:,round(t/params.time_step), idx_CA_Man - 1) = DV_CA;
        params.u_module(idx_CA_Man) = norm(DV_CA);
        params.sat(idx_CA_Man).FiniteBurnTime = norm(DV_CA)/(params.sat(idx_CA_Man).Thruster.MaxThrust/params.sat(idx_CA_Man).M);
        params.BurningTimes(idx_CA_Man-1, round(t/params.time_step)) = params.sat(idx_CA_Man).FiniteBurnTime;
        params.control_dir_Hill(:,idx_CA_Man) = DV_CA./params.u_module(idx_CA_Man);
                    
    end
    
end

end