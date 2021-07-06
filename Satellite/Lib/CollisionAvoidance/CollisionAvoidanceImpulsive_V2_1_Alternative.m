function [satellites_iner_ECI, params] = CollisionAvoidanceImpulsive_V2_1_Alternative(t, satellites_iner_ECI, deputy_rel_LVLH, DeltaV_inst, CollisionProb,CollisionProb_vs_chief, params)

params.CollisionFlag = 0;

if params.CollisionAvoidance == 1
    
    % Check if the collision is expected
    params = CheckCollision_V2_2(CollisionProb, CollisionProb_vs_chief, params);
    
    if params.CollisionFlag == 1

        %           params.ParkingTime = t + 2*pi/params.n;
        % Extract quantities from "params" structure
        n = params.n;
        Dmin = params.MinimumSeparationDistance;
        
        % Initialize the deputy that is going to exert the collision avoidanc Delta V
        j = 0;
        
        % Matrix of the linearized dynamics
        A = [0   , 0, 0   , 1   , 0  , 0;
            0    , 0, 0   , 0   , 1  , 0;
            0    , 0, 0   , 0   , 0  , 1;
            3*n^2, 0, 0   , 0   , 2*n, 0;
            0    , 0, 0   , -2*n, 0  , 0;
            0    , 0, -n^2, 0   , 0  , 0];
        
        % Transition Matrix
%         phi = expm(A*(params.DT_CollisionCheck));
        phi = expm(A*(params.CollisionTime));
        phi12 = phi(1:3,4:6);
        phi11 = phi(1:3,1:3);
        phi21 = phi(4:6,1:3);
        phi22 = phi(4:6,4:6);
        [eig_vec, eig_val] = eig(phi12'*phi12);
        
        % Compute the Delta V to have a minimum separation distance (ref. Slater, Byram, Williams)
        [lambda_max, ind_max] = max(diag(eig_val));
        DV_vers = eig_vec(:,ind_max)/norm(eig_vec(:,ind_max));
        DV = sqrt(Dmin^2/lambda_max)*DV_vers;
        
        fun = @(x)quadobj(x);
        H1 = [0 1 0]; H2 = [1 0 0];
        
        PHI = phi12'*phi12;
        nonlconstr = @(x)quadconstr(x,PHI,Dmin);
        
        options = optimoptions(@fmincon,'Algorithm','interior-point','SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true, 'HessianFcn',@(x,lambda)quadhess(x,lambda,PHI),...
            'TypicalX',1e-3*ones(3,1),'OptimalityTolerance',1e-9,'MaxIterations',100000,'MaxFunctionEvaluations',100000);
        x0 = 1e-4*ones(3,1); % column vector
        tic
        Aeq1 = H1*phi22 +2*n*H2*phi12;
        Aeq2 = H2*phi22 - n/2*H1*phi12;
        Aeq = [Aeq1; Aeq2];
        rho = deputy_rel_LVLH(1:3 ,params.colliding_sat(1)) ;
        rho_dot = deputy_rel_LVLH(4:6 ,params.colliding_sat(1)) ;
        beq1 = -H1*phi21*rho - H1*phi22*rho_dot -2*n*H2*phi11*rho - 2*n*H2*phi12*rho_dot;
        CoEllipse = 1*n/2;
        
        beq2 = -H2*phi21*rho - H2*phi22*rho_dot +n/2*H1*phi11*rho + n/2*H1*phi12*rho_dot - CoEllipse;
        beq = [beq1; beq2];
        DV_opt = zeros(3,2);
        [DV_opt(:,1),fval,eflag,output,lambda] = fmincon(fun,x0,[],[],Aeq,beq,[],[],nonlconstr,options);
        toc
        tic
        if params.colliding_sat_chief == 0
            rho = deputy_rel_LVLH(1:3 ,params.colliding_sat(2)) ;
            rho_dot = deputy_rel_LVLH(4:6 ,params.colliding_sat(2)) ;
            beq1 = -H1*phi21*rho - H1*phi22*rho_dot -2*n*H2*phi11*rho - 2*n*H2*phi12*rho_dot;
            beq2 = -H2*phi21*rho - H2*phi22*rho_dot +n/2*H1*phi11*rho + n/2*H1*phi12*rho_dot - CoEllipse;
            beq = [beq1; beq2];
            x0 = 1e-4*ones(3,1);
            [DV_opt(:,2),fval,eflag,output,lambda] = fmincon(fun,x0,[],[],Aeq,beq,[],[],nonlconstr,options);
            toc
            %         j = params.colliding_sat(1);
            %         % Select the deputy that is asked to exert the collision avoidance Delta V
            if params.sat(params.colliding_sat(1) + 1).IsWorking == 1 && params.sat(params.colliding_sat(2) + 1).IsWorking == 1
                [minDV, j_min] =min([norm(DV_opt(:,1)),norm(DV_opt(:,2))]);
                j = params.colliding_sat(j_min);
                DV = DV_opt(:,j_min);
            elseif params.sat(params.colliding_sat(1) + 1).IsWorking == 1
                j = params.colliding_sat(1);
                DV = DV_opt(:,1);
            elseif params.sat(params.colliding_sat(2) + 1).IsWorking == 1
                j = params.colliding_sat(2);
                DV = DV_opt(:,2);
            end
        else
            DV = DV_opt(:,1);
            j = params.colliding_sat_chief*params.sat(params.colliding_sat(1) + 1).IsWorking;
        end
      
        % Apply the Collision Avoidance Delta V (if a deputy has been succesfully selected)
        if j > 0
            
            if params.CA_GlobalSafeMode
                
                new_CA = num2cell([0, ones(1, params.Ndeputy)]);
                [params.sat.CA_mode] = new_CA{:};
                
            else
                
                params.sat(j+1).CA_mode = 1;
                
            end
            
            params.u_module(j+1) = norm(DV);
            params.sat(j+1).Burn_executed = 0;
            params.sat(j+1).FiniteBurnTime = norm(DV)/(params.sat(j+1).Thruster.MaxThrust/params.sat(j+1).M);
            DV_applied = DV;
            params.DV(:,round(t/params.time_step),j) = params.DV(:,round(t/params.time_step),j) + DV_applied;
            if params.Attitude
                
                params.control_dir_Hill(:,j+1) = DV./params.u_module(j+1);
                params.sat(j+1).CanManeuverAttitude = 1;
                
            elseif params.RealThruster == 0
                
                
                deputy_rel_LVLH(:,j) = deputy_rel_LVLH(:,j) + [zeros(3,1); DV_applied];
                
                % Compute chief COE
                chief_coe = rv2coe_V1_1(satellites_iner_ECI(1:3), satellites_iner_ECI(4:6), params.mi);
                
                % Correct j-th deputy inertial coordinates
                satellites_iner_ECI(1 + 6*j : 6 + 6*j, 1) = rel2iner_V2_2(deputy_rel_LVLH(:,j)', satellites_iner_ECI(1:6)', chief_coe, params);
            end
            
        end
        
    end
    
end

%
%

