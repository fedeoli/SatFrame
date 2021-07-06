function [satellites_iner_ECI, params] = CollisionAvoidanceImpulsive_V2_2(t, satellites_iner_ECI, deputy_rel_LVLH, DeltaV_inst, CollisionProb,CollisionProb_vs_chief, params)

params.CollisionFlag = 0;

if params.CollisionAvoidance == 1
    
    % Check if the collision is expected
    params = CheckCollision_V2_2(CollisionProb, CollisionProb_vs_chief, params);
    
    if params.CollisionFlag == 1 % && params.CA_happened == 0
        params.CA_happened = 1;
        %         params.ParkingTime = t + 2*pi/params.n;
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
        phi = expm(A*(params.DT_CollisionCheck));
        phi12 = phi(1:3,4:6);
        [eig_vec, eig_val] = eig(phi12'*phi12);
        
        % Compute the Delta V to have a minimum separation distance (ref. Slater, Byram, Williams)
        [lambda_max, ind_max] = max(diag(eig_val));
        DV_vers = eig_vec(:,ind_max)/norm(eig_vec(:,ind_max));
        DV = sqrt(Dmin^2/lambda_max)*DV_vers;
        
        % Select the deputy that is asked to exert the collision avoidance Delta V
        if params.colliding_sat_chief == 0
            if params.sat(params.colliding_sat(1) + 1).IsWorking == 1 && params.sat(params.colliding_sat(2) + 1).IsWorking == 1
                [minDV, j_min] = min(DeltaV_inst([params.colliding_sat(1), params.colliding_sat(2)]));
                j = params.colliding_sat(j_min);
            elseif params.sat(params.colliding_sat(1) + 1).IsWorking == 1
                j = params.colliding_sat(1);
            elseif params.sat(params.colliding_sat(2) + 1).IsWorking == 1
                j = params.colliding_sat(2);
            end
        else
            j = params.colliding_sat_chief*params.sat(params.colliding_sat(1) + 1).IsWorking;
        end
        
        % Apply the Collision Avoidance Delta V (if a deputy has been succesfully selected)
        if j > 0
            params.u_module(j+1) = norm(DV);
            params.sat(j+1).Burn_executed = 0;
            params.sat(j+1).FiniteBurnTime = norm(DV)/(params.sat(j+1).Thruster.MaxThrust/params.sat(j+1).M);
            DV_applied = DV;
            params.DV(:,round(t/params.time_step),j) = params.DV(:,round(t/params.time_step),j) + DV_applied;
            if params.Attitude
                
                params.control_dir_Hill(:,j+1) = DV./params.u_module(j+1);
                params.sat(j+1).CanManeuverAttitude = 1;
                
            elseif params.RealThruster == 0
                % DV_applied = R_Hill2Body'*[norm(DV); 0; 0];
                
                deputy_rel_LVLH(:,j) = deputy_rel_LVLH(:,j) + [zeros(3,1); DV_applied];
                
                % Compute chief COE
                chief_coe = rv2coe_V1_1(satellites_iner_ECI(1:3), satellites_iner_ECI(4:6), params.mi);
                
                % Correct j-th deputy inertial coordinates
                satellites_iner_ECI(1 + 6*j : 6 + 6*j, 1) = rel2iner_V2_2(deputy_rel_LVLH(:,j)', satellites_iner_ECI(1:6)', chief_coe, params);
            end
        end
        
    end
    
end

end
