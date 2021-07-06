function params = CheckCollision_V2_2(prob, prob_vs_chief, params)

% Initialize collision flag
params.CollisionFlag = 0;

% Extract number of deputies
N_deputy = params.Ndeputy;
params.colliding_sat_chief = 0;
params.colliding_sat = 0;

for j = 1:N_deputy
    
    if prob_vs_chief(j)*100 > params.CollisionProbabilityThreshold
        
        params.colliding_sat_chief(1) = j;
        params.colliding_sat = params.colliding_sat_chief;
        params.CollisionFlag = 1;
        
    end
    
end

if params.colliding_sat_chief == 0
    
    for j = 1:N_deputy
        
        for k = j : N_deputy - 1
            
            if prob(j,k+1)*100 > params.CollisionProbabilityThreshold
                
                params.colliding_sat(1:2) = [j,k+1];
                params.CollisionFlag = 1;
                
            end
            
        end
        
    end
end
