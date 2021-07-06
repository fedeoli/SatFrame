function params = CheckCollision_V2_3(prob, prob_vs_chief, params)

% Initialize collision flag
params.CollisionFlag = 0;

% Extract number of deputies
N_deputy = params.Ndeputy;
params.colliding_deputy_chief = 0;
params.colliding_deputy = 0;

for j = 1:N_deputy
    
    if prob_vs_chief(j)*100 > params.CollisionProbabilityThreshold
        
        params.colliding_deputy_chief(1) = j;
        params.colliding_deputy = params.colliding_deputy_chief;
        params.CollisionFlag = 1;
        
    end
    
end

if params.colliding_deputy_chief == 0
    
    for j = 1:N_deputy
        
        for k = j : N_deputy - 1
            
            if prob(j,k+1)*100 > params.CollisionProbabilityThreshold
                
                params.colliding_deputy(1:2) = [j,k+1];
                params.CollisionFlag = 1;
                
            end
            
        end
        
    end
end
