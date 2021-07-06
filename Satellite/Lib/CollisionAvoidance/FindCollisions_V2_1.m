function params = FindCollisions_V2_1(prob, prob_vs_chief, params)

% Initialize collision flag
params.CollisionFlag = 0;

% Extract number of deputies
params.colliding_sat_chief = 0;
params.colliding_sat = 0;


%%%%%   Collisions with Chief   %%%%%

% Find number of collisions with chief
params.ChiefCollisions = sum(prob_vs_chief*100 > params.CollisionProbabilityThreshold);

if params.ChiefCollisions > 0
    
    % Declare that there are collisions
    params.CollisionFlag = 1;
    
    % Find deputies colliding with chief
    params.colliding_sat_chief = find(prob_vs_chief*100 > params.CollisionProbabilityThreshold);
    
end


%%%%%   Collisions between deputies   %%%%%

% Extract upper triangular matrix
prob_triu = triu(prob);

% Find number of collisions
params.DeputyCollisions = sum(sum(prob_triu*100 > params.CollisionProbabilityThreshold));

if params.DeputyCollisions > 0
    
    % Declare that there are collisions
    params.CollisionFlag = 1;
    
    % Find indices of colliding sat
    [s1, s2] = find(prob_triu*100 > params.CollisionProbabilityThreshold);
    params.colliding_sat = [s1, s2];
   
end

end
