function params = CollisionAvoidanceStatusCheck_V2_1(params)

% Compute the index of the satellite maneuvering in CA mode
CA_Man = [params.sat.CA_Maneuvering];
idx_CA_Man = find(CA_Man == 1);

% Check if the Holding Collision Avoidance maneuver has been applied or is undergoing and update Collision Holding flag accordingly
if params.CollisionHolding && ( ( params.sat(idx_CA_Man).IsBurning && params.sat(idx_CA_Man).Burn_executed == 0 ) || ( params.sat(idx_CA_Man).IsBurning == 0 && params.sat(idx_CA_Man).Burn_executed == 1 ) )
    
    params.CollisionHolding = 0;
    
end

% Check the Collision Avoidance status of each satellite and update the CA Residual Time if needed
for i = 1:params.Ndeputy
    
    if params.sat(i+1).CA_mode && ( ( params.sat(idx_CA_Man).Burn_executed == 0 && params.sat(idx_CA_Man).IsBurning == 1 ) || ( params.sat(idx_CA_Man).Burn_executed == 1 && params.sat(idx_CA_Man).IsBurning == 0 ) )
        
        if params.sat(i+1).CA_ResidualTime > 0
            
            params.sat(i+1).CA_ResidualTime = params.sat(i+1).CA_ResidualTime - params.time_step;
            
        elseif params.sat(i+1).CA_ResidualTime <= 0
            
            params.sat(i+1).CA_mode = 0;
            params.sat(i+1).CA_Maneuvering = 0;
            
        end
        
    end
    
end

end