deputy_rel_LVLH_alltimes(:,i+1,:) = reshape(deputy_rel_LVLH, [6, 1, N_deputy]);        % Allocation of deputy's relative coordinates with the following order: [relative coordinates] x [times] x [number of deputys]
u_out(:,i+1,:) = reshape(u, [3, 1, N_deputy]);                                         % Allocation of the control
chief_coord(1:6,i) = satellites_iner_ECI(1:6);

for j = 1:N_deputy
    
    % Control module calculation
    u_module(j,i+1) = norm(u(:,j));
    
    % Istantaneous Delta V calculation
    if strcmpi(params.ControlStrategy, 'Continuous')
        
        DeltaV_inst(j,i+1) = mean(u_module(j,1:i+1))*time(i);
        
    elseif strcmpi(params.ControlStrategy, 'Impulsive')
        
        DeltaV_inst(j,i+1) = DeltaV_inst(j,i) + norm(params.DV(:,i,j));
        
    end
    
    params.sat(j+1).Burnt_DV = DeltaV_inst(j,i+1);
    
    % Error between measured and reference trajectory
    params.ReferenceTrajectory = params.sat(j+1);
    [error(:,i+1,j), error_norm(j,i+1)] = ErrorCalculation_V1_2(deputy_rel_LVLH(:,j), t, params);
    
end