%% EKF A matrix numeric %%
function H = Hmatrix_EKF_v3(DynOpt,x,j_select)
    
    % nagents
    nagent = DynOpt.ObserverTest.Nagents;
    theta = DynOpt.ObserverTest.theta;
    
    % adjmat UWB
    adj_UWB = DynOpt.y_UWB;
    
    % adjmat Chi/GPS
    x_pos = x(DynOpt.ObserverTest.PositionArrayIndex);
    adj = setAdjacencyMatrixNorm(x_pos,nagent);
    
    %%%%%%%%%%%%%%%% position %%%%%%%%%%%%%%%%
    % current state
    x_j = x(1+(j_select-1)*6:3+(j_select-1)*6);
    
    % position - summation term
    sum_term = 0;
    for i=1:nagent
       if i~=j_select
            % neighbour state
            x_i = x(1+(i-1)*6:3+(i-1)*6);
            
            % Delta
            Delta =  x_j -x_i;
            
            % distances
            dij = adj(j_select,i);
            dij_UWB = adj_UWB(j_select,i);
            
            % summation term
            tmp_1 = (transpose(Delta)*Delta - dij_UWB*norm(Delta) + Delta)./(2*transpose(Delta)*Delta);
            tmp_2 = 2*Delta.^2*(norm(Delta) - dij_UWB)./(transpose(Delta)*Delta*norm(Delta));
            tmp = 1 + tmp_1 - tmp_2;
       else
            tmp = 0;
       end
       
       sum_term = sum_term + tmp;
    end
    
    % position derivative
    dpdx = (1-theta)/(nagent-1)*sum_term + theta;
    
    % final computation   
    h = [dpdx; ones(3,1)];
    H = diag(h);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
end