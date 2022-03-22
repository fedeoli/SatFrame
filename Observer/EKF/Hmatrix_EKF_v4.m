%% EKF A matrix numeric %%
function H = Hmatrix_EKF_v4(DynOpt,x,j_select)
    
    % nagents
    nagent = DynOpt.ObserverTest.Nagents;
    theta = DynOpt.ObserverTest.theta;
    
    % adjmat UWB
    adj_UWB = DynOpt.y_UWB;
    
    % Chi/GPS
    x_pos = x(DynOpt.ObserverTest.PositionArrayIndex);
    
    %%%%%%%%%%%%%%%% position %%%%%%%%%%%%%%%%
    % current state
    x_i = [];
    x_j = [];
    for j=1:nagent
        if j~=j_select
            x_j = [x_j; x_pos(1+(j-1)*3:3+(j-1)*3)];
        else
            tmp = reshape(x_pos(1+(j-1)*3:3+(j-1)*3),3,1);
            x_i = tmp;
        end
    end
    
    % distances
    dij_UWB = nonzeros(transpose(adj_UWB(j_select,:)));
    
    % gps
    GPS = reshape(DynOpt.y_GPS(1+6*(j_select-1):3+6*(j_select-1),DynOpt.iter),3,1);
    
    % state
    
    state = [x_i; x_j; GPS; dij_UWB];
    
    % position derivative
    if DynOpt.ObserverTest.EKF_withRD
        H = Hmatrix_EKF_withRD(DynOpt,x,j_select);
    else
        H = double(DynOpt.sym.H_pos);
    end
    if 1 && DynOpt.iter > DynOpt.ObserverTest.UWBOptimizationNoBeforeThan && nagent > 1 && DynOpt.ObserverTest.GPSopt_flag
        dpdx = DynOpt.KF(j_select).J(1:3,1:3);
        H(1:3,1:3) = dpdx;
    end
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
end