%% EKF A matrix numeric %%
function G = Gmatrix_EKF_v2(DynOpt,params,x)

    %%% params %%%
    mie = params.mi;
    rx = x(1);
    ry = x(2);
    rz = x(3);

    vx = x(4);
    vy = x(5);
    vz = x(6);
    
    
    %%% matrix computation %%%
    G = DynOpt.sym.Gsym_pos(    rx, ry, rz, ...
                                vx, vy, vz, ...
                                mie);
                
    G = double(G);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
end