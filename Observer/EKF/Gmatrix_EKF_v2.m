%% EKF A matrix numeric %%
function G = Gmatrix_EKF_v2(DynOpt,params,x)

    %%% params %%%
    mie = params.mi;
    rx = x(1);
    ry = x(2);
    rz = x(3);
    
    
    %%% matrix computation %%%
    G = DynOpt.sym.Gsym_pos(    rx, ry, rz, ...
                                mie);
                
    G = double(G);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
end