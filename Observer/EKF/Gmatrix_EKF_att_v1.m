%% EKF A matrix numeric %%
function G = Gmatrix_EKF_att_v1(DynOpt,params,x,agent)

    %%% params %%%
    q0 = x(1);
    q1 = x(2);
    q2 = x(3);
    q3 = x(4);

    wx = x(5);
    wy = x(6);
    wz = x(7);
    
    Ixx = params.sat(agent).I(1,1);
    Iyy = params.sat(agent).I(2,2);
    Izz = params.sat(agent).I(3,3);
    
    
    %%% matrix computation %%%
    G = DynOpt.sym_att(agent).Gsym_att( Ixx, Iyy, Izz, ...
                                    wx, wy, wz, ...
                                    q0, q1, q2, q3);
                
    G = double(G);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
end