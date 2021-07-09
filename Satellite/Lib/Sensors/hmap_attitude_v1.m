%% magnetometers function 
function zhat = hmap_attitude_v1(x, B, DynOpt, agent)

    % quaternion
    q0 = x(1);
    q1 = x(2);
    q2 = x(3);
    q3 = x(4);
    
    % omega
    wx = x(5);
    wy = x(6);
    wz = x(7);
   
    
    % magnetometers   
    Bx1 = B(1);
    By1 = B(2);
    Bz1 = B(3);
    Bx2 = B(1);
    By2 = B(2);
    Bz2 = B(3);

    
    %%% matrix computation %%%
    zhat = DynOpt.sym_att(agent).hsym_att( wx, wy, wz, ...
                                        q0, q1, q2, q3, ...
                                        Bx1, By1, Bz1, ...
                                        Bx2, By2, Bz2 );
                
    zhat = double(zhat);
end
                                                       

                                                       
                                                       
               