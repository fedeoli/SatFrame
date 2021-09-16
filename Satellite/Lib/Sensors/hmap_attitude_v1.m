%% magnetometers function 
function zhat = hmap_attitude_v1(x, B, S, DynOpt, agent)

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
    % magnetometers   
    if DynOpt.ObserverTest.nMagneto == 2
        Bx1 = B(1);
        By1 = B(2);
        Bz1 = B(3);
        Bx2 = B(4);
        By2 = B(5);
        Bz2 = B(6);
    elseif DynOpt.ObserverTest.nMagneto == 1
        Bx1 = B(1);
        By1 = B(2);
        Bz1 = B(3);
        Bx2 = 0*B(1);
        By2 = 0*B(2);
        Bz2 = 0*B(3);
    else
        Bx1 = 0;
        By1 = 0;
        Bz1 = 0;
        Bx2 = 0;
        By2 = 0;
        Bz2 = 0;
    end
    
    % Sun sensor
    if (DynOpt.ObserverTest.Sun == 1) && (DynOpt.ObserverTest.Eclipse == 0)
        Sx = S(1);
        Sy = S(2);
        Sz = S(3);
    else
        Sx = 0;
        Sy = 0;
        Sz = 0;
    end

    
    %%% matrix computation %%%
    if (DynOpt.ObserverTest.Sun == 1) && (DynOpt.ObserverTest.Eclipse == 0)
        if DynOpt.ObserverTest.nMagneto == 1 
            zhat = DynOpt.sym_att(agent).hsym_att(wx, wy, wz, q0, q1, q2, q3, Bx1, By1, Bz1, Sx, Sy, Sz);
        elseif DynOpt.ObserverTest.nMagneto == 2
            zhat = DynOpt.sym_att(agent).hsym_att(wx, wy, wz, q0, q1, q2, q3, Bx1, By1, Bz1, Bx2, By2, Bz2, Sx, Sy, Sz);
        else            
            zhat = DynOpt.sym_att(agent).hsym_att(wx, wy, wz, q0, q1, q2, q3, Sx, Sy, Sz);
        end
    else
        if DynOpt.ObserverTest.nMagneto == 1             
            zhat = DynOpt.sym_att(agent).hsym_att_nosun(wx, wy, wz, q0, q1, q2, q3, Bx1, By1, Bz1);
        elseif DynOpt.ObserverTest.nMagneto == 2            
            zhat = DynOpt.sym_att(agent).hsym_att_nosun(wx, wy, wz, q0, q1, q2, q3, Bx1, By1, Bz1, Bx2, By2, Bz2);
        else            
            zhat = DynOpt.sym_att(agent).hsym_att_nosun(wx, wy, wz, q0, q1, q2, q3);
        end
    end
                
    zhat = double(zhat);
    
    %%% remove zero elements
    zhat = nonzeros(zhat);
end
                                                       

                                                       
                                                       
               