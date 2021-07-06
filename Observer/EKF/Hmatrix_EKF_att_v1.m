%% EKF A matrix numeric %%
function H = Hmatrix_EKF_att_v1(DynOpt,params,x,z,agent)

    % quaternion
    q0 = x(1);
    q1 = x(2);
    q2 = x(3);
    q3 = x(4);
    
    dcm_v1 = quat2dcm(transpose(x(1:4)));
    
    % omega
    wx = x(5);
    wy = x(6);
    wz = x(7);
   
    
    % magnetometers    
    if DynOpt.ObserverTest.nMagneto >= 1
        % first magnetometer
        z_body1 = z(1:3);
             
        MagECI_1 = pinv(dcm_v1)*z_body1;
        Bx1 = MagECI_1(1);
        By1 = MagECI_1(2);
        Bz1 = MagECI_1(3);
        
        % second magnetometer
        z_body2 = z(4:6);

        MagECI_2 = pinv(DynOpt.sym_att(agent).dcm_v2)*pinv(dcm_v1)*z_body2;
        Bx2 = MagECI_2(1);
        By2 = MagECI_2(2);
        Bz2 = MagECI_2(3);
    end
    
    %%% matrix computation %%%
    H = DynOpt.sym_att(agent).Hsym_att( wx, wy, wz, ...
                                        q0, q1, q2, q3, ...
                                        Bx1, By1, Bz1, ...
                                        Bx2, By2, Bz2 );
                
    H = double(H);
end