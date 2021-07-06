%% EKF A matrix numeric %%
function H = Hmatrix_EKF_v2(map,params,x,u,z)

    % position offset
    offset = map.integration_pos*6;

    % quaternion
    q0 = x(offset+1);
    q1 = x(offset+2);
    q2 = x(offset+3);
    q3 = x(offset+4);
    
    % omega
    wx = x(offset+5);
    wy = x(offset+6);
    wz = x(offset+7);
    
%%%%%%%%%%%%%%%%%%% TEST %%%%%%%%%%%%%%%%%%%%
%     wx = z(1);
%     wy = z(2);
%     wz = z(3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % magnetometers    
    if map.nMagneto >= 1
        z_body1 = z(4:6);
        MagECI_1 = pinv(map.Rmag_1)*z_body1;
        Bx1 = MagECI_1(1)/(norm(MagECI_1));
        By1 = MagECI_1(2)/(norm(MagECI_1));
        Bz1 = MagECI_1(3)/(norm(MagECI_1));
        Bx2 = Bx1;
        By2 = By1;
        Bz2 = Bz1;
    end
    if map.nMagneto >= 2
        z_body2 = z(7:9);
        MagECI_1 = pinv(map.Rmag_2)*z_body2;
        Bx2 = MagECI_1(1)/(norm(MagECI_1));
        By2 = MagECI_1(2)/(norm(MagECI_1));
        Bz2 = MagECI_1(3)/(norm(MagECI_1));
    end
    
%%%%%%%%%%%%%%%%%%% TEST %%%%%%%%%%%%%%%%%%%%    
%     if map.nMagneto >= 1
%         Bx1 = map.MagECI(1);
%         By1 = map.MagECI(2);
%         Bz1 = map.MagECI(3);
%         Bx2 = Bx1;
%         By2 = By1;
%         Bz2 = Bz1; 
%     end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%% params %%%
    Ixx = params.sat(1).I(1,1);
    Iyy = params.sat(1).I(2,2);
    Izz = params.sat(1).I(3,3);
    mie = params.mi;
    r = params.SatellitesCoordinates(1:3);
    rx = r(1);
    ry = r(2);
    rz = r(3);
    r_norm = norm(r);
    v = params.SatellitesCoordinates(4:6);
    vx = v(1);
    vy = v(2);
    vz = v(3);
    v_norm = norm(v);
    
    % input (useless here)
    taux = u(1);
    tauy = u(2);
    tauz = u(3);
    
    % magnetometers
    dt1 = map.RPYbetweenMagSensors(1);
    dt2 = map.RPYbetweenMagSensors(2);
    dt3 = map.RPYbetweenMagSensors(3);
    
    
    %%% matrix computation %%%
    H = map.Hsym(Ixx, Iyy, Izz, ...
                    wx, wy, wz, ...
                    q1, q2, q3, q0, ...
                    rx, ry, rz, r_norm, ...
                    vx, vy, vz, v_norm, ...
                    mie, ...
                    Bx1, By1, Bz1, ...
                    Bx2, By2, Bz2, ...
                    dt1, dt2, dt3, ...
                    taux, tauy, tauz);
                
    H = double(H);
end