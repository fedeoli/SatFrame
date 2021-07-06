%% symbolic analysys
function [DynOpt_out,params_out] = SymAnalysis_RL_v3

    %%% global vars %%%
    global DynOpt params
    
    %%%%%%%%%%%%%%%%%%%% GENERATE MAPS %%%%%%%%%%%%%%%%%%
    %%%%% Sym analysis %%%%%
    fprintf('Setting Observer parameters\n');
    syms Ixx Iyy Izz;       % System Inertia
    syms wx wy wz;          % System angular velocity
    syms q1 q2 q3 q0;       % Quaternion
    syms rx ry rz r_norm;   % sat-Earth vector
    syms vx vy vz v_norm;   % sat-Earth velocity
    syms mie;               % Earth constant
    syms Bx1 By1 Bz1
    syms Bx2 By2 Bz2
    syms dt1 dt2 dt3
    syms taux tauy tauz
    
    symarray_H = [ Ixx Iyy Izz ...
                 wx wy wz ...
                 q1 q2 q3 q0 ...
                 rx ry rz r_norm ...
                 vx vy vz v_norm ...
                 mie ...
                 Bx1 By1 Bz1 ...
                 Bx2 By2 Bz2 ...
                 dt1 dt2 dt3 ...
                 taux tauy tauz];
             
     symarray_G = [ Ixx Iyy Izz ...
                 wx wy wz ...
                 q1 q2 q3 q0 ...
                 rx ry rz r_norm ...
                 vx vy vz v_norm ...
                 mie, ...
                 taux, tauy, tauz];

    % vectors
    q = [q0; q1; q2; q3];
    w = [wx; wy; wz];
    I = diag([Ixx Iyy Izz]);
    r = [rx; ry; rz];
    v = [vx; vy; vz];
    B1 = [Bx1; By1; Bz1];
    B2 = [Bx2; By2; Bz2];
    theta = [dt1; dt2; dt3];
    tauSym = [taux; tauy; tauz];

    %%%%%%%%%%% quaternion dynamics %%%%%%%%%%%
    % cross product
    Om = [0, -w(1), -w(2), -w(3);
        w(1), 0, w(3), -w(2);
        w(2), -w(3), 0, w(1);
        w(3), w(2), -w(1), 0];
    % f map
    Fq = 0.5*Om*q;

    %%%%%%%%%%% Inertial dynamics %%%%%%%%%%%
    R_ECI2Body = Rq(transpose(q));
    
    
    Iom = I*w;
    cross_omIom = cross(w,Iom);

    % gravity gradient
    vers_o_Body = -R_ECI2Body*(r/r_norm);
    Io = I*vers_o_Body;
    cross_oIo = cross(vers_o_Body,Io);
    GG_torque = (3*mie/(r_norm^3))*cross_oIo;

    % final dynamics
    if DynOpt.control == 1
%         Fw = I\( - cross_omIom + GG_torque + tauSym);
        Fw = I\( - cross_omIom + tauSym);
    else
        Fw = I\( - cross_omIom + GG_torque);
    end

    %%%%%% global vars store %%%%%%%%
    DynOpt.q_sym = q;
    DynOpt.w_sym = w;
    DynOpt.mie_sym = mie;
    DynOpt.v_sym = v;
    DynOpt.v_norm_sym = v_norm;

    % map generation
    DynOpt.X = [DynOpt.q_sym; DynOpt.w_sym];
    DynOpt.f = [Fq; Fw];
      
    %%% second magnetometer %%%
    DynOpt.Rtheta = eul2rotm_sym(transpose(theta));
%     DynOpt.Rtheta = eye(3);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    DynOpt.h = DynOpt.w_sym;
    if DynOpt.nMagneto >= 1
        DynOpt.h = [DynOpt.h; R_ECI2Body*B1];
    end
    if DynOpt.nMagneto >= 2
        DynOpt.h = [DynOpt.h; DynOpt.Rtheta*R_ECI2Body*B2];
    end 
     
    %%% linearisation %%%
    % linearization of satellite equations
    DynOpt.G = jacobian(DynOpt.f,DynOpt.X);    
    DynOpt.H = jacobian(DynOpt.h,DynOpt.X); 
    
    DynOpt.Gsym = symfun(DynOpt.G,symarray_G);
    DynOpt.Hsym = symfun(DynOpt.H,symarray_H);
    
    % output
    DynOpt_out = DynOpt;
    params_out = params;

end

