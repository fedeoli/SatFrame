%% symbolic analysis
function [DynOpt,params] = SymAnalysis_att_v1(DynOpt,params)
    
    
    %%%%%%%%%%%%%%%%%%%% GENERATE MAPS %%%%%%%%%%%%%%%%%%
    %%%%% Sym analysis %%%%%
    if (~DynOpt.montecarlo) && DynOpt.print
        fprintf('Setting Observer parameters: attitude\n');
    end
    
    syms Ixx Iyy Izz;               % System Inertia
    syms wx wy wz;                  % System angular velocity
    syms q1 q2 q3 q0;               % Quaternion
    syms Bx1 By1 Bz1 Bx2 By2 Bz2;   % Magnetometer measures
    syms Sx Sy Sz;                  % Sun sensor measure

    symarray_G = [ wx wy wz ...
                   q0 q1 q2 q3]; 
                    
             
    
    symarray_H = [  wx wy wz ...
                    q0 q1 q2 q3 ...
                    Bx1 By1 Bz1 ...
                    Bx2 By2 Bz2 ...
                    Sx Sy Sz ];
                
    

    %%%%%%%%%%%%% dynamics %%%%%%%%%%%%
    % angular velocity measured by gyro - Body frame
    omega_Body2ECI_Body = [wx wy wz];
    w_body = omega_Body2ECI_Body;

    % quaternion in Body frame
    q_ECI2Body = [q0 q1 q2 q3];
    qin = q_ECI2Body;

    % Magneto measures - Inertial frame
    BI_1 = [Bx1 By1 Bz1];
    DynOpt.ObserverTest.BI1_sym = BI_1;
    BI_2 = [Bx2 By2 Bz2];
    params.BI2_sym = BI_2;

    % Dynamics init - inertia and mass
    In = [Ixx, Iyy, Izz];

    % Cosines matrix from quaternion
    dcm = sym('dcm', [3,3]);

    % MATLAB VERSION
    dcm(1,1,:) = qin(:,1).^2 + qin(:,2).^2 - qin(:,3).^2 - qin(:,4).^2;
    dcm(1,2,:) = 2.*(qin(:,2).*qin(:,3) + qin(:,1).*qin(:,4));  
    dcm(1,3,:) = 2.*(qin(:,2).*qin(:,4) - qin(:,1).*qin(:,3));
    dcm(2,1,:) = 2.*(qin(:,2).*qin(:,3) - qin(:,1).*qin(:,4));
    dcm(2,2,:) = qin(:,1).^2 - qin(:,2).^2 + qin(:,3).^2 - qin(:,4).^2;
    dcm(2,3,:) = 2.*(qin(:,3).*qin(:,4) + qin(:,1).*qin(:,2));
    dcm(3,1,:) = 2.*(qin(:,2).*qin(:,4) + qin(:,1).*qin(:,3));
    dcm(3,2,:) = 2.*(qin(:,3).*qin(:,4) - qin(:,1).*qin(:,2));
    dcm(3,3,:) = qin(:,1).^2 - qin(:,2).^2 - qin(:,3).^2 + qin(:,4).^2;
    
    DynOpt.ObserverTest.dcm = dcm;
    
    % Sun sensor
    DynOpt.ObserverTest.Psun = [Sx, Sy, Sz];
    [DynOpt, params] = Sun_init(DynOpt,params);

    % Magneto measures - Body frame
    B_body_1 = dcm*transpose(BI_1);
    Dtheta = DynOpt.ObserverTest.RPYbetweenMagSensors;
    dcm_v2 = angle2dcm(Dtheta(1),Dtheta(2),Dtheta(3)); 
    DynOpt.ObserverTest.dcm_v2 = dcm_v2;

    if DynOpt.ObserverTest.nMagneto == 2
        B_body_2 = dcm_v2*dcm*transpose(BI_2);
    end

    % Vett. di stato
    X = [q_ECI2Body, omega_Body2ECI_Body]; 

    if DynOpt.ObserverTest.Sun
        if DynOpt.ObserverTest.nMagneto == 1 
            h = [B_body_1; transpose(w_body); DynOpt.ObserverTest.Pbody];
        elseif DynOpt.ObserverTest.nMagneto == 2
            h = [B_body_1 ; B_body_2; transpose(w_body); DynOpt.ObserverTest.Pbody];
        end
    else
        if DynOpt.ObserverTest.nMagneto == 1 
            h = [B_body_1; transpose(w_body)];
        elseif DynOpt.ObserverTest.nMagneto == 2
            h = [B_body_1 ; B_body_2; transpose(w_body)];
        end
    end

    %%%%%% Dynamics setup section %%%%%%%
    % satellite equation
    f = attitude_Kin_eqs(omega_Body2ECI_Body, transpose(q_ECI2Body), In, params, dcm); 

    % Numerical substitution
    for n=1:DynOpt.ObserverTest.Nagents
        % nonlinear eqns
        DynOpt.sym_att(n).f = subs(f, In, [params.sat(n).I(1,1) params.sat(n).I(2,2) params.sat(n).I(3,3)]);
        DynOpt.sym_att(n).h = h;
        DynOpt.sym_att(n).hsym_att = symfun(DynOpt.sym_att(n).h,symarray_H);
        
        % linearization of satellite equations
        DynOpt.sym_att(n).A = jacobian(DynOpt.sym_att(n).f,X);    
        DynOpt.sym_att(n).H = jacobian(h,X);
        
        DynOpt.sym_att(n).Gsym_att = symfun(DynOpt.sym_att(n).A,symarray_G);
        DynOpt.sym_att(n).Hsym_att = symfun(DynOpt.sym_att(n).H,symarray_H);
        
        
        DynOpt.sym_att(n).dcm = dcm;
        DynOpt.sym_att(n).dcm_v2 = dcm_v2;              
    end    
    
    
    % observability analysis - only on one dynamics to save time
    if DynOpt.ObserverTest.obsAnalysis
        
        % flags
        DynOpt.obs.SelAgent = 1;
        DynOpt.obs.nder = 1;
        
        % measure
        DynOpt.obs.Magneto = [BI_1 BI_2];
        DynOpt.obs.Sun = DynOpt.ObserverTest.Psun;
        
        % state
        DynOpt.obs.X = X;
        
        % mappings
        DynOpt.obs.h = h;
        DynOpt.obs.f = DynOpt.sym_att(DynOpt.obs.SelAgent).f;
        
        % sym and assign
        [theta,dtheta,dtheta_num] = ObsAnalysis(DynOpt,DynOpt.obs.nder,0,0,1);
        DynOpt.obs.theta = theta;
        DynOpt.obs.dtheta = dtheta;
        DynOpt.obs.dtheta_num = dtheta_num;
    end
    
end


