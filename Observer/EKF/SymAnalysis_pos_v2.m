%% symbolic analysys
function [DynOpt,params] = SymAnalysis_pos_v2(DynOpt,params)

    
    %%%%%%%%%%%%%%%%%%%% GENERATE MAPS %%%%%%%%%%%%%%%%%%
    %%%%% Sym analysis %%%%%
    fprintf('Setting Observer parameters\n');
    syms rx ry rz;   % sat-Earth vector
    syms vx vy vz;   % sat-Earth velocity
    syms mie;        % Earth constant
    
    
    symarray_H = [  rx ry rz ...
                    vx vy vz ...
                    mie];
             
    symarray_G = symarray_H;

    % vectors
    r = [rx; ry; rz];
    r_norm = norm(r);
    v = [vx; vy; vz];

    %%%%%%%%%%% dynamics %%%%%%%%%%
    F = sym(zeros(6));
    F(1:3,4:6) = eye(3);
    F(4,1) = mie*(3*rx^2/r_norm^5 - 1/r_norm^3);
    F(4,2) = mie*(3*rx*ry/r_norm^5);
    F(4,3) = mie*(3*rx*rz/r_norm^5);

    F(5,1) = F(4,2);
    F(5,2) = mie*(3*ry^2/r_norm^5 - 1/r_norm^3);
    F(5,3) = mie*(3*ry*rz/r_norm^5 - 1/r_norm^3);

    F(6,1) = F(4,3);
    F(6,2) = F(5,3);
    F(6,3) = mie*(3*rz^2/r_norm^5 - 1/r_norm^3);

    %%%%%% global vars store %%%%%%%%
    DynOpt.sym.r_sym = r;
    DynOpt.sym.r_norm_sym = r_norm;
    DynOpt.sym.mie_sym = mie;
    DynOpt.sym.v_sym = v;
    DynOpt.sym.v_norm_sym = v_norm;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    DynOpt.sym.x_pos = [r; v];
    DynOpt.sym.h = [r ; v]; 
     
    %%% linearisation %%%
    % linearization of satellite equations
    DynOpt.sym.G_pos = F;    
    DynOpt.sym.H_pos = jacobian(DynOpt.sym.h,DynOpt.sym.x_pos); 
    
    DynOpt.sym.Gsym_pos = symfun(DynOpt.sym.G_pos,symarray_G);
    DynOpt.sym.Hsym_pos = symfun(DynOpt.sym.H_pos,symarray_H);

end

