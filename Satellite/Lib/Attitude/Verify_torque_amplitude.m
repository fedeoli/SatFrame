function [GG_torque, air_drag_torque] = Verify_torque_amplitude(w, params)

n_sat = length(w)/7;
dw_out = zeros(7*n_sat,1);
mie = params.mi;
for i = 2
    Att_sat_i = w(7*(i-1)+1:i*7);
    q =  Att_sat_i(1:4);        % q_Hill2Body
    omega_Body2ECI_Body = Att_sat_i(5:7);
    I = params.sat(i).I;
    
    
    X_sat_i = params.X_SAT_ECI(6*(i-1)+1:(i)*6);
    vect_r = X_sat_i(1:3);
    vect_v = X_sat_i(4:6);
    r = norm(vect_r);
    R_ECI2Hill = RECI2Hill(vect_r, vect_v);
    C_Hill2Body = quat2dcm(q');
    R_ECI2Body = C_Hill2Body*R_ECI2Hill;
    omega_Hill2ECI_ECI = OmegaHill2ECI(vect_r, vect_v, mie);
    omega_Body2Hill_Body = -(R_ECI2Body*omega_Hill2ECI_ECI - omega_Body2ECI_Body);
    
%     Om = [0, -omega_Body2Hill_Body(1), -omega_Body2Hill_Body(2), -omega_Body2Hill_Body(3);
%         omega_Body2Hill_Body(1), 0, omega_Body2Hill_Body(3), -omega_Body2Hill_Body(2);
%         omega_Body2Hill_Body(2), -omega_Body2Hill_Body(3), 0, omega_Body2Hill_Body(1);
%         omega_Body2Hill_Body(3), omega_Body2Hill_Body(2), -omega_Body2Hill_Body(1), 0];
%     
%     
%     % Kinematics Equations (Quaternions)
%     dw(1:4,1) = 0.5*Om*q;
%     
%     % Dynamics Equations (Gravity Gradient)
%     % vers_o = - R_ECI2Body*(vect_r'/r);
%     % dw(5:7) = I\(-cross(omega_Body2ECI_Body, I*omega_Body2ECI_Body) + (3*mie/r^3)*cross(vers_o, I*vers_o));
    
    vers_o_Body = -R_ECI2Body*(vect_r/r);
    Iom = I*omega_Body2ECI_Body;
    Io = I*vers_o_Body;
    cross_omIom = [omega_Body2ECI_Body(2)*Iom(3) - omega_Body2ECI_Body(3)*Iom(2);...
        omega_Body2ECI_Body(3)*Iom(1) - omega_Body2ECI_Body(1)*Iom(3);...
        omega_Body2ECI_Body(1)*Iom(2) - omega_Body2ECI_Body(2)*Iom(1)];
   %%%%%%%%%%%%%%%%%%% 
    
    cross_oIo = [vers_o_Body(2)*Io(3) - vers_o_Body(3)*Io(2);...
        vers_o_Body(3)*Io(1) - vers_o_Body(1)*Io(3);...
        vers_o_Body(1)*Io(2) - vers_o_Body(2)*Io(1)];
    GG_torque = (3*mie/(r^3))*cross_oIo;
    
    %%%% aero_drag
    
    air_drag_torque = [0; 0; 0];
    
    pos = params.X_SAT_ECI(6*(i-1) + 1 : 6*(i-1) + 3);
    vel =params.X_SAT_ECI(6*(i-1) + 4 : 6*(i-1) + 6);
    v_rel = [vel(1) + pos(2)*params.omega_e; vel(2) - pos(1)*params.omega_e; vel(3)];                                              % satellite's relative velocity wrt Earth's rotation
    v_rel_mod = norm(v_rel);                                                                                                % module of the relative velocity
    rho = ExponentialAtmDensity_V1_1(norm(pos) - params.Re)*params.Drag_on;                                                                                 % atmospheric density at the altitude of i-th satellite
    
    Cd = 2.2;
    surf_num = max(size(params.sat(i).aero_prop));
    for j = 1: surf_num
        Area = params.sat(i).aero_prop(j).A;
        Surf_normal = params.sat(i).aero_prop(j).n;
        arm = params.sat(i).aero_prop(j).arm;
        v_rel_body = R_ECI2Body*v_rel;
        cross_section = max(Area*dot(v_rel_body,Surf_normal),0);
        
        F_drag = -(1/2)*rho*Cd*v_rel_mod^2*cross_section*v_rel_body/v_rel_mod;
        T_drag = cross(arm, F_drag);
        air_drag_torque = air_drag_torque + T_drag;
    end
    
%     dw(5:7,1) = I\( - cross_omIom + GG_torque + air_drag_torque);
% 
%     % dw(8:10) = vect_v;
%     % dw(11:13) = -mie/(r^3)*vect_r;
%     dw_out(7*(i-1)+1:i*7,1) = dw;
end

