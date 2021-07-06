function [f] = attitude_Kin_eqs(omega_Body2ECI_Body, q_ECI2Body, In, params, R_ECI2Body)

% Init section
mie = params.mi;

% agent selection
i = 1;

% Extract "i"-th satellite's inertial position and velocity
vect_r = params.SatellitesCoordinates(6*(i-1) + 1 : 6*(i-1) + 3);
vect_v = params.SatellitesCoordinates(6*(i-1) + 4 : 6*(i-1) + 6);
r = norm(vect_r);

%q_dot
Om = [0, -omega_Body2ECI_Body(1), -omega_Body2ECI_Body(2), -omega_Body2ECI_Body(3);
    omega_Body2ECI_Body(1), 0, omega_Body2ECI_Body(3), -omega_Body2ECI_Body(2);
    omega_Body2ECI_Body(2), -omega_Body2ECI_Body(3), 0, omega_Body2ECI_Body(1);
    omega_Body2ECI_Body(3), omega_Body2ECI_Body(2), -omega_Body2ECI_Body(1), 0];
dw(1:4,1) = 0.5*Om*q_ECI2Body; 

%omega_dot
I = diag(In);
Iom = I*omega_Body2ECI_Body.';
cross_omIom = [omega_Body2ECI_Body(2)*Iom(3) - omega_Body2ECI_Body(3)*Iom(2);...
        omega_Body2ECI_Body(3)*Iom(1) - omega_Body2ECI_Body(1)*Iom(3);...
        omega_Body2ECI_Body(1)*Iom(2) - omega_Body2ECI_Body(2)*Iom(1)];
    
% Gravity Gradient Torque Computation
    vers_o_Body = -R_ECI2Body*(vect_r/r);
    Io = I*vers_o_Body;
    cross_oIo = [vers_o_Body(2)*Io(3) - vers_o_Body(3)*Io(2);...
        vers_o_Body(3)*Io(1) - vers_o_Body(1)*Io(3);...
        vers_o_Body(1)*Io(2) - vers_o_Body(2)*Io(1)]; 
    GG_torque = (3*mie/(r^3))*cross_oIo;
%     GG_torque = 0;
    
% Aerodynamic Drag Torque Computation
v_rel = [vect_v(1) + vect_r(2)*params.omega_e; vect_v(2) - vect_r(1)*params.omega_e; vect_v(3)];                        % satellite's relative velocity wrt Earth's rotation
v_rel_mod = norm(v_rel);                                                                                                % module of the relative velocity
rho = ExponentialAtmDensity_V1_1(norm(vect_r) - params.Re);                                                             % atmospheric density at the altitude of i-th satellite
CD = params.sat(i).CD;                                                                                                  % extract i-th satellite CD coefficient
air_drag_torque = [0; 0; 0];                                                                                            % Initialize drag torque for i-th satellite
    
for j = 1:size(params.sat(i).aero_prop, 2)
        
    % Extract i-th satellite j-th surface properties
    Area = params.sat(i).aero_prop(j).A;
    Surf_normal = params.sat(i).aero_prop(j).n;
    arm = params.sat(i).aero_prop(j).arm;

    
    v_rel_body = R_ECI2Body*v_rel;                                                                                        % Compute velocity in BRF
%     cross_section = max(Area*dot(v_rel_body, Surf_normal), 0);                                                          % j-th surface cross section
%     F_drag = -(1/2)*rho*CD*v_rel_mod*cross_section*v_rel_body;                                                          % Drag force on j-th surface
%     T_drag = cross(arm, F_drag);                                                                                        % Drag torque caused by j-th surface
%     air_drag_torque = air_drag_torque + T_drag;                                                                         % Sum j-th torque to already calculated ones
    T_drag = 0;
    air_drag_torque = air_drag_torque + T_drag;
        
end

% air_drag_torque = [0; 0; 0];
% T_drag = 0;
% GG_torque = 0;

% cross_omIom = 0;
dw(5:7, 1) = I\( - cross_omIom + GG_torque + air_drag_torque);

% dw(5:7, 1) = 0;

f = vpa(simplify(dw));

end
