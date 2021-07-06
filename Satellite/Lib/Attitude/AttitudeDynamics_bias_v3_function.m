function [dw,DynOpt] = AttitudeDynamics_bias_v3_function(w, params, DynOpt)

% Initialize integration vectors and parameters
dw = zeros(size(w));
mie = params.mi;

% Extract attitude, angular velocity and inertia of satellite "i"
Att_sat = w(1:7);
q_ECI2Body =  Att_sat(1:4);
omega_Body2ECI_Body = Att_sat(5:7);

% system inertia
I = params.sat(1).I;

% Extract "i"-th satellite's inertial position and velocity
vect_r = params.SatellitesCoordinates(1:3);
vect_v = params.SatellitesCoordinates(4:6);
r = norm(vect_r);


%%%%%%%%%%%%%%%%%% Attitude Kinematics Equations (Quaternions) %%%%%%%%%%%%%%%%%%
Om = [0, -omega_Body2ECI_Body(1), -omega_Body2ECI_Body(2), -omega_Body2ECI_Body(3);
    omega_Body2ECI_Body(1), 0, omega_Body2ECI_Body(3), -omega_Body2ECI_Body(2);
    omega_Body2ECI_Body(2), -omega_Body2ECI_Body(3), 0, omega_Body2ECI_Body(1);
    omega_Body2ECI_Body(3), omega_Body2ECI_Body(2), -omega_Body2ECI_Body(1), 0];
dw(1:4,1) = 0.5*Om*q_ECI2Body;


%%%%%%%%%%%%%%%%%%%%%%%% Attitude Dynamics Equations %%%%%%%%%%%%%%%%%%%%%%%%
% q_ECI2Body_conv = ConvertQuat_V2_1(q_ECI2Body', 'ScalarTo4');

%%%%%% rotation matrix %%%%%
% R_ECI2Body = RotationConversion_V2_1('QtoDCM', q_ECI2Body_conv);
R_ECI2Body = quat2dcm(transpose(q_ECI2Body));

Iom = I*omega_Body2ECI_Body;
cross_omIom = [omega_Body2ECI_Body(2)*Iom(3) - omega_Body2ECI_Body(3)*Iom(2);...
    omega_Body2ECI_Body(3)*Iom(1) - omega_Body2ECI_Body(1)*Iom(3);...
    omega_Body2ECI_Body(1)*Iom(2) - omega_Body2ECI_Body(2)*Iom(1)];
%     cross_omIom = cross(omega_Body2ECI_Body,Iom);

% Gravity Gradient Torque Computation
vers_o_Body = -R_ECI2Body*(vect_r/r);
Io = I*vers_o_Body;
cross_oIo = [vers_o_Body(2)*Io(3) - vers_o_Body(3)*Io(2);...
    vers_o_Body(3)*Io(1) - vers_o_Body(1)*Io(3);...
    vers_o_Body(1)*Io(2) - vers_o_Body(2)*Io(1)]; 
GG_torque = (3*mie/(r^3))*cross_oIo;

% Aerodynamic Drag Torque Computation
v_rel = [vect_v(1) + vect_r(2)*params.omega_e; vect_v(2) - vect_r(1)*params.omega_e; vect_v(3)];                        % satellite's relative velocity wrt Earth's rotation
v_rel_mod = norm(v_rel);                                                                                                % module of the relative velocity
rho = ExponentialAtmDensity_V1_1(norm(vect_r) - params.Re);                                                             % atmospheric density at the altitude of i-th satellite
CD = params.sat(1).CD;                                                                                                  % extract i-th satellite CD coefficient
air_drag_torque = [0; 0; 0];                                                                                            % Initialize drag torque for i-th satellite

for j = 1:size(params.sat(1).aero_prop, 2)

    % Extract i-th satellite j-th surface properties
    Area = params.sat(1).aero_prop(j).A;
    Surf_normal = params.sat(1).aero_prop(j).n;
    arm = params.sat(1).aero_prop(j).arm;

    v_rel_body = R_ECI2Body*v_rel;                                                                                      % Compute velocity in BRF
    cross_section = max(Area*dot(v_rel_body, Surf_normal), 0);                                                          % j-th surface cross section
    F_drag = -(1/2)*rho*CD*v_rel_mod*cross_section*v_rel_body;                                                          % Drag force on j-th surface
    T_drag = cross(arm, F_drag);                                                                                        % Drag torque caused by j-th surface
    air_drag_torque = air_drag_torque + T_drag;                                                                         % Sum j-th torque to already calculated ones

end

tau = params.tau(:,1);

dw(5:7, 1) = I\( - cross_omIom + GG_torque + air_drag_torque + tau);
% dw(5:7, 1) = I\( - cross_omIom + tau);

%%%%% BIAS DYNAMICS %%%%
if DynOpt.bias_dyn == 1
    if DynOpt.synthetic_int == 1
        dw_gyro = params.RW_var*randn(DynOpt.nbias,1) + params.RW_mean;
        dw_mag = params.RW_var_mag*randn(3*DynOpt.nMagneto,1) + params.RW_mean_mag;
        dw(8:10,1) = dw_gyro;
        DynOpt.RW_mem(DynOpt.current_pos,:) = dw(8:8+DynOpt.nbias-1,1);
    else
        dw(8:10,1) = zeros(DynOpt.nbias,1);
    end
end

%%%%% TEST WORKAROUND %%%%%
% dw = zeros(7,1);

%%% SPEED UP MODIFICATION %%%
dw = params.eps_coef*dw;
end
