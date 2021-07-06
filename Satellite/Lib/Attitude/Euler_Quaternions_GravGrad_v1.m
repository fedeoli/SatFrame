function dw = Euler_Quaternions_GravGrad_v1(~, w, vect_r, vect_v, params)

dw = zeros(7,1);
q = w(1:4);         % q_Hill2Body
omega_Body2ECI_Body = w(5:7);
% vect_r = w(8:10);
% vect_v = w(11:13);

r = norm(vect_r);
I = params.I;
mie = params.mi;
R_ECI2Hill = RECI2Hill(vect_r, vect_v);
C_Hill2Body = quat2dcm(q');

R_ECI2Body = C_Hill2Body*R_ECI2Hill;

omega_Hill2ECI_ECI = OmegaHill2ECI(vect_r, vect_v, params.mi);
omega_Body2Hill_Body = -(R_ECI2Body*omega_Hill2ECI_ECI - omega_Body2ECI_Body);

Om = [0, -omega_Body2Hill_Body(1), -omega_Body2Hill_Body(2), -omega_Body2Hill_Body(3);
    omega_Body2Hill_Body(1), 0, omega_Body2Hill_Body(3), -omega_Body2Hill_Body(2);
    omega_Body2Hill_Body(2), -omega_Body2Hill_Body(3), 0, omega_Body2Hill_Body(1);
    omega_Body2Hill_Body(3), omega_Body2Hill_Body(2), -omega_Body2Hill_Body(1), 0];


% Kinematics Equations (Quaternions)
dw(1:4) = 0.5*Om*q;

% Dynamics Equations (Gravity Gradient)
% vers_o = - R_ECI2Body*(vect_r'/r);
% dw(5:7) = I\(-cross(omega_Body2ECI_Body, I*omega_Body2ECI_Body) + (3*mie/r^3)*cross(vers_o, I*vers_o));

vers_o_Body = -R_ECI2Body*(vect_r/r);
Iom = I*omega_Body2ECI_Body;
Io = I*vers_o_Body;
cross_omIom = [omega_Body2ECI_Body(2)*Iom(3) - omega_Body2ECI_Body(3)*Iom(2);...
               omega_Body2ECI_Body(3)*Iom(1) - omega_Body2ECI_Body(1)*Iom(3);...
               omega_Body2ECI_Body(1)*Iom(2) - omega_Body2ECI_Body(2)*Iom(1)];
cross_oIo = [vers_o_Body(2)*Io(3) - vers_o_Body(3)*Io(2);...
             vers_o_Body(3)*Io(1) - vers_o_Body(1)*Io(3);...
             vers_o_Body(1)*Io(2) - vers_o_Body(2)*Io(1)];
         
dw(5:7) = I\( - cross_omIom + (3*mie/(r^3))*cross_oIo );
% dw(8:10) = vect_v;
% dw(11:13) = -mie/(r^3)*vect_r;

end

