function q_out = fun_attitude_target(eul_Hill2Body,omega_Body2Hill_Body , time, chief_coord, deputy_rel_LVLH_alltimes, params)

      
% True relative position and velocity at t0
% rho_Hill_true0 = deputy_rel_LVLH_alltimes(1:3,1,1);
% drho_Hill_true0 = deputy_rel_LVLH_alltimes(4:6,1,1);



q_Hill2Body = eul2quat(eul_Hill2Body,'XYZ');
C_Hill2Body = quat2dcm(q_Hill2Body);
% True rotational velocity between BRF (target) and ECI expressed in BRF

    vect_r = chief_coord(1:3,1);
    vect_v = chief_coord(4:6,1);
omega_Hill2ECI_ECI = OmegaHill2ECI(vect_r, vect_v, params.mi);
R_ECI2Hill = RECI2Hill(vect_r, vect_v);


       
R_ECI2Body = C_Hill2Body*R_ECI2Hill;

omega_Body2ECI_Body = omega_Body2Hill_Body + R_ECI2Body*omega_Hill2ECI_ECI;
% omega_Body2ECI_Body_true0 = [0, 0, 2*pi/(T/8)];

% True rotation matrix between HRF and CRF (Mother) at t0
% R_Hill2Cam_true0 = RHill2Cam(rho_Hill_true0);

% True rotation matrix between CRF and BRF at t0
% R_Hill2Body_true0 = eye(3);
% C_Cam2Body_true0 = R_Hill2Body_true0*R_Hill2Cam_true0';
% C_Hill2Body = eye(3);
% q_Hill2Body = dcm2quat(C_Cam2Body_true0);


N = size(time,2); 
dt = round(time(end)/N);
x_att = zeros(7, N); eul_out = zeros(3, N); q_out= zeros(4, N);
x_att(:,1) = [q_Hill2Body'; omega_Body2ECI_Body];
  x =   x_att(:,1) ;      
 
for i = 1:N
    
    % State and time at ti
    
    t = time(i);
    vect_r = chief_coord(1:3,i);
    vect_v = chief_coord(4:6,i);
    % Runge Kutta 4
    K1 = Euler_Quaternions_GravGrad_v1(t,      x,            vect_r, vect_v, params);
    K2 = Euler_Quaternions_GravGrad_v1(t+dt/2, x + K1*dt/2, vect_r, vect_v, params);
    K3 = Euler_Quaternions_GravGrad_v1(t+dt/2, x + K2*dt/2, vect_r, vect_v, params);
    K4 = Euler_Quaternions_GravGrad_v1(t+dt,   x + K3*dt,   vect_r, vect_v, params);
    
    % Solution at ti+1
    x = x + (dt/6)*(K1 + 2*K2 + 2*K3 + K4);
    
    % output
    x_att(:,i) = x;
    eul_out(:,i) = quat2eul(x(1:4)')*180/pi;
    q_out(:,i)  = x(1:4)';
    
end
% plot(time, eul_out(1:3,:))
% figure
% plot(time, x_att(6,:))
% figure
% plot(time, x_att(7,:))
% 
% plot(time, x_att(5,:))
% figure
% plot(time, x_att(6,:))
% figure
% plot(time, x_att(7,:))
% 
%            