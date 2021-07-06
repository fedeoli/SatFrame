function [q_out, om_out]  = fun_attitude(q_Hill2Body,omega_Body2ECI_Body , t, dt, chief_coord, params, I_body)



x = [reshape(q_Hill2Body,4,1); reshape(omega_Body2ECI_Body,3,1)];
vect_r = chief_coord(1:3);
vect_v = chief_coord(4:6);
% Runge Kutta 4
K1 = Euler_Quaternions_GravGrad_v2(t,      x,            vect_r, vect_v, params, I_body);
K2 = Euler_Quaternions_GravGrad_v2(t+dt/2, x + K1*dt/2, vect_r, vect_v, params, I_body);
K3 = Euler_Quaternions_GravGrad_v2(t+dt/2, x + K2*dt/2, vect_r, vect_v, params, I_body);
K4 = Euler_Quaternions_GravGrad_v2(t+dt,   x + K3*dt,   vect_r, vect_v, params, I_body);

x = x + (dt/6)*(K1 + 2*K2 + 2*K3 + K4);

q_out= x(1:4)';
om_out = x(5:7);