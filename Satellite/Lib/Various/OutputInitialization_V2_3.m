% Initialization of output arrays
deputy_rel_LVLH_alltimes = zeros(6, length(time), N_deputy);

for i = 1:N_deputy
    
    deputy_rel_LVLH_alltimes(:,1,i) = deputy_rel0_LVLH(i,:);
    
end

satellites_iner_ECI_out = zeros(6*(N_deputy + 1), length(time));
satellites_iner_ECI_out(:,1) = satellites_iner_ECI;

if params.Attitude
    
    satellites_attitude_out = zeros(7*(N_deputy + 1), length(time));
    satellites_attitude_out(:,1) = satellites_attitude;
    
end

DesiredAttitude_out = zeros(3*(N_deputy+1),length(time));
chief_coord = zeros(6, length(time));
u_out = zeros(3, length(time), N_deputy);
u_module = zeros(N_deputy, tlength);
u = zeros(3, N_deputy);
tau_PD = zeros(3, N_deputy + 1);
DeltaV = zeros(N_deputy, 1);
DeltaV_inst = zeros(N_deputy, tlength);
error = zeros(6, tlength, N_deputy);
error_norm = zeros(N_deputy, tlength);
TotalDrift = zeros(3, N_deputy);
params.DV(:,1,:) = zeros(3, 1, N_deputy);
params.control_dir_Hill = zeros(3, N_deputy);
tau_PD_out = zeros(3, tlength, N_deputy + 1);
tau_GG_out = tau_PD_out;
tau_D_out = tau_PD_out;
CollisionProb = zeros(N_deputy, N_deputy, tlength - 1);
CollisionProb_vs_chief = zeros(N_deputy, tlength - 1);
params.DV = zeros(3, tlength - 1, N_deputy);
params.BurningTimes = zeros(N_deputy, tlength - 1);