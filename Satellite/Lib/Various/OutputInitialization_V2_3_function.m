function params = OutputInitialization_V2_3_function(params,satellites_iner_ECI,satellites_attitude)
    % Initialization of output arrays
    time = params.time;
    tlength = params.tlength;
    N_deputy = params.Ndeputy;
    deputy_rel_LVLH_alltimes = zeros(6, length(time), N_deputy);

    for i = 1:N_deputy

        deputy_rel_LVLH_alltimes(:,1,i) = params.deputy_rel0_LVLH(i,:);

    end

    params.satellites_iner_ECI_out = zeros(6*(N_deputy + 1), length(time));
    params.satellites_iner_ECI_out(:,1) = satellites_iner_ECI;

    if params.Attitude

        params.satellites_attitude_out = zeros(7*(N_deputy + 1), length(time));
        params.satellites_attitude_out(:,1) = satellites_attitude;

    end

    params.DesiredAttitude_out = zeros(3*(N_deputy+1),length(time));
    params.chief_coord = zeros(6, length(time));
    params.u_out = zeros(3, length(time), N_deputy);
    params.u_module = zeros(N_deputy, tlength);
    params.u = zeros(3, N_deputy);
    params.tau_PD = zeros(3, N_deputy + 1);
    params.DeltaV = zeros(N_deputy, 1);
    params.DeltaV_inst = zeros(N_deputy, tlength);
    params.error = zeros(6, tlength, N_deputy);
    params.error_norm = zeros(N_deputy, tlength);
    params.TotalDrift = zeros(3, N_deputy);
    params.DV(:,1,:) = zeros(3, 1, N_deputy);
    params.control_dir_Hill = zeros(3, N_deputy);
    params.tau_PD_out = zeros(3, tlength, N_deputy + 1);
    params.tau_GG_out = params.tau_PD_out;
    params.tau_D_out = params.tau_PD_out;
    params.CollisionProb = zeros(N_deputy, N_deputy, tlength - 1);
    params.CollisionProb_vs_chief = zeros(N_deputy, tlength - 1);
    params.DV = zeros(3, tlength - 1, N_deputy);
    params.BurningTimes = zeros(N_deputy, tlength - 1);
end