function [DynOpt, params,satellites_iner_ECI,satellites_attitude] = satellite_init_function(DynOpt, params, struct)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                     ANALYSIS SETTINGS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if (~DynOpt.montecarlo) && DynOpt.print
        fprintf('INIT PARAMETERS\n');
    end

    params.InertiaVar = [1, 1, 1];
%     params.InertiaVar = [1.1, 1.2, 1];

    params.Omega0 = 5e-2;

    [params, DynOpt, satellites_iner_ECI, satellites_attitude] = Scenario_K_ORB_A_function(params,DynOpt,struct);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %               DYNOPT INITIALIZATION
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%% STATE AND OBSERVER %%%%
    DynOpt.control = struct.control;
    params.Control = struct.control;


    %%% state data %%%%
    DynOpt.StateDim = 6*params.Nagents;
    DynOpt.StateDim_single_agent = 6;
    DynOpt.init_state = satellites_iner_ECI;

    %%% INTEGRATION SETUP %%%%
    DynOpt.Niter = length(params.time);
    DynOpt.time = params.time;
    DynOpt.Ts = struct.Ts;
    DynOpt.Tstart = params.time(1);
    DynOpt.Tend = params.time(end);
    DynOpt.tspan = [1, 1+DynOpt.Ts];

    %%% MODEL PARAMETERS %%%%
    params.eps_coef = 1;

    %% UPDATE AND INIT %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                     INITIALIZATION
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % final params
    DynOpt.params = params;
    
    % control
    DynOpt.params.input_flag = params.Control;
    params.DesiredAttitude = zeros(3,params.Nagents);

    %%%%%%%%% Initialization scripts %%%%%%%%%%
    params = OutputInitialization_V2_3_function(params,satellites_iner_ECI,satellites_attitude);  
    ObserverTest = SetObserver_v1_2(satellites_iner_ECI, satellites_attitude, params, DynOpt);
    
    % time
    myutc = [2019 12 15 10 20 36];
    ObserverTest.myutc = myutc;
    
    % pass to DynOpt
    DynOpt.ObserverTest = ObserverTest;
    
    %%% Sigma analysis %%%
    if DynOpt.ObserverTest.SigmaAnalysis
        namestr = strcat('SigmaFun_N0',num2str(DynOpt.ObserverTest.Nagents));
        load(namestr);
        DynOpt.ObserverTest.T1 = T1;
        DynOpt.ObserverTest.T2 = T2;
        DynOpt.ObserverTest.T3 = T3;
    end

    for n = 1:DynOpt.ObserverTest.Nagents
        % position
        DynOpt.KF(n).P = DynOpt.ObserverTest.Pi;
        DynOpt.KF(n).Q = DynOpt.ObserverTest.Qi;
        DynOpt.KF(n).R = DynOpt.ObserverTest.Ri;
        
        % attitude
        DynOpt.KF(n).AttitudeP = DynOpt.ObserverTest.AttitudeP;
        DynOpt.KF(n).AttitudeQ = DynOpt.ObserverTest.AttitudeQ;
        DynOpt.KF(n).AttitudeR = DynOpt.ObserverTest.AttitudeR;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%% set orbital position %%%%%
    params.SatellitesCoordinates = satellites_iner_ECI;
    params.SatellitesAttitude = satellites_attitude;
    
    %%% real and corrupted data - position %%%
    DynOpt.Xstory_pos_true(:,1) = satellites_iner_ECI;
    DynOpt.Xstory_pos_est(:,1) = reshape(transpose(DynOpt.ObserverTest.xHatUKF_0),size(satellites_iner_ECI));
    DynOpt.Xstory_pos_wrong(:,1) = DynOpt.Xstory_pos_est(:,1);
    
    %%% real and corrupted data - attitude %%%
    DynOpt.Xstory_att_true(:,1) = satellites_attitude;
    DynOpt.Xstory_att_est(:,1) = reshape(transpose(DynOpt.ObserverTest.attitude_xHatUKF_0),size(satellites_attitude));
    DynOpt.Xstory_att_wrong(:,1) = DynOpt.Xstory_att_est(:,1);
    
    %%% init control %%%
    params.tau = AttitudeControl_V2_5(DynOpt.Xstory_att_est(:,1), DynOpt.Xstory_pos_est(:,1), DynOpt.time(1), params);
    params.DesiredAttitude_default = params.DesiredAttitude;
    DynOpt.ObserverTest.u_freq = 0.01;
    DynOpt.ObserverTest.d = 1;
    DynOpt.ObserverTest.u_amp = 1*-0.05;
    
        
    %%% init apriori estimation - position ad attitude%%% 
    for k=1:DynOpt.ObserverTest.Nagents
        % position
        DynOpt.ObserverTest.APrioriEstimationXYZ(1+3*(k-1):3+3*(k-1)) = DynOpt.Xstory_pos_est(1+6*(k-1):3+6*(k-1),1);
        DynOpt.ObserverTest.APrioriEstimationVel(1+3*(k-1):3+3*(k-1)) = DynOpt.Xstory_pos_est(4+6*(k-1):6+6*(k-1),1);
        
        % attitude
        DynOpt.ObserverTest.APrioriEstimationQuat(1+4*(k-1):4+4*(k-1),1) = DynOpt.Xstory_att_est(1+7*(k-1):4+7*(k-1),1);
        DynOpt.ObserverTest.APrioriEstimationOmega(1+3*(k-1):3+3*(k-1),1) = DynOpt.Xstory_att_est(5+7*(k-1):7+7*(k-1),1);
    end
    
    %%%%%%%% init magnetic field %%%%%%%%
    LatLongAlt = eci2lla(params.SatellitesCoordinates(1:3)'*1E3,myutc); %converto from ECI to latitude, longitude,  altitude
    [mag_field_vector,~,~,~,~] = igrfmagm(max(1000,min(LatLongAlt(3),6E5)),LatLongAlt(1),LatLongAlt(2),decyear(2019,12,15),13); %mag_field_vector is in nanotesla, by IGRF11-12
    DynOpt.mag_field_vector = mag_field_vector;

    
    %%% packet loss - position %%%
    tmp_adj = ones(DynOpt.ObserverTest.Nagents) - eye(DynOpt.ObserverTest.Nagents);
    DynOpt.ObserverTest.SuccessfullyReadUWB(1,:,:) = tmp_adj;
    DynOpt.ObserverTest.SuccessfullyReadGPS = ones(4,1);
    
    %%% EKF %%%
    % pos
    if DynOpt.ObserverOn_pos
        [DynOpt, params] = SymAnalysis_pos_v1(DynOpt,params);
    end
    % att
    [DynOpt,params] = SymAnalysis_att_v1(DynOpt,params);
end




