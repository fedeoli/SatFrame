%% Montecarlo simulations 
function H = Montecarlo_plot_compare(pathname,initperc)
    path_local = pathname;
    
    % get element in folder
    a = dir(strcat(path_local,'/EKF_200_T500/*.mat'));
    nelem = numel(a)-1;
    
    % plot window
    load(strcat(path_local,'/EKF_200_T500/simulation_',num2str(1),'.mat'))
    TimeLength = length(DynOpt.time);

    start_step = max(1,floor(initperc*(TimeLength)));
    end_step = floor(DynOpt.ObserverTest.EndIntervalWindowPercentage*(TimeLength));
    window_interval = start_step:1:end_step;
    time_interval = DynOpt.time(window_interval);

    %%%%%%%%%%% RANDOM SHOTS COMPARISON %%%%%%%%%%%%
    if 1
        % select random shot
        shot = randi(nelem);
        load(strcat(path_local,'/EKF_200_T500/simulation_',num2str(shot),'.mat'))
        
        figure()
        hold on
        grid on
%         sgtitle('error comparison')
        semilogy(time_interval,DynOpt.out(1).errnorm_pos(window_interval),'--','LineWidth',2);
        
        load(strcat(path_local,'/GPSEKF_200_T500/simulation_',num2str(shot),'.mat'))
        semilogy(time_interval,DynOpt.out(1).errnorm_pos(window_interval),'--','LineWidth',2);
    end
    
    %%%%%%%%%%% DISPERSION ANALYSIS %%%%%%%%%%%
    load(strcat(path_local,'/EKF_200_T500/recap.mat'))
    Montecarlo_UKF = Montecarlo_data;
    load(strcat(path_local,'/GPSEKF_200_T500/recap.mat'))
    Montecarlo_GPSUKF = Montecarlo_data;
    for n=1:4
       H(n) = Montecarlo_GPSUKF.out(n).errsign_sigma_pos/Montecarlo_UKF.out(n).errsign_sigma_pos;
    end
end