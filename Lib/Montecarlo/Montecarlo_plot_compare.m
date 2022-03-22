%% Montecarlo simulations 
function H = Montecarlo_plot_compare(pathname_1,pathname_2,initperc)
    
    % get element in folder
    a = dir(strcat(pathname_1,'/*.mat'));
    nelem = numel(a)-1;
    
    % plot window
    load(strcat(pathname_1,'/simulation_',num2str(1),'.mat'))
    TimeLength = length(DynOpt.time);

    start_step = max(1,floor(initperc*(TimeLength)));
    end_step = floor(DynOpt.ObserverTest.EndIntervalWindowPercentage*(TimeLength));
    window_interval = start_step:1:end_step;
    time_interval = DynOpt.time(window_interval);

    %%%%%%%%%%% RANDOM SHOTS COMPARISON %%%%%%%%%%%%
    if 1
        % select random shot
%         shot = randi(nelem);
        shot = 10;
        load(strcat(pathname_1,'/simulation_',num2str(shot),'.mat'))
        
        figure()
        hold on
        grid on
%         sgtitle('error comparison')
        semilogy(time_interval,DynOpt.out(1).errnorm_pos(window_interval),'--','LineWidth',2);
        
        load(strcat(pathname_2,'/simulation_',num2str(shot),'.mat'))
        semilogy(time_interval,DynOpt.out(1).errnorm_pos(window_interval),'--','LineWidth',2);
    end
    
    %%%%%%%%%%% DISPERSION ANALYSIS %%%%%%%%%%%
    load(strcat(pathname_1,'/recap.mat'))
    Montecarlo_UKF = Montecarlo_data;
    load(strcat(pathname_2,'/recap.mat'))
    Montecarlo_GPSUKF = Montecarlo_data;
    for n=1:4
       H(n) = Montecarlo_GPSUKF.out(n).errsign_sigma_pos/Montecarlo_UKF.out(n).errsign_sigma_pos;
    end
end