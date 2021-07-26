%% Montecarlo simulations 
function SatFrame_plot_compare(initperc)
    
    % plot window
    load('simulations/Singleshot/position/GPS_UKF.mat')
    TimeLength = length(DynOpt.time);

    start_step = max(1,floor(initperc*(TimeLength)));
    end_step = floor(DynOpt.ObserverTest.EndIntervalWindowPercentage*(TimeLength));
    window_interval = start_step:1:end_step;
    time_interval = DynOpt.time(window_interval);

    %%%%%%%%%%% THETA COMPARISON %%%%%%%%%%%%
    if 0
        % load and plot
        load('simulations/Singleshot/position/GPS_UKF_theta003.mat')
        
        figure()
        hold on
        grid on
        sgtitle('error comparison')
        i = 2;
        semilogy(time_interval,DynOpt.out(i).errnorm_pos(window_interval),'--','LineWidth',2);
        
        load('simulations/Singleshot/position/GPS_UKF_theta0005.mat')
        semilogy(time_interval,DynOpt.out(i).errnorm_pos(window_interval),'--','LineWidth',2);
        
        load('simulations/Singleshot/position/GPS_UKF_thetadyn.mat')
        semilogy(time_interval,DynOpt.out(i).errnorm_pos(window_interval),'--','LineWidth',2);
    end
    
    %%%%%%%%%%%%%% DISPERSION ANALYSIS %%%%%%%%%%%%%
    if 1 
        figure()
        sgtitle("Agent 1: GPS vs Filtered GPS estimation error");
        n = 1;
        for i = 1:3
            subplot(3,1,i)
            grid on;
            hold on   

            xlabel('Time [s]')
            if i == 1
                ylabel('X axis [Km]')
            elseif i == 2
                ylabel('Y axis [Km]')
            else
                ylabel('Z axis [Km]')
            end

            load('simulations/Singleshot/position/UKF.mat')
            plot(time_interval,DynOpt.out(n).traj_err_pos(i,window_interval),'r','LineWidth',2);
            load('simulations/Singleshot/position/GPS_UKF.mat')
            plot(time_interval,DynOpt.out(n).traj_err_pos(i,window_interval),'b','LineWidth',2);
            
            load('simulations/Singleshot/position/UKF.mat')
            var_GPS = 2*ones(1,length(DynOpt.time))*DynOpt.out(n).errsign_sigma_pos;
            plot(time_interval,var_GPS(window_interval),'k--','LineWidth',2)
            plot(time_interval,-var_GPS(window_interval),'k--','LineWidth',2)
            legend('GPS','Filtered GPS')
        end 
    end
end