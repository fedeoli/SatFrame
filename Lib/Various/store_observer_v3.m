%% observer - storage
function DynOpt = store_observer_v3(DynOpt,params,initperc)

    TimeLength = length(DynOpt.time);

    start_step = max(1,floor(initperc*(TimeLength)));
    end_step = floor(DynOpt.ObserverTest.EndIntervalWindowPercentage*(TimeLength));

    window_interval = start_step:1:end_step;
    DynOpt.ObserverTest.window_interval = window_interval;
    DynOpt.ObserverTest.window = DynOpt.time([window_interval(1),window_interval(end)]);

    nagent = params.Nagents;
    
    DynOpt.time_interval = DynOpt.time(window_interval);

    % cycle over the dimensions and agents
    for i = 1:3
        for n = 1:nagent
    
            Chi = DynOpt.Xstory_pos_true(1+6*(n-1):3+6*(n-1),:);
            Chi_est = DynOpt.Xstory_pos_est(1+6*(n-1):3+6*(n-1),:);
            Vel = DynOpt.Xstory_pos_true(4+6*(n-1):6+6*(n-1),:);
            Vel_est = DynOpt.Xstory_pos_est(4+6*(n-1):6+6*(n-1),:);

            % trajectories - pos
            DynOpt.out(n).traj_true_pos = Chi;
            DynOpt.out(n).traj_est_pos = Chi_est;
            
            % trajectories - vel
            DynOpt.out(n).traj_true_vel = Vel;
            DynOpt.out(n).traj_est_vel = Vel_est;

            % error
            DynOpt.out(n).traj_err_pos = DynOpt.out(n).traj_true_pos - DynOpt.out(n).traj_est_pos;
            
            % error - vel
            DynOpt.out(n).traj_err_vel = DynOpt.out(n).traj_true_vel - DynOpt.out(n).traj_est_vel;
        end
    end

    % compute error metrics - sign
    for n = 1:nagent     
        
        for i = 1:TimeLength
            DynOpt.out(n).errsign_pos(i) = mean(DynOpt.out(n).traj_err_pos(:,i));
            DynOpt.out(n).errsign_vel(i) = mean(DynOpt.out(n).traj_err_vel(:,i));
        end
        
        DynOpt.out(n).errsign_mean_pos = mean(DynOpt.out(n).errsign_pos(:,window_interval));
        DynOpt.out(n).errsign_sigma_pos = std(DynOpt.out(n).errsign_pos(:,window_interval));
        
        DynOpt.out(n).errsign_mean_vel = mean(DynOpt.out(n).errsign_vel(:,window_interval));
        DynOpt.out(n).errsign_sigma_vel = std(DynOpt.out(n).errsign_vel(:,window_interval));
        
        
    end


    % compute error metrics - norm
    for n = 1:nagent     
        for i = 1:TimeLength
            DynOpt.out(n).errnorm_pos(i) = norm(DynOpt.out(n).traj_err_pos(:,i));
            DynOpt.out(n).errnorm_vel(i) = norm(DynOpt.out(n).traj_err_vel(:,i));            
        end
        
        DynOpt.out(n).errnorm_mean_pos = mean(DynOpt.out(n).errnorm_pos(:,window_interval));
        DynOpt.out(n).errnorm_sigma_pos = std(DynOpt.out(n).errnorm_pos(:,window_interval));
        
        DynOpt.out(n).errnorm_mean_vel = mean(DynOpt.out(n).errnorm_vel(:,window_interval));
        DynOpt.out(n).errnorm_sigma_vel = std(DynOpt.out(n).errnorm_vel(:,window_interval));
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ATTITUDE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%% attitude data %%%%%%%%%%%%%%%%%%%%%%%
    for n = 1:nagent
        for i = 1:TimeLength
            DynOpt.out(n).q_Euler_true(:,i) = DynOpt.wrap(180/pi*quat2eul(transpose(DynOpt.Xstory_att_true(1+7*(n-1):4+7*(n-1),i)), 'ZYX')'); 
            DynOpt.out(n).q_Euler_est(:,i) = DynOpt.wrap(180/pi*quat2eul(transpose(DynOpt.Xstory_att_est(1+7*(n-1):4+7*(n-1),i)), 'ZYX')'); 
            DynOpt.out(n).q_Euler_err(:,i) = DynOpt.wrap(DynOpt.out(n).q_Euler_true(:,i) - DynOpt.out(n).q_Euler_est(:,i));
        end
    end

end