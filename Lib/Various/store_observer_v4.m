%% observer - storage
function DynOpt = store_observer_v4(DynOpt,params,initperc_pos, initperc_att)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% POSITION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    TimeLength = length(DynOpt.time);

    start_step = max(1,floor(initperc_pos*(TimeLength)));
    end_step = floor(DynOpt.ObserverTest.EndIntervalWindowPercentage*(TimeLength));

    window_interval = start_step:1:end_step;
    DynOpt.ObserverTest.window_interval_pos = window_interval;
    DynOpt.ObserverTest.window_pos = DynOpt.time([window_interval(1),window_interval(end)]);

    nagent = params.Nagents;
    
    DynOpt.time_interval_pos = DynOpt.time(window_interval);

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
        
        % temp arrays
        tmp_pos_mean = [];
        tmp_pos_sigma = [];
        tmp_vel_mean = [];
        tmp_vel_sigma = [];
        
        for i = 1:3
            % III.12
            DynOpt.out(n).errsign_mean_pos(i) = mean(DynOpt.out(n).traj_err_pos(i,window_interval));
            DynOpt.out(n).errsign_mean_vel(i) = mean(DynOpt.out(n).traj_err_vel(i,window_interval));
        
            % III.13
            DynOpt.out(n).errsign_sigma_pos(i) = std(DynOpt.out(n).traj_err_pos(i,window_interval));
            DynOpt.out(n).errsign_sigma_vel(i) = std(DynOpt.out(n).traj_err_vel(i,window_interval));    
            
            % temp arrays
            tmp_pos_mean = [tmp_pos_mean; DynOpt.out(n).errsign_mean_pos(i)];
            tmp_pos_sigma = [tmp_pos_sigma; DynOpt.out(n).errsign_sigma_pos(i)];
            tmp_vel_mean = [tmp_vel_mean; DynOpt.out(n).errsign_mean_vel(i)];
            tmp_vel_sigma = [tmp_vel_sigma; DynOpt.out(n).errsign_sigma_vel(i)];
        end
        
        % III.14
        DynOpt.out(n).errsign_simga_pos_mean = mean(tmp_pos_sigma);
        
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
    TimeLength = length(DynOpt.time);

    start_step = max(1,floor(initperc_att*(TimeLength)));
    end_step = floor(DynOpt.ObserverTest.EndIntervalWindowPercentage*(TimeLength));

    window_interval = start_step:1:end_step;
    DynOpt.ObserverTest.window_interval_att = window_interval;
    DynOpt.ObserverTest.window_att = DynOpt.time([window_interval(1),window_interval(end)]);

    nagent = params.Nagents;
    
    DynOpt.time_interval_att = DynOpt.time(window_interval);
    
    %%%%%%%%%%%%%%%%%%% attitude data %%%%%%%%%%%%%%%%%%%%%%%
    for n = 1:nagent
        for i = 1:TimeLength
            
            % euler angles
            DynOpt.out(n).q_Euler_true(:,i) = DynOpt.wrap(180/pi*quat2eul(transpose(DynOpt.Xstory_att_true(1+7*(n-1):4+7*(n-1),i)), 'ZYX')'); 
            DynOpt.out(n).q_Euler_est(:,i) = DynOpt.wrap(180/pi*quat2eul(transpose(DynOpt.Xstory_att_est(1+7*(n-1):4+7*(n-1),i)), 'ZYX')'); 
            DynOpt.out(n).q_Euler_err(:,i) = DynOpt.wrap(DynOpt.out(n).q_Euler_true(:,i) - DynOpt.out(n).q_Euler_est(:,i));
            
            % omega
            DynOpt.out(n).omega_true(:,i) = DynOpt.Xstory_att_true(5+7*(n-1):7+7*(n-1),i); 
            DynOpt.out(n).omega_est(:,i) = DynOpt.Xstory_att_est(5+7*(n-1):7+7*(n-1),i); 
            DynOpt.out(n).omega_err(:,i) = DynOpt.out(n).omega_true(:,i) - DynOpt.out(n).omega_est(:,i);
        end
    end
    
    % compute error metrics - sign
    for n = 1:nagent     
        
        for i = 1:TimeLength
            DynOpt.out(n).errsign_qEuler(i) = mean(DynOpt.out(n).q_Euler_err(:,i));
            DynOpt.out(n).errsign_omega(i) = mean(DynOpt.out(n).omega_err(:,i));
        end
        
        DynOpt.out(n).errsign_mean_qEuler = mean(DynOpt.out(n).errsign_qEuler(:,window_interval));
        DynOpt.out(n).errsign_sigma_qEuler = std(DynOpt.out(n).errsign_qEuler(:,window_interval));
        
        DynOpt.out(n).errsign_mean_omega = mean(DynOpt.out(n).errsign_omega(:,window_interval));
        DynOpt.out(n).errsign_sigma_omega = std(DynOpt.out(n).errsign_omega(:,window_interval));
               
    end
    
    % compute error metrics - norm
    for n = 1:nagent     
        for i = 1:TimeLength
            DynOpt.out(n).errnorm_qEuler(i) = norm(DynOpt.out(n).q_Euler_err(:,i));
            DynOpt.out(n).errnorm_omega(i) = norm(DynOpt.out(n).omega_err(:,i));            
        end
        
        DynOpt.out(n).errnorm_mean_qEuler = mean(DynOpt.out(n).errnorm_qEuler(:,window_interval));
        DynOpt.out(n).errnorm_sigma_qEuler = std(DynOpt.out(n).errnorm_qEuler(:,window_interval));
        
        DynOpt.out(n).errnorm_mean_omega = mean(DynOpt.out(n).errnorm_omega(:,window_interval));
        DynOpt.out(n).errnorm_sigma_omega = std(DynOpt.out(n).errnorm_omega(:,window_interval));
    end
    
    %%%%%%%%%%%%%%%% KF TIMES %%%%%%%%%%%%%%
    if DynOpt.ObserverOn_pos
        tmpmean = mean(DynOpt.ObserverTest.KFtime_pos);
        tmpstd = std(DynOpt.ObserverTest.KFtime_pos);
        tmpmin = find(DynOpt.ObserverTest.KFtime_pos <= tmpmean-4*tmpstd);
        DynOpt.ObserverTest.KFtime_pos(tmpmin) = [];
        tmpmax = find(DynOpt.ObserverTest.KFtime_pos >= tmpmean+4*tmpstd);
        DynOpt.ObserverTest.KFtime_pos(tmpmax) = [];
    end
    
    if DynOpt.ObserverOn_att
        tmpmean = mean(DynOpt.ObserverTest.KFtime_att);
        tmpstd = std(DynOpt.ObserverTest.KFtime_att);
        tmpmin = find(DynOpt.ObserverTest.KFtime_att <= tmpmean-4*tmpstd);
        DynOpt.ObserverTest.KFtime_att(tmpmin) = [];
        tmpmax = find(DynOpt.ObserverTest.KFtime_att >= tmpmean+4*tmpstd);
        DynOpt.ObserverTest.KFtime_att(tmpmax) = [];
    end

end