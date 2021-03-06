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
            
            %%% only GPSopt error %%%
            if DynOpt.ObserverTest.GPSopt_flag && DynOpt.ObserverOn_pos
                GPS_est = DynOpt.out(n).OnlyGPSopt;
                DynOpt.out(n).traj_est_pos_GPS = GPS_est;
                DynOpt.out(n).traj_err_pos_GPS = DynOpt.out(n).traj_true_pos - DynOpt.out(n).traj_est_pos_GPS;
            end
            
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
        tmp_pos_mean = zeros(3,1);
        tmp_pos_sigma = zeros(3,1);
        tmp_vel_mean = zeros(3,1);
        tmp_vel_sigma = zeros(3,1);
        
        for i = 1:3
            % III.12
            DynOpt.out(n).errsign_mean_pos(i) = mean(DynOpt.out(n).traj_err_pos(i,window_interval));
            DynOpt.out(n).errsign_mean_vel(i) = mean(DynOpt.out(n).traj_err_vel(i,window_interval));
        
            % III.13
            DynOpt.out(n).errsign_sigma_pos(i) = std(DynOpt.out(n).traj_err_pos(i,window_interval));
            DynOpt.out(n).errsign_sigma_vel(i) = std(DynOpt.out(n).traj_err_vel(i,window_interval));    
            
            % temp arrays
            tmp_pos_mean(i) = DynOpt.out(n).errsign_mean_pos(i);
            tmp_pos_sigma(i) = DynOpt.out(n).errsign_sigma_pos(i);
            tmp_vel_mean(i) = DynOpt.out(n).errsign_mean_vel(i);
            tmp_vel_sigma(i) = DynOpt.out(n).errsign_sigma_vel(i);
            
            %%% only GPSopt error %%%
            if DynOpt.ObserverTest.GPSopt_flag && DynOpt.ObserverOn_pos
                DynOpt.out(n).errsign_mean_pos_GPS(i) = mean(DynOpt.out(n).traj_err_pos_GPS(i,window_interval));
                DynOpt.out(n).errsign_sigma_pos_GPS(i) = std(DynOpt.out(n).traj_err_pos_GPS(i,window_interval));
                % GPSopt Sigma Analysis
                DynOpt.out(n).sigma_p_mean(i) = mean(DynOpt.out(n).sigma_p(i,window_interval));
                DynOpt.out(n).sigma_p_mean_exp(i) = std(DynOpt.out(n).traj_err_pos_GPS(i,window_interval));
            end
        end
        
        % III.14
        DynOpt.out(n).errsign_simga_pos_mean = mean(tmp_pos_sigma);
        
    end
    
    if DynOpt.ObserverTest.GPSopt_flag && DynOpt.ObserverOn_pos
        %%% compute GPSopt sigma from theory %%%
        buf_sigma_p = zeros(nagent,3);
        buf_sigma_p_exp = zeros(nagent,3);
        for i=1:nagent
           buf_sigma_p(i,:) = DynOpt.out(i).sigma_p_mean;
           buf_sigma_p_exp(i,:) = DynOpt.out(i).sigma_p_mean_exp;
        end
        DynOpt.sigma_p_mean_all = mean(buf_sigma_p,1);
        DynOpt.sigma_p_mean_all_exp = mean(buf_sigma_p_exp,1);
        DynOpt.sigma_p_ratio_all = DynOpt.sigma_p_mean_all./DynOpt.sigma_p_mean_all_exp;
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
        
        %%% only GPSopt error %%%
        if DynOpt.ObserverTest.GPSopt_flag && DynOpt.ObserverOn_pos
            for i = 1:TimeLength
                DynOpt.out(n).errnorm_pos_GPS(i) = norm(DynOpt.out(n).traj_err_pos_GPS(:,i));
            end        
            % only GPS opt 
            DynOpt.out(n).errnorm_mean_pos_GPS = mean(DynOpt.out(n).errnorm_pos_GPS(:,window_interval));
            DynOpt.out(n).errnorm_sigma_pos_GPS = std(DynOpt.out(n).errnorm_pos_GPS(:,window_interval));
        end
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
            DynOpt.out(n).q_true(:,i) = DynOpt.Xstory_att_true(1+7*(n-1):4+7*(n-1),i); 
            DynOpt.out(n).q_est(:,i) = DynOpt.Xstory_att_est(1+7*(n-1):4+7*(n-1),i); 
            DynOpt.out(n).q_err(:,i) = quatmultiply(quatinv(transpose(DynOpt.out(n).q_true(:,i))),transpose(DynOpt.out(n).q_est(:,i)));
            
            DynOpt.out(n).q_Euler_est(:,i) = DynOpt.wrap(180/pi*quat2eul(transpose(DynOpt.out(n).q_est(:,i))));
            DynOpt.out(n).q_Euler_true(:,i) = DynOpt.wrap(180/pi*quat2eul(transpose(DynOpt.out(n).q_true(:,i))));
            DynOpt.out(n).q_Euler_err(:,i) = DynOpt.wrap(180/pi*quat2eul(transpose(DynOpt.out(n).q_err(:,i))));
            
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
    if DynOpt.ObserverOn_pos && DynOpt.ObserverTest.KF_flag == 1
        tmpmean = mean(DynOpt.ObserverTest.KFtime_pos);
        tmpstd = std(DynOpt.ObserverTest.KFtime_pos);
        tmpmin = find(DynOpt.ObserverTest.KFtime_pos <= tmpmean-4*tmpstd);
        DynOpt.ObserverTest.KFtime_pos(tmpmin) = [];
        tmpmax = find(DynOpt.ObserverTest.KFtime_pos >= tmpmean+4*tmpstd);
        DynOpt.ObserverTest.KFtime_pos(tmpmax) = [];
    end
    
    if DynOpt.ObserverOn_att && DynOpt.ObserverTest.KF_flag == 1
        tmpmean = mean(DynOpt.ObserverTest.KFtime_att);
        tmpstd = std(DynOpt.ObserverTest.KFtime_att);
        tmpmin = find(DynOpt.ObserverTest.KFtime_att <= tmpmean-4*tmpstd);
        DynOpt.ObserverTest.KFtime_att(tmpmin) = [];
        tmpmax = find(DynOpt.ObserverTest.KFtime_att >= tmpmean+4*tmpstd);
        DynOpt.ObserverTest.KFtime_att(tmpmax) = [];
    end

end