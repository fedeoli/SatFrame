%% Montecarlo simulations 
function Montecarlo_save(nsim,pathname)
    path = pathname;
    
    %% Data analysis
    for n = 1:4
        % position
        Montecarlo_data.out(n).errnorm_pos = [];
        Montecarlo_data.out(n).errnorm_mean_pos = [];
        Montecarlo_data.out(n).errnorm_sigma_pos = [];
        
        Montecarlo_data.out(n).errsign_pos = [];
        Montecarlo_data.out(n).errsign_mean_pos = [];
        Montecarlo_data.out(n).errsign_sigma_pos = [];
        
        Montecarlo_data.GPStime = [];
        Montecarlo_data.KFtime_pos = [];
        Montecarlo_data.out(n).KF_Hdet_pos = [];
        
        % attitude
        Montecarlo_data.out(n).errnorm_qEuler = [];
        Montecarlo_data.out(n).errnorm_mean_qEuler = [];
        Montecarlo_data.out(n).errnorm_sigma_qEuler = [];
        
        Montecarlo_data.out(n).errsign_qEuler = [];
        Montecarlo_data.out(n).errsign_mean_qEuler = [];
        Montecarlo_data.out(n).errsign_sigma_qEuler = [];
        
        Montecarlo_data.KFtime_att = [];
    end
    
    for counter = 1:nsim
        
        try 
            clc
            disp(['Iteration Number: ', num2str(counter),'/',num2str(nsim)])

            file = strcat('/simulation_',int2str(counter));
            final_path = strcat(path,file);
            load(final_path);

            % store data
            for n = 1:DynOpt.ObserverTest.Nagents

                % position 
                Montecarlo_data.out(n).errnorm_pos = [Montecarlo_data.out(n).errnorm_pos, DynOpt.out(n).errnorm_pos(:,DynOpt.ObserverTest.window_interval_pos)];
    %             Montecarlo_data.out(n).errnorm_mean_pos = [Montecarlo_data.out(n).errnorm_mean_pos, DynOpt.out(n).errnorm_mean_pos];
    %             Montecarlo_data.out(n).errnorm_sigma_pos = [Montecarlo_data.out(n).errnorm_sigma_pos, DynOpt.out(n).errnorm_sigma_pos];

    %             Montecarlo_data.out(n).errsign_pos = [Montecarlo_data.out(n).errsign_pos, DynOpt.out(n).errsign_pos(:,DynOpt.ObserverTest.window_interval_pos)];
                Montecarlo_data.out(n).errsign_mean_pos = [Montecarlo_data.out(n).errsign_mean_pos, DynOpt.out(n).errsign_mean_pos];
                Montecarlo_data.out(n).errsign_sigma_pos = [Montecarlo_data.out(n).errsign_sigma_pos, DynOpt.out(n).errsign_sigma_pos];

                % attitude 
                Montecarlo_data.out(n).errnorm_qEuler = [Montecarlo_data.out(n).errnorm_qEuler, DynOpt.out(n).errnorm_qEuler(:,DynOpt.ObserverTest.window_interval_att)];
    %             Montecarlo_data.out(n).errnorm_mean_pos = [Montecarlo_data.out(n).errnorm_mean_qEuler, DynOpt.out(n).errnorm_mean_qEuler];
    %             Montecarlo_data.out(n).errnorm_sigma_pos = [Montecarlo_data.out(n).errnorm_sigma_qEuler, DynOpt.out(n).errnorm_sigma_qEuler];

                Montecarlo_data.out(n).errsign_qEuler = [Montecarlo_data.out(n).errsign_qEuler, DynOpt.out(n).errsign_qEuler(:,DynOpt.ObserverTest.window_interval_att)];
    %             Montecarlo_data.out(n).errsign_mean_qEuler = [Montecarlo_data.out(n).errsign_mean_qEuler, DynOpt.out(n).errsign_mean_qEuler];
    %             Montecarlo_data.out(n).errsign_sigma_qEuler = [Montecarlo_data.out(n).errsign_sigma_qEuler, DynOpt.out(n).errsign_sigma_qEuler];
            end

            if isfield(DynOpt.ObserverTest,'GPStime')
                Montecarlo_data.GPStime = [Montecarlo_data.GPStime , DynOpt.ObserverTest.GPStime];
            end
            if isfield(DynOpt.ObserverTest,'KFtime_pos')
                Montecarlo_data.KFtime_pos = [Montecarlo_data.KFtime_pos , DynOpt.ObserverTest.KFtime_pos];
                
                %%% H determinant %%%
                for n = 1:DynOpt.ObserverTest.Nagents
                    Montecarlo_data.out(n).KF_Hdet_pos = [Montecarlo_data.out(n).KF_Hdet_pos, prod(DynOpt.KF(n).H(DynOpt.ObserverTest.window_interval_pos),1)];
                end
            end
            if isfield(DynOpt.ObserverTest,'KFtime_att')
                Montecarlo_data.KFtime_att = [Montecarlo_data.KFtime_att , DynOpt.ObserverTest.KFtime_att];
            end
        catch ME
            err = ME;
        end
    end
    
    % mean values
    for n = 1:DynOpt.ObserverTest.Nagents
        % position
        Montecarlo_data.out(n).errnorm_mean_pos = mean(Montecarlo_data.out(n).errnorm_pos);
        Montecarlo_data.out(n).errnorm_sigma_pos = std(Montecarlo_data.out(n).errnorm_pos);
        
        Montecarlo_data.out(n).errsign_mean_pos = mean(Montecarlo_data.out(n).errsign_mean_pos);
        Montecarlo_data.out(n).errsign_sigma_pos = mean(Montecarlo_data.out(n).errsign_sigma_pos);
        
        Montecarlo_data.out(n).KF_Hdet_pos_mean = mean(Montecarlo_data.out(n).KF_Hdet_pos);
        
        % attitude
        Montecarlo_data.out(n).errnorm_mean_qEuler = mean(Montecarlo_data.out(n).errnorm_qEuler);
        Montecarlo_data.out(n).errnorm_sigma_qEuler = std(Montecarlo_data.out(n).errnorm_qEuler);
        
        Montecarlo_data.out(n).errsign_mean_qEuler = mean(Montecarlo_data.out(n).errsign_qEuler);
        Montecarlo_data.out(n).errsign_sigma_qEuler = std(Montecarlo_data.out(n).errsign_qEuler);
    end
    
    Montecarlo_data.Nagents = DynOpt.ObserverTest.Nagents;
    keep path Montecarlo_data    
    temp = strcat(path,'/recap');
    save(temp);
end