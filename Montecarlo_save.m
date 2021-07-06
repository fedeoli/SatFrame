%% Montecarlo simulations 
function Montecarlo_save(nsim,pathname)
    path = pathname;
    
    %% Data analysis
    for n = 1:4
        Montecarlo_data.out(n).errnorm_pos = [];
        Montecarlo_data.out(n).errnorm_mean_pos = [];
        Montecarlo_data.out(n).errnorm_sigma_pos = [];
        
        Montecarlo_data.out(n).errsign_pos = [];
        Montecarlo_data.out(n).errsign_mean_pos = [];
        Montecarlo_data.out(n).errsign_sigma_pos = [];
        
        Montecarlo_data.GPStime = [];
        Montecarlo_data.KFtime = [];
    end
    
    for counter = 1:nsim
        
        clc
        disp(['Iteration Number: ', num2str(counter),'/',num2str(nsim)])
        
        file = strcat('/simulation_',int2str(counter));
        final_path = strcat(path,file);
        load(final_path);
        for n = 1:DynOpt.ObserverTest.Nagents
            Montecarlo_data.out(n).errnorm_pos = [Montecarlo_data.out(n).errnorm_pos, DynOpt.out(n).errnorm_pos(:,DynOpt.ObserverTest.window_interval)];
            Montecarlo_data.out(n).errnorm_mean_pos = [Montecarlo_data.out(n).errnorm_mean_pos, DynOpt.out(n).errnorm_mean_pos];
            Montecarlo_data.out(n).errnorm_sigma_pos = [Montecarlo_data.out(n).errnorm_sigma_pos, DynOpt.out(n).errnorm_sigma_pos];
            
            Montecarlo_data.out(n).errsign_pos = [Montecarlo_data.out(n).errsign_pos, DynOpt.out(n).errsign_pos(:,DynOpt.ObserverTest.window_interval)];
            Montecarlo_data.out(n).errsign_mean_pos = [Montecarlo_data.out(n).errsign_mean_pos, DynOpt.out(n).errsign_mean_pos];
            Montecarlo_data.out(n).errsign_sigma_pos = [Montecarlo_data.out(n).errsign_sigma_pos, DynOpt.out(n).errsign_sigma_pos];
        end
        
        if isfield(DynOpt.ObserverTest,'GPStime')
            Montecarlo_data.GPStime = [Montecarlo_data.GPStime , DynOpt.ObserverTest.GPStime];
        end
        if isfield(DynOpt.ObserverTest,'KFtime')
            Montecarlo_data.KFtime = [Montecarlo_data.GPStime , DynOpt.ObserverTest.KFtime];
        end
    end
    
    for n = 1:DynOpt.ObserverTest.Nagents
        Montecarlo_data.out(n).errnorm_mean_pos = mean(Montecarlo_data.out(n).errnorm_mean_pos);
        Montecarlo_data.out(n).errnorm_sigma_pos = mean(Montecarlo_data.out(n).errnorm_sigma_pos);
        
        Montecarlo_data.out(n).errsign_mean_pos = mean(Montecarlo_data.out(n).errsign_mean_pos);
        Montecarlo_data.out(n).errsign_sigma_pos = mean(Montecarlo_data.out(n).errsign_sigma_pos);
        
    end
    
    Montecarlo_data.Nagents = DynOpt.ObserverTest.Nagents;
    keep path Montecarlo_data    
    temp = strcat(path,'/recap');
    save(temp);
end