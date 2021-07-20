%% Montecarlo simulations 
function Montecarlo_plot(pathname)
    path = pathname;
    
    temp = strcat(path,'/recap.mat');
    load(temp);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% POSITION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% ERRORBAR %%%
    if 0
        figure
        sgtitle('norm error boxplot - position')
        hold on
        grid on
        x = [];
        g = [];
        for n = 1:Montecarlo_data.Nagents
            data = Montecarlo_data.out(n).errnorm_pos;
            x = [x, data];
            g = [g, n*ones(size(data))];
        end
        normchart = boxplot(x,g,'symbol','');
        ylim auto
        
        figure
        sgtitle('raw error boxplot - position')
        hold on
        grid on
        x = [];
        g = [];
        for n = 1:Montecarlo_data.Nagents
            data = Montecarlo_data.out(n).errsign_pos;
            x = [x, data];
            g = [g, n*ones(size(data))];
        end
        signchart = boxplot(x,g,'symbol','');
        ylim auto
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ATTITUDE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% ERRORBAR %%%
    if 1
        figure
        sgtitle('norm error boxplot - attitude')
        hold on
        grid on
        x = [];
        g = [];
        for n = 1:Montecarlo_data.Nagents
            data = Montecarlo_data.out(n).errnorm_qEuler;
            x = [x, data];
            g = [g, n*ones(size(data))];
        end
        normchart = boxplot(x,g,'symbol','');
        ylim([0,20])
        
        figure
        sgtitle('raw error boxplot - attitude')
        hold on
        grid on
        x = [];
        g = [];
        for n = 1:Montecarlo_data.Nagents
            data = Montecarlo_data.out(n).errsign_qEuler;
            x = [x, data];
            g = [g, n*ones(size(data))];
        end
        signchart = boxplot(x,g,'symbol','');
        ylim auto
    end
    
end