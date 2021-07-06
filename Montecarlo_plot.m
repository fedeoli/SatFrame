%% Montecarlo simulations 
function Montecarlo_plot(pathname)
    path = pathname;
    
    temp = strcat(path,'/recap.mat');
    load(temp);
    
    %%% ERRORBAR %%%
    if 1
        figure
        sgtitle('norm error boxplot')
        hold on
        grid on
        x = [];
        g = [];
        for n = 1:Montecarlo_data.Nagents
            data = Montecarlo_data.out(n).errnorm_pos;
            x = [x, data];
            g = [g, n*ones(size(data))];
        end
        normchart = boxplot(x,g);
%         h = findobj(gca,'Tag','Median');
%         for n = 1:Montecarlo_data.Nagents
%             h(n).YData = [1 1]*Montecarlo_data.out(n).errnorm_mean_pos;
%         end
        ylim auto
        
        figure
        sgtitle('raw error boxplot')
        hold on
        grid on
        x = [];
        g = [];
        for n = 1:Montecarlo_data.Nagents
            data = Montecarlo_data.out(n).errsign_pos;
            x = [x, data];
            g = [g, n*ones(size(data))];
        end
        signchart = boxplot(x,g);
%         h = findobj(gca,'Tag','Median');
%         for n = 1:Montecarlo_data.Nagents
%             h(n).YData = [1 1]*Montecarlo_data.out(n).errsign_mean_pos;
%         end
        ylim auto
    end
    
end