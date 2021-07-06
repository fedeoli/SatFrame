%% plot orbits - from official data
function plot_montecarlo_v1(nsim,path)


    %% GPS OPTIMIZATION 
    figure
    hold on
    grid on

    title('Fleet trajectories')
    xlabel('x Km')
    ylabel('y Km')
    zlabel('z Km')

    for sim = 1:nsim
        file = strcat('/simulation_',int2str(sim));
        final_path = strcat(path,file,'.mat');
        load(final_path)
        
        hold on
        grid on
        
        % nagent = ObserverTest.Nagents;
        nagent = ObserverTest.Nagents;
        % nagent = 1;

            % real trajectories
            for i = 1:nagent
                color = [rand rand rand];
                style_real = '.';
                linewidth = 2;

                Chi = ObserverTest.AllSimulation(ObserverTest.Ntest).iner_ECI(1+6*(i-1):3+6*(i-1),:);
                xreal = Chi(1,:);
                yreal = Chi(2,:);
                zreal = Chi(3,:);
                plot3(xreal,yreal,zreal,style_real,'MarkerFaceColor',color);
                plot3(xreal(1),yreal(1),zreal(1),'bo')
                plot3(xreal(end),yreal(end),zreal(end),'bo')
            end

            % estimated trajectories
            for i = 1:nagent
                color = [rand rand rand];
                style_est = '--';
                linewidth = 2;

                Chi_est = ObserverTest.AllSimulation(ObserverTest.Ntest).estimated_iner_ECI(1+6*(i-1):3+6*(i-1),:);
                xest = Chi_est(1,:);
                yest = Chi_est(2,:);
                zest = Chi_est(3,:);
                plot3(xest,yest,zest,style_est,'MarkerFaceColor',color);
                plot3(xest(1),yest(1),zest(1),'+r')
                plot3(xest(end),yest(end),zest(end),'+r')

        %         xGPS = Agent(i).GPS(1,:);
        %         yGPS = Agent(i).GPS(2,:);
        %         zGPS = Agent(i).GPS(3,:);
        %         plot3(xGPS,yGPS,zGPS,'+r');
            end
    end
end
