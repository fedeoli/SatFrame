%% plot orbits - from official data
function SatFrame_plot(DynOpt,params,initperc_pos,initperc_att)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% POSITION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% general info %%%
    nagent = params.Nagents;
    
    TimeLength = length(DynOpt.time);

    start_step = max(1,floor(initperc_pos*(TimeLength)));
    end_step = floor(DynOpt.ObserverTest.EndIntervalWindowPercentage*(TimeLength));
    window_interval = start_step:1:end_step;
    time_interval = DynOpt.time(window_interval);
    
    
    %%% Orbit %%
    if DynOpt.ObserverOn_pos
        if 1
            figure()
            hold on
            grid on

            title('Fleet trajectories')
            xlabel('x Km')
            ylabel('y Km')
            zlabel('z Km')

            % real trajectories
            for i = 1:nagent
                color = [rand rand rand];
                style_real = '.';

                Chi = DynOpt.Xstory_pos_true(1+6*(i-1):3+6*(i-1),:);
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

            Chi_est = DynOpt.Xstory_pos_est(1+6*(i-1):3+6*(i-1),:);
            xest = Chi_est(1,:);
            yest = Chi_est(2,:);
            zest = Chi_est(3,:);
            plot3(xest,yest,zest,style_est,'MarkerFaceColor',color);
            plot3(xest(1),yest(1),zest(1),'+r')
            plot3(xest(end),yest(end),zest(end),'+r')

        end

        end

        %%% State vars - all agents errors %%%
        if 1
            figure()
            for i = 1:3

                subplot(3,1,i)
                grid on;
                hold on   

                for n = 1:nagent
                    color = [rand rand rand];  

                    % error
                    plot(time_interval,DynOpt.out(n).traj_err_pos(window_interval),'--','MarkerFaceColor',color);
                end


            end

            figure()
            for i = 1:3

                subplot(3,1,i)
                grid on;
                hold on   

                for n = 1:nagent
                    color = [rand rand rand];  

                    % error
                    plot(time_interval,DynOpt.out(n).traj_err_vel(window_interval),'--','MarkerFaceColor',color);
                end


            end
        end

        %%% old agents position error %%%
        if 1
            figure
            sgtitle("Agents position estimation error");
            for n = 1:1
                grid on;
                hold on

                plot(time_interval,DynOpt.out(n).errnorm_pos(window_interval),'--','LineWidth',2);
            end
        end

        %%%% THETA STORY %%%
        if 1     
            figure()
            subplot(2,1,1)
            hold on
            grid on
            plot(DynOpt.ObserverTest.theta_story)
            plot(DynOpt.ObserverTest.beta_story)
            legend('theta','beta')

            subplot(2,1,2)
            hold on
            grid on
            nelem = length(DynOpt.ObserverTest.dcond_mean);
            plot(DynOpt.ObserverTest.dcond_mean)
            plot(DynOpt.ObserverTest.dcond_thresh*ones(1,nelem))
            legend('dcond','thresh') 
        end

        %%% ERRORBAR %%%
        if 1
            figure
            hold on
            grid on
            x = [];
            g = [];
            for n = 1:DynOpt.ObserverTest.Nagents
            %         errorbar(n, DynOpt.out(n).errnorm_mean_pos,DynOpt.out(n).errnorm_sigma_pos,'LineWidth',2);
                data = DynOpt.out(n).errnorm_pos(DynOpt.ObserverTest.window_interval_pos);
                x = [x, data];
                g = [g, n*ones(size(data))];
            end
            a = boxplot(x,g);
            ylim auto
        end
    
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% ATTITUDE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%% general info %%%
    nagent = params.Nagents;
    
    TimeLength = length(DynOpt.time);

    start_step = max(1,floor(initperc_att*(TimeLength)));
    end_step = floor(DynOpt.ObserverTest.EndIntervalWindowPercentage*(TimeLength));
    window_interval = start_step:1:end_step;
    time_interval = DynOpt.time(window_interval);
    
    if 0 || DynOpt.ObserverOn_att

        % all agents errors
        if 1
        figure()
        for i = 1:3

            subplot(3,1,i)
            grid on;
            hold on   
            
            % labels
            xlabel('Time [s]');
            ylabel(strcat('err_',num2str(i),' [deg]'));

            for n = 1:1
                color = [rand rand rand];  

                % state est and true
%                 plot(time_interval,DynOpt.out(n).q_Euler_true(i,window_interval),'--','MarkerFaceColor',color);
%                 plot(time_interval,DynOpt.out(n).q_Euler_est(i,window_interval),'--','MarkerFaceColor',color);

                % reference attitude
%                 target = reshape(DynOpt.ObserverTest.target_attitude(:,i,n),size(window_interval));
%                 plot(time_interval,180/pi*target,'-','MarkerFaceColor',color);

                % error
                plot(time_interval,DynOpt.out(n).q_Euler_err(i,window_interval),'--','MarkerFaceColor',color,'LineWidth',1.5);
            end


        end
        end

        if 0
        figure()
        for i = 1:3

            subplot(3,1,i)
            grid on;
            hold on   

            for n = 1:nagent
                color = [rand rand rand];  

                % state est and true
    %             plot(time_interval,DynOpt.out(n).omega_true(i,window_interval),'--','MarkerFaceColor',color);
    %             plot(time_interval,DynOpt.out(n).omega_est(i,window_interval),'--','MarkerFaceColor',color);

                % error
                plot(time_interval,DynOpt.out(n).omega_err(i,window_interval),'--','MarkerFaceColor',color);
            end


        end
        end

        %%% ERRORBAR %%%
        if 1
            figure()
            hold on
            grid on
            x = [];
            g = [];
            for n = 1:DynOpt.ObserverTest.Nagents
                data = DynOpt.out(n).errnorm_qEuler(DynOpt.ObserverTest.window_interval_att);
                x = [x, data];
                g = [g, n*ones(size(data))];
            end
            a = boxplot(x,g);
            ylim([0 50])
            
            % labels
            xlabel('Agent');
            ylabel(strcat('err [deg]'));
        end


        % Alignement analysis
        if 0
        figure()
        for i = 1:3

            subplot(4,1,i)
            xlabel('Time [s]')
            ylabel('Error [deg]')
            grid on;
            hold on   

            for n = 1:1
                color = [rand rand rand];  

                % error
                plot(time_interval,DynOpt.out(n).q_Euler_err(i,window_interval),'--','MarkerFaceColor',color);
            end

        end

        subplot(4,1,4)
        grid on
        hold on
        % state est and true
        for i=1:size(DynOpt.obs.ObsCondB,1)
            color = [rand rand rand];
            plot(time_interval,DynOpt.obs.ObsCondB(i,window_interval),'.','MarkerFaceColor',color);
        end
        xlabel('Time [s]')
        ylabel('Mag angle')
        end
        
        %%% Knorm terms %%%
        if 1
            figure()
            dim = 3*DynOpt.ObserverTest.nMagneto;
            for i=1:dim
%                 color = [rand rand rand];
                plot(DynOpt.time,DynOpt.ObserverTest.att_Knorm(i,:),'r+')
                hold on
            end
            if DynOpt.ObserverTest.Sun
                dim = 3+3*DynOpt.ObserverTest.nMagneto+1;
                for i=dim:dim+2
%                     color = [rand rand rand];
                    plot(DynOpt.time,DynOpt.ObserverTest.att_Knorm(i,:),'bo')
                    hold on
                end 
            end
        end
        
        %%% Kmean terms %%%
        if 0
            figure()
            dim = 3*DynOpt.ObserverTest.nMagneto;
            for i=1:dim
%                 color = [rand rand rand];
                plot(DynOpt.time,DynOpt.ObserverTest.att_Kmean(i,:),'r+')
                hold on
            end
            if DynOpt.ObserverTest.Sun
                dim = 3+3*DynOpt.ObserverTest.nMagneto+1;
                for i=dim:dim+2
%                     color = [rand rand rand];
                    plot(DynOpt.time,DynOpt.ObserverTest.att_Kmean(i,:),'bo')
                    hold on
                end 
            end
        end
    
    end

end
