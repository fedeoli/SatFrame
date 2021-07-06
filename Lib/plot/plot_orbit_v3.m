%% plot orbits - from official data
global ObserverTest

%% GPS OPTIMIZATION 
figure
hold on
grid on

title('Fleet trajectories')
xlabel('x Km')
ylabel('y Km')
zlabel('z Km')
    
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

%% old agents position error
figure
sgtitle("Agents position estimation error");
nagent = ObserverTest.Nagents;
% nagent = 4;
% nplot = 1;
for i = 1:nagent
    subplot(nagent,1,i)
    grid on;
    hold on
    
    for z = 1:ObserverTest.Npassi
       Chi_norm(z) = norm(Chi(:,z));
       Chi_est_norm(z) = norm(Chi_est(:,z));
       Chi_err(z) = norm(Chi(:,z)-Chi_est(:,z));
    end

    traj_error(:,i) = Chi_err;
    plot(1:ObserverTest.Npassi,traj_error(:,i),'b--','LineWidth',2);
end

%%%%%%%%%%%%%% SINGLE AGENT DISPERSION ANALYSIS %%%%%%%%%%%%%
% start_step_local = ObserverTest.UWBOptimizationNoBeforeThan;
start_step_local = 20;

figure
% sgtitle("Agent 1: GPS vs Filtered GPS estimation error");
nagent = 1;
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

    temp_KFerror = reshape(ObserverTest.KFerror(nagent,:,:),size(ObserverTest.KFerror,2),size(ObserverTest.KFerror,3));
    temp_Gpserror = reshape(ObserverTest.Gpserror(nagent,:,:),size(ObserverTest.Gpserror,2),size(ObserverTest.Gpserror,3));

    plot(start_step_local:ObserverTest.Npassi,temp_Gpserror(i,start_step_local:end),'r','LineWidth',2);
    plot(start_step_local:ObserverTest.Npassi,temp_KFerror(i,start_step_local:end),'b','LineWidth',2);
    
    var_GPS = 2*ones(1,ObserverTest.Npassi)*std(traj_error_GPS_only(1,start_step_local:end),0,2);
    plot(start_step_local:ObserverTest.Npassi,var_GPS(start_step_local:end),'k--','LineWidth',2)
    plot(start_step_local:ObserverTest.Npassi,-var_GPS(start_step_local:end),'k--','LineWidth',2)
    legend('GPS','Filtered GPS')
end

%%% why doesn't it filter more? %%%
figure()
plot(start_step_local:ObserverTest.Npassi,traj_error_KF_sum(start_step_local:end)-traj_error_GPS_sum(start_step_local:end),'LineWidth',2); 
ylabel('KF-GEOM')
xlabel('time')

%%%%%%%%%%%%%%%%%%%%% DISPERSION ANALYSIS %%%%%%%%%%%%%%%%%%%%
figure    
plot(start_step_local:ObserverTest.Npassi,traj_error_GPS_only_sum(start_step_local:end) ,'LineWidth',2);
hold on
plot(start_step_local:ObserverTest.Npassi,traj_error_GPS_sum(start_step_local:end),'LineWidth',2);
plot(start_step_local:ObserverTest.Npassi,traj_error_KF_sum(start_step_local:end),'LineWidth',2); 

legend('GPS','GPSopt','Xopt')
figure
errorbar(1:1:ObserverTest.Nagents, mean(traj_error_GPS_only(:,start_step_local:end),2),var(traj_error_GPS_only(:,start_step_local:end),0,2)*100,'LineWidth',2);
hold on
errorbar(1:1:ObserverTest.Nagents, mean(traj_error_GPS(:,start_step_local:end),2),var(traj_error_GPS(:,start_step_local:end),0,2)*100,'LineWidth',2);
errorbar(1:1:ObserverTest.Nagents, mean(traj_error_KF(:,start_step_local:end),2),var(traj_error_KF(:,start_step_local:end),0,2)*100,'LineWidth',2); 

legend('GPS','GPSopt','Xopt')

%%%%%%%%%%%%%%%%% THETA STORY %%%%%%%%%%%%
figure()
hold on
grid on
plot(ObserverTest.theta_story)
plot(ObserverTest.beta_story)
legend('theta','beta')

figure()
hold on
grid on
nelem = length(ObserverTest.dcond_mean);
plot(ObserverTest.dcond_mean)
plot(ObserverTest.dcond_thresh*ones(1,nelem))
legend('dcond','thresh')

