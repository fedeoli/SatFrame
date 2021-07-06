% Plot layout
% set(0, 'defaultAxesFontName', 'normal')
set(0, 'defaultAxesFontSize', 18)
set(0, 'defaultAxesFontWeight', 'bold')
set(0, 'defaultAxesGridAlpha', 1)
set(0, 'defaultFigureColor', 'white')
set(0, 'defaultLineLineWidth', 2)
set(groot, 'defaulttextinterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
% 
% % Plots
figure('units', 'normalized', 'outerposition', [0 0 1 1], 'Name', 'Simulation')
hold on

for j = 1:N_deputy
        
    % Radial Position
    subplot(3,4,1)
    hold on
    plot(time/3600, deputy_rel_LVLH_alltimes(1,:,j))
    ttl = title('Radial Position');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel('Time $(h)$')
    ylabel('$x\,(km)$')
    grid
    
    % Along-track Position
    subplot(3,4,2)
    hold on
    plot(time/3600, deputy_rel_LVLH_alltimes(2,:,j))
    ttl = title('Along Track Position');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel('Time $(h)$')
    ylabel('$y\,(km)$')
    grid
    
    % Cross-track Position
    subplot(3,4,3)
    hold on
    plot(time/3600, deputy_rel_LVLH_alltimes(3,:,j))
    ttl = title('Cross Track Position');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel('Time $(h)$')
    ylabel('$z\,(km)$')
    grid
    
    % Norm of the Error
    subplot(3,4,4)
    hold on
    plot(time/3600, error_norm(j,:))
    ttl = title('Norm of the Error');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel('Time $(h)$')
    ylabel('Error $(km)$')
    grid
    
    % Radial Velocity
    subplot(3,4,5)
    hold on
    plot(time/3600, deputy_rel_LVLH_alltimes(4,:,j))
    ttl = title('Radial Velocity');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel('Time $(h)$')
    ylabel('$\dot{x}\,(km/s)$')
    grid
    
    % Along-Track Velocity
    subplot(3,4,6)
    hold on
    plot(time/3600, deputy_rel_LVLH_alltimes(5,:,j))
    ttl = title('Along Track Velocity');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel('Time $(h)$')
    ylabel('$\dot{y}\,(km/s)$')
    grid
    
    % Cross-Track Velocity
    subplot(3,4,7)
    hold on
    plot(time/3600, deputy_rel_LVLH_alltimes(6,:,j))
    ttl = title('Cross Track Velocity');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel('Time $(h)$')
    ylabel('$\dot{z}\,(km/s)$')
    grid
    
    if strcmpi(params.ControlStrategy, 'Continuous')
        
        % Control Acceleration
        subplot(3,4,8)
        hold on
        plot(time/3600, u_out(:,:,j)*M_d*1000)
        ttl = title('Control Profile');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel('Time $(h)$')
        ylabel('Thrust (N)')
        legend('$u_{radial}$', '$u_{alongtrack}$','$u_{crosstrack}$')
        grid
        
    elseif strcmpi(params.ControlStrategy, 'Impulsive')
        
        % Impulsive Delta V
        subplot(3,4,8)
        hold on
        bar(time(1:end-1)/params.DT_burn, module_V2_1(params.DV(:,:,j))*1000, 100)
%         bar(time(1:end-1)/params.DT_burn, params.DV(2,:,j)*1000, 20)
%         bar(time(1:end-1)/params.DT_burn, params.DV(3,:,j)*1000, 20)
        xlabel('Number of thrusting opportunities')
        ylabel('$\Delta V$ (m/s)')
           
    end
    
    
    
    % XY Trajectory
    subplot(3,4,9)
    hold on
    plot(deputy_rel_LVLH_alltimes(1,:,j), deputy_rel_LVLH_alltimes(2,:,j))
    ttl = title('XY Plane');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel('$x\,(km)$')
    ylabel('$y\,(km)$')
    axis equal
    grid
    
    % YZ Trajectory
    subplot(3,4,10)
    hold on
    plot(deputy_rel_LVLH_alltimes(2,:,j), deputy_rel_LVLH_alltimes(3,:,j))
    ttl = title('YZ Plane');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel('$y\,(km)$')
    ylabel('$z\,(km)$')
    axis equal
    grid
    
    % XZ Trajectory
    subplot(3,4,11)
    hold on
    plot(deputy_rel_LVLH_alltimes(1,:,j), deputy_rel_LVLH_alltimes(3,:,j))
    ttl = title('XZ Plane');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel('$x\,(km)$')
    ylabel('$z\,(km)$')
    axis equal
    grid
    
    % Delta V
    subplot(3,4,12)
    hold on
    plot(time/3600, DeltaV_inst(j,:)*1000)
    ttl = title('$\Delta V \, (m/s)$');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel('Time $(h)$')
    ylabel('$\Delta V\,(m/s)$')
    grid
    
end
%  
% for j = 1:N_deputy
%         
%     % Radial Position
%     figure(1)
%     grid on
%     hold on
%     plot(time/3600, deputy_rel_LVLH_alltimes(1,:,j))
%     ttl = title('Radial Position');
%     set(ttl, 'FontSize', 16, 'fontweight', 'bold');
%     xlabel('Time $(h)$')
%     ylabel('$x\,(km)$')
%     
%     % Along-track Position
%     figure(2)
%     hold on
%     grid on    
%     plot(time/3600, deputy_rel_LVLH_alltimes(2,:,j))
%     ttl = title('Along Track Position');
%     set(ttl, 'FontSize', 16, 'fontweight', 'bold');
%     xlabel('Time $(h)$')
%     ylabel('$y\,(km)$')
%     
%     % Cross-track Position
%     figure(3)
%     hold on
%     grid on    
%     plot(time/3600, deputy_rel_LVLH_alltimes(3,:,j))
%     ttl = title('Cross Track Position');
%     set(ttl, 'FontSize', 16, 'fontweight', 'bold');
%     xlabel('Time $(h)$')
%     ylabel('$z\,(km)$')
%     
%     % Norm of the Error
%     figure(4)
%     hold on
%     grid on    
%     plot(time/3600, error_norm(j,:))
%     ttl = title('Norm of the Error');
%     set(ttl, 'FontSize', 16, 'fontweight', 'bold');
%     xlabel('Time $(h)$')
%     ylabel('Error $(km)$')
%     
%     % Radial Velocity
%     figure(5)
%     hold on
%     plot(time/3600, deputy_rel_LVLH_alltimes(4,:,j))
%     ttl = title('Radial Velocity');
%     set(ttl, 'FontSize', 16, 'fontweight', 'bold');
%     xlabel('Time $(h)$')
%     ylabel('$\dot{x}\,(km/s)$')
%     grid
%     
%     % Along-Track Velocity
%     figure(6)
%     hold on
%     plot(time/3600, deputy_rel_LVLH_alltimes(5,:,j))
%     ttl = title('Along Track Velocity');
%     set(ttl, 'FontSize', 16, 'fontweight', 'bold');
%     xlabel('Time $(h)$')
%     ylabel('$\dot{y}\,(km/s)$')
%     grid
%     
%     % Cross-Track Velocity
%     figure(7)
%     hold on
%     plot(time/3600, deputy_rel_LVLH_alltimes(6,:,j))
%     ttl = title('Cross Track Velocity');
%     set(ttl, 'FontSize', 16, 'fontweight', 'bold');
%     xlabel('Time $(h)$')
%     ylabel('$\dot{z}\,(km/s)$')
%     grid
%     
%     % Control Acceleration
%     figure(8)
%     hold on
%     grid on    
%     plot(time/3600, u_out(:,:,j)*M_d*1000)
%     ttl = title('Control Profile');
%     set(ttl, 'FontSize', 16, 'fontweight', 'bold');
%     xlabel('Time $(h)$')
%     ylabel('Thurst (N)')
%     legend('$u_{radial}$', '$u_{alongtrack}$','$u_{crosstrack}$')
%     
%     % XY Trajectory
%     figure(9)
%     hold on
%     grid on
%     plot(deputy_rel_LVLH_alltimes(1,:,j), deputy_rel_LVLH_alltimes(2,:,j))
%     ttl = title('XY Plane');
%     set(ttl, 'FontSize', 16, 'fontweight', 'bold');
%     xlabel('$x\,(km)$')
%     ylabel('$y\,(km)$')
%     axis equal
%     
%     % YZ Trajectory
%     figure(10)
%     hold on
%     plot(deputy_rel_LVLH_alltimes(2,:,j), deputy_rel_LVLH_alltimes(3,:,j))
%     ttl = title('YZ Plane');
%     set(ttl, 'FontSize', 16, 'fontweight', 'bold');
%     xlabel('$y\,(km)$')
%     ylabel('$z\,(km)$')
%     axis equal
%     grid
%     
%     % XZ Trajectory
%     figure(11)
%     hold on
%     plot(deputy_rel_LVLH_alltimes(1,:,j), deputy_rel_LVLH_alltimes(3,:,j))
%     ttl = title('XZ Plane');
%     set(ttl, 'FontSize', 16, 'fontweight', 'bold');
%     xlabel('$x\,(km)$')
%     ylabel('$z\,(km)$')
%     axis equal
%     grid
%     
%     % Delta V
%     figure(12)
%     hold on
%     grid on    
%     plot(time/3600, DeltaV_inst(j,:))
%     ttl = title('$\Delta V \, (km/s)$');
%     set(ttl, 'FontSize', 16, 'fontweight', 'bold');
%     xlabel('Time $(h)$')
%     ylabel('$\Delta V\,(km/s)$')
%     
% end

