if strcmpi(params.PlotWindowFormat, 'single')
    
    % Create Window
    figure('units', 'normalized', 'outerposition', [0 0 1 1], 'Name', 'Orbital Plot')
    hold on
    
    for j = 1:N_deputy
        
        % Radial Position
        subplot(3,4,1)
        hold on
        plot(time_plot, deputy_rel_LVLH_alltimes(1,:,j))
        ttl = title('Radial Position');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('$x\,(km)$')
        grid
        
        % Along-track Position
        subplot(3,4,2)
        hold on
        plot(time_plot, deputy_rel_LVLH_alltimes(2,:,j))
        ttl = title('Along Track Position');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('$y\,(km)$')
        grid
        
        % Cross-track Position
        subplot(3,4,3)
        hold on
        plot(time_plot, deputy_rel_LVLH_alltimes(3,:,j))
        ttl = title('Cross Track Position');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('$z\,(km)$')
        grid
        
        % Norm of the Error
        subplot(3,4,4)
        hold on
        plot(time_plot, error_norm(j,:))
        ttl = title('Norm of the Error');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('Error $(km)$')
        grid
        
        % Radial Velocity
        subplot(3,4,5)
        hold on
        plot(time_plot, deputy_rel_LVLH_alltimes(4,:,j))
        ttl = title('Radial Velocity');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('$\dot{x}\,(km/s)$')
        grid
        
        % Along-Track Velocity
        subplot(3,4,6)
        hold on
        plot(time_plot, deputy_rel_LVLH_alltimes(5,:,j))
        ttl = title('Along Track Velocity');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('$\dot{y}\,(km/s)$')
        grid
        
        % Cross-Track Velocity
        subplot(3,4,7)
        hold on
        plot(time_plot, deputy_rel_LVLH_alltimes(6,:,j))
        ttl = title('Cross Track Velocity');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('$\dot{z}\,(km/s)$')
        grid
        
        if strcmpi(params.ControlStrategy, 'Continuous')
            
            % Control Acceleration
            subplot(3,4,8)
            hold on
            grid on
            plot(time_plot, u_out(:,:,j)*M_d*1000)
            ttl = title('Thrust Profile');
            set(ttl, 'FontSize', 16, 'fontweight', 'bold');
            xlabel(time_label)
            ylabel('Thrust (N)')
            legend('$u_{radial}$', '$u_{alongtrack}$','$u_{crosstrack}$')
            
        elseif strcmpi(params.ControlStrategy, 'Impulsive')
            
            % Impulsive Delta V
            subplot(3,4,8)
            hold on
            grid on
            ttl = title('Delta V');
            set(ttl, 'FontSize', 16, 'fontweight', 'bold');
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
        plot(time_plot, DeltaV_inst(j,:)*1000)
        ttl = title('$\Delta V \, (m/s)$');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('$\Delta V\,(m/s)$')
        grid
        
    end
    
elseif strcmpi(params.PlotWindowFormat, 'multiple')
    
    
    % Radial Position
    figure('Name', 'Radial Position')
    grid on
    hold on
    for j = 1:N_deputy
        
        plot(time_plot, deputy_rel_LVLH_alltimes(1,:,j))
        
    end
    ttl = title('Radial Position');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel(time_label)
    ylabel('$x\,(km)$')
    
    % Along-track Position
    figure('Name', 'Along Track Position')
    hold on
    grid on
    for j = 1:N_deputy
        
        plot(time_plot, deputy_rel_LVLH_alltimes(2,:,j))
        
    end
    ttl = title('Along Track Position');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel(time_label)
    ylabel('$y\,(km)$')
    
    % Cross-track Position
    figure('Name', 'Cross Track Position')
    hold on
    grid on
    for j = 1:N_deputy
        
        plot(time_plot, deputy_rel_LVLH_alltimes(3,:,j))
        
    end
    ttl = title('Cross Track Position');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel(time_label)
    ylabel('$z\,(km)$')
    
    % Radial Velocity
    figure('Name', 'Radial Velocity')
    hold on
    grid on
    for j = 1:N_deputy
        
        plot(time_plot, deputy_rel_LVLH_alltimes(4,:,j))
        
    end
    ttl = title('Radial Velocity');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel(time_label)
    ylabel('$\dot{x}\,(km/s)$')
    
    % Along-Track Velocity
    figure('Name', 'Along Track Velocity')
    hold on
    grid on
    for j = 1:N_deputy
        
        plot(time_plot, deputy_rel_LVLH_alltimes(5,:,j))
        
    end
    ttl = title('Along Track Velocity');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel(time_label)
    ylabel('$\dot{y}\,(km/s)$')
    
    % Cross-Track Velocity
    figure('Name', 'Cross Track Velocity')
    hold on
    grid on
    for j = 1:N_deputy
        
        plot(time_plot, deputy_rel_LVLH_alltimes(6,:,j))
        
    end
    ttl = title('Cross Track Velocity');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel(time_label)
    ylabel('$\dot{z}\,(km/s)$')
    
    % XY Trajectory
    figure('Name', 'XY Plane')
    hold on
    grid on
    for j = 1:N_deputy
        
        plot(deputy_rel_LVLH_alltimes(1,:,j), deputy_rel_LVLH_alltimes(2,:,j))
        
    end
    ttl = title('XY Plane');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel('$x\,(km)$')
    ylabel('$y\,(km)$')
    axis equal
    
    % YZ Trajectory
    figure('Name', 'YZ Plane')
    hold on
    grid on
    for j = 1:N_deputy
        
        plot(deputy_rel_LVLH_alltimes(2,:,j), deputy_rel_LVLH_alltimes(3,:,j))
        
    end
    ttl = title('YZ Plane');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel('$y\,(km)$')
    ylabel('$z\,(km)$')
    axis equal
    
    % XZ Trajectory
    figure('Name', 'XZ Plane')
    hold on
    grid on
    for j = 1:N_deputy
        
        plot(deputy_rel_LVLH_alltimes(1,:,j), deputy_rel_LVLH_alltimes(3,:,j))
        
    end
    ttl = title('XZ Plane');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel('$x\,(km)$')
    ylabel('$z\,(km)$')
    axis equal
    
    % Norm of the Error
    figure('Name', 'Norm of the Error')
    hold on
    grid on
    for j = 1:N_deputy
        
        plot(time_plot, error_norm(j,:))
        
    end
    ttl = title('Norm of the Error');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel(time_label)
    ylabel('Error $(km)$')
    
    if strcmpi(params.ControlStrategy, 'Continuous')
        
        % Control Acceleration
        figure('Name', 'Thrust Profile')
        hold on
        grid on
        for j = 1:N_deputy
            
            plot(time_plot, u_out(:,:,j)*M_d*1000)
            
        end
        xlabel(time_label)
        ylabel('Thrust (N)')
        legend('$u_{radial}$', '$u_{alongtrack}$','$u_{crosstrack}$')
        
    elseif strcmpi(params.ControlStrategy, 'Impulsive')
        
        % Impulsive Delta V
        figure('Name', 'Firing Impulses')
        hold on
        grid on
        for j = 1:N_deputy
            
            bar(time(1:end-1)/params.DT_burn, module_V2_1(params.DV(:,:,j))*1000, 100)
            
        end
        %         bar(time(1:end-1)/params.DT_burn, params.DV(2,:,j)*1000, 20)
        %         bar(time(1:end-1)/params.DT_burn, params.DV(3,:,j)*1000, 20)
        xlabel('Number of thrusting opportunities')
        ylabel('$\Delta V$ (m/s)')
        
    end
    
    % Delta V
    figure('Name', 'Delta V')
    hold on
    grid on
    for j = 1:N_deputy
        
        plot(time_plot, DeltaV_inst(j,:))
        
    end
    ttl = title('$\Delta V \, (km/s)$');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel(time_label)
    ylabel('$\Delta V\,(km/s)$')
    
end

