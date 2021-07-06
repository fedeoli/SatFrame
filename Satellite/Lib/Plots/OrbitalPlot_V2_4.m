
if strcmpi(params.PlotWindowFormat, 'single')
    
    % Create Window
    figure('units', 'normalized', 'outerposition', [0 0 1 1], 'Name', 'Orbital Plot')
    hold on
    
    for j = 1:N_deputy
        
        % Radial Position
        ax(1) = subplot(3,4,1);
        hold on
        plot(time_plot, deputy_rel_LVLH_alltimes(1,:,j), 'color', colors(j+1,:));
        ttl = title('Radial Position');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('$x\,(km)$')
        grid
        
        % Along-track Position
        ax(2) = subplot(3,4,2);
        hold on
        plot(time_plot, deputy_rel_LVLH_alltimes(2,:,j), 'color', colors(j+1,:))
        ttl = title('Along Track Position');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('$y\,(km)$')
        grid
        
        % Cross-track Position
        ax(3) = subplot(3,4,3);
        hold on
        plot(time_plot, deputy_rel_LVLH_alltimes(3,:,j), 'color', colors(j+1,:))
        ttl = title('Cross Track Position');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('$z\,(km)$')
        grid
        
        % Control effort
        if strcmpi(params.ControlStrategy, 'Continuous')
            
            % Control Acceleration
            ax(4) = subplot(3,4,4);
            hold on
            grid on
            plot(time_plot, u_out(:,:,j)*M_d*1000, 'color', colors(j+1,:))
            ttl = title('Thrust Profile');
            set(ttl, 'FontSize', 16, 'fontweight', 'bold');
            xlabel(time_label)
            ylabel('Thrust (N)')
            legend('$u_{radial}$', '$u_{alongtrack}$','$u_{crosstrack}$')
            
        elseif strcmpi(params.ControlStrategy, 'Impulsive')
            
            % Impulsive Delta V
            subplot(3,4,4);
            hold on
            grid on
            ttl = title('Firings');
            set(ttl, 'FontSize', 16, 'fontweight', 'bold');
            bar(time(1:end-1)/params.DT_burn, module_V2_1(params.DV(:,:,j))*1000, 100, 'FaceColor', colors(j+1,:), 'FaceAlpha', 0.7, 'EdgeColor', 'black')
            xlabel('Number of thrusting opportunities')
            ylabel('$\Delta V$ (m/s)')
            
        end
        
        % Radial Velocity
        ax(5) = subplot(3,4,5);
        hold on
        plot(time_plot, deputy_rel_LVLH_alltimes(4,:,j), 'color', colors(j+1,:))
        ttl = title('Radial Velocity');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('$\dot{x}\,(km/s)$')
        grid
        
        % Along-Track Velocity
        ax(6) = subplot(3,4,6);
        hold on
        plot(time_plot, deputy_rel_LVLH_alltimes(5,:,j), 'color', colors(j+1,:))
        ttl = title('Along Track Velocity');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('$\dot{y}\,(km/s)$')
        grid
        
        % Cross-Track Velocity
        ax(7) = subplot(3,4,7);
        hold on
        plot(time_plot, deputy_rel_LVLH_alltimes(6,:,j), 'color', colors(j+1,:))
        ttl = title('Cross Track Velocity');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('$\dot{z}\,(km/s)$')
        grid
        
        % Delta V
        ax(8) = subplot(3,4,8);
        hold on
        plot(time_plot, DeltaV_inst(j,:)*1000, 'color', colors(j+1,:))
        ttl = title('$\Delta V \, (m/s)$');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('$\Delta V\,(m/s)$')
        grid
        
        % XY Trajectory
        subplot(3,4,9);
        hold on
        plot(deputy_rel_LVLH_alltimes(1,:,j), deputy_rel_LVLH_alltimes(2,:,j), 'color', colors(j+1,:))
        ttl = title('XY Plane');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel('$x\,(km)$')
        ylabel('$y\,(km)$')
        axis equal
        grid
        
        % YZ Trajectory
        subplot(3,4,10);
        hold on
        plot(deputy_rel_LVLH_alltimes(2,:,j), deputy_rel_LVLH_alltimes(3,:,j), 'color', colors(j+1,:))
        ttl = title('YZ Plane');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel('$y\,(km)$')
        ylabel('$z\,(km)$')
        axis equal
        grid
        
        % XZ Trajectory
        subplot(3,4,11);
        hold on
        plot(deputy_rel_LVLH_alltimes(1,:,j), deputy_rel_LVLH_alltimes(3,:,j), 'color', colors(j+1,:))
        ttl = title('XZ Plane');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel('$x\,(km)$')
        ylabel('$z\,(km)$')
        axis equal
        grid
        
        % 3D Plot
        subplot(3,4,12);
        hold on
        grid on
        axis equal
        plot3(deputy_rel_LVLH_alltimes(1,:,j), deputy_rel_LVLH_alltimes(2,:,j), deputy_rel_LVLH_alltimes(3,:,j), 'color', colors(j+1,:))
        xlabel('x ($km$)')
        ylabel('y ($km$)')
        zlabel('z ($km$)')
        
        % Link 'x' axes
%         linkaxes(ax, 'x');
        
    end
    
    legend(arrayfun(@(N_dep) sprintf('Deputy %d', N_dep), 1:N_deputy, 'UniformOutput', false))
    
    if params.Control
        
        % Norm of the Error
        figure('Name', 'Norm of the Error')
        hold on
        grid on
        for j = 1:N_deputy
            
            plot(time_plot, error_norm(j,:), 'color', colors(j+1,:))
            
        end
        ttl = title('Norm of the Error');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('Error $(km)$')
        legend(arrayfun(@(N_dep) sprintf('Deputy %d', N_dep), 1:N_deputy, 'UniformOutput', false))
        
    end
    
elseif strcmpi(params.PlotWindowFormat, 'multiple')
    
    
    % Radial Position
    figure('Name', 'Radial Position')
    grid on
    hold on
    for j = 1:N_deputy
        
        plot(time_plot, deputy_rel_LVLH_alltimes(1,:,j), 'color', colors(j+1,:))
        
    end
    ttl = title('Radial Position');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel(time_label)
    ylabel('$x\,(km)$')
    legend(arrayfun(@(N_dep) sprintf('Deputy %d', N_dep), 1:N_deputy, 'UniformOutput', false))
    
    % Along-track Position
    figure('Name', 'Along Track Position')
    hold on
    grid on
    for j = 1:N_deputy
        
        plot(time_plot, deputy_rel_LVLH_alltimes(2,:,j), 'color', colors(j+1,:))
        
    end
    ttl = title('Along Track Position');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel(time_label)
    ylabel('$y\,(km)$')
    legend(arrayfun(@(N_dep) sprintf('Deputy %d', N_dep), 1:N_deputy, 'UniformOutput', false))
    
    % Cross-track Position
    figure('Name', 'Cross Track Position')
    hold on
    grid on
    for j = 1:N_deputy
        
        plot(time_plot, deputy_rel_LVLH_alltimes(3,:,j), 'color', colors(j+1,:))
        
    end
    ttl = title('Cross Track Position');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel(time_label)
    ylabel('$z\,(km)$')
    legend(arrayfun(@(N_dep) sprintf('Deputy %d', N_dep), 1:N_deputy, 'UniformOutput', false))
    
    % Radial Velocity
    figure('Name', 'Radial Velocity')
    hold on
    grid on
    for j = 1:N_deputy
        
        plot(time_plot, deputy_rel_LVLH_alltimes(4,:,j), 'color', colors(j+1,:))
        
    end
    ttl = title('Radial Velocity');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel(time_label)
    ylabel('$\dot{x}\,(km/s)$')
    legend(arrayfun(@(N_dep) sprintf('Deputy %d', N_dep), 1:N_deputy, 'UniformOutput', false))
    
    % Along-Track Velocity
    figure('Name', 'Along Track Velocity')
    hold on
    grid on
    for j = 1:N_deputy
        
        plot(time_plot, deputy_rel_LVLH_alltimes(5,:,j), 'color', colors(j+1,:))
        
    end
    ttl = title('Along Track Velocity');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel(time_label)
    ylabel('$\dot{y}\,(km/s)$')
    legend(arrayfun(@(N_dep) sprintf('Deputy %d', N_dep), 1:N_deputy, 'UniformOutput', false))
    
    % Cross-Track Velocity
    figure('Name', 'Cross Track Velocity')
    hold on
    grid on
    for j = 1:N_deputy
        
        plot(time_plot, deputy_rel_LVLH_alltimes(6,:,j), 'color', colors(j+1,:))
        
    end
    ttl = title('Cross Track Velocity');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel(time_label)
    ylabel('$\dot{z}\,(km/s)$')
    legend(arrayfun(@(N_dep) sprintf('Deputy %d', N_dep), 1:N_deputy, 'UniformOutput', false))
    
    % XY Trajectory
    figure('Name', 'XY Plane')
    hold on
    grid on
    for j = 1:N_deputy
        
        plot(deputy_rel_LVLH_alltimes(1,:,j), deputy_rel_LVLH_alltimes(2,:,j), 'color', colors(j+1,:))
        
    end
    ttl = title('XY Plane');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel('$x\,(km)$')
    ylabel('$y\,(km)$')
    axis equal
    legend(arrayfun(@(N_dep) sprintf('Deputy %d', N_dep), 1:N_deputy, 'UniformOutput', false))
    
    % YZ Trajectory
    figure('Name', 'YZ Plane')
    hold on
    grid on
    for j = 1:N_deputy
        
        plot(deputy_rel_LVLH_alltimes(2,:,j), deputy_rel_LVLH_alltimes(3,:,j), 'color', colors(j+1,:))
        
    end
    ttl = title('YZ Plane');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel('$y\,(km)$')
    ylabel('$z\,(km)$')
    axis equal
    legend(arrayfun(@(N_dep) sprintf('Deputy %d', N_dep), 1:N_deputy, 'UniformOutput', false))
    
    % XZ Trajectory
    figure('Name', 'XZ Plane')
    hold on
    grid on
    for j = 1:N_deputy
        
        plot(deputy_rel_LVLH_alltimes(1,:,j), deputy_rel_LVLH_alltimes(3,:,j), 'color', colors(j+1,:))
        
    end
    ttl = title('XZ Plane');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel('$x\,(km)$')
    ylabel('$z\,(km)$')
    axis equal
    legend(arrayfun(@(N_dep) sprintf('Deputy %d', N_dep), 1:N_deputy, 'UniformOutput', false))
    
    if params.Control
        
    % Norm of the Error
    figure('Name', 'Norm of the Error')
    hold on
    grid on
    for j = 1:N_deputy
        
        plot(time_plot, error_norm(j,:), 'color', colors(j+1,:))
        
    end
    ttl = title('Norm of the Error');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel(time_label)
    ylabel('Error $(km)$')
    legend(arrayfun(@(N_dep) sprintf('Deputy %d', N_dep), 1:N_deputy, 'UniformOutput', false))
    
    if strcmpi(params.ControlStrategy, 'Continuous')
        
        % Control Acceleration
        figure('Name', 'Thrust Profile')
        hold on
        grid on
        for j = 1:N_deputy
            
            plot(time_plot, u_out(:,:,j)*M_d*1000, 'color', colors(j+1,:))
            
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
            
            bar(time(1:end-1)/params.DT_burn, module_V2_1(params.DV(:,:,j))*1000, 100, 'FaceColor', colors(j+1,:), 'FaceAlpha', 0.7, 'EdgeColor', 'black')
            
        end
        
        xlabel('Number of thrusting opportunities')
        ylabel('$\Delta V$ (m/s)')
        legend(arrayfun(@(N_dep) sprintf('Deputy %d', N_dep), 1:N_deputy, 'UniformOutput', false))
        
    end
    
    % Delta V
    figure('Name', 'Delta V')
    hold on
    grid on
    for j = 1:N_deputy
        
        plot(time_plot, DeltaV_inst(j,:), 'color', colors(j+1,:))
        
    end
    ttl = title('$\Delta V \, (km/s)$');
    set(ttl, 'FontSize', 16, 'fontweight', 'bold');
    xlabel(time_label)
    ylabel('$\Delta V\,(km/s)$')
    legend(arrayfun(@(N_dep) sprintf('Deputy %d', N_dep), 1:N_deputy, 'UniformOutput', false))
    
    end
    
    % 3D Plot
    figure('Name', '3D Plot')
    hold on
    grid on
    axis equal
    
    for j = 1:N_deputy
        
        plot3(deputy_rel_LVLH_alltimes(1,:,j), deputy_rel_LVLH_alltimes(2,:,j), deputy_rel_LVLH_alltimes(3,:,j), 'color', colors(j+1,:))
        
    end
    
    xlabel('x ($km$)')
    ylabel('y ($km$)')
    zlabel('z ($km$)')
    legend(arrayfun(@(N_dep) sprintf('Deputy %d', N_dep), 1:N_deputy, 'UniformOutput', false))
    
elseif strcmpi(params.PlotWindowFormat, 'mixed')
    
    %%%%%   First Subplot   %%%%%
    figure('units', 'normalized', 'outerposition', [0 0 1 1], 'Name', 'Relative Position and Velocity Components')
    hold on   
    
    for j = 1:N_deputy
        
        % Radial Position
        ax(1) = subplot(2,3,1);
        hold on
        plot(time_plot, deputy_rel_LVLH_alltimes(1,:,j), 'color', colors(j+1,:))
        ttl = title('Radial Position');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('$x\,(km)$')
        grid
        
        % Along-track Position
        ax(2) = subplot(2,3,2);
        hold on
        plot(time_plot, deputy_rel_LVLH_alltimes(2,:,j), 'color', colors(j+1,:))
        ttl = title('Along Track Position');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('$y\,(km)$')
        grid
        
        % Cross-track Position
        ax(3) = subplot(2,3,3);
        hold on
        plot(time_plot, deputy_rel_LVLH_alltimes(3,:,j), 'color', colors(j+1,:))
        ttl = title('Cross Track Position');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('$z\,(km)$')
        grid
        
        % Radial Velocity
        ax(4) = subplot(2,3,4);
        hold on
        plot(time_plot, deputy_rel_LVLH_alltimes(4,:,j), 'color', colors(j+1,:))
        ttl = title('Radial Velocity');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('$\dot{x}\,(km/s)$')
        grid
        
        % Along-Track Velocity
        ax(5) = subplot(2,3,5);
        hold on
        plot(time_plot, deputy_rel_LVLH_alltimes(5,:,j), 'color', colors(j+1,:))
        ttl = title('Along Track Velocity');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('$\dot{y}\,(km/s)$')
        grid
        
        % Cross-Track Velocity
        ax(6) = subplot(2,3,6);
        hold on
        plot(time_plot, deputy_rel_LVLH_alltimes(6,:,j), 'color', colors(j+1,:))
        ttl = title('Cross Track Velocity');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel(time_label)
        ylabel('$\dot{z}\,(km/s)$')
        grid
        
    end
    
    % Link 'x' axes
%     linkaxes(ax, 'x');
    
    legend(arrayfun(@(N_dep) sprintf('Deputy %d', N_dep), 1:N_deputy, 'UniformOutput', false))
    
    
    %%%%%   Second Subplot    %%%%%
    figure('units', 'normalized', 'outerposition', [0 0 1 1], 'Name', 'Relative Trajectories: projection over coordinate planes and 3D plot')
    hold on  
    
    for j = 1:N_deputy
        
        % XY Trajectory
        subplot(2,2,1);
        hold on
        plot(deputy_rel_LVLH_alltimes(1,:,j), deputy_rel_LVLH_alltimes(2,:,j), 'color', colors(j+1,:))
        ttl = title('XY Plane');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel('$x\,(km)$')
        ylabel('$y\,(km)$')
        axis equal
        grid
        
        % YZ Trajectory
        subplot(2,2,2);
        hold on
        plot(deputy_rel_LVLH_alltimes(2,:,j), deputy_rel_LVLH_alltimes(3,:,j), 'color', colors(j+1,:))
        ttl = title('YZ Plane');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel('$y\,(km)$')
        ylabel('$z\,(km)$')
        axis equal
        grid
        
        % XZ Trajectory
        subplot(2,2,3);
        hold on
        plot(deputy_rel_LVLH_alltimes(1,:,j), deputy_rel_LVLH_alltimes(3,:,j), 'color', colors(j+1,:))
        ttl = title('XZ Plane');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        xlabel('$x\,(km)$')
        ylabel('$z\,(km)$')
        axis equal
        grid
        
        % 3D Plot
        subplot(2,2,4);
        axis equal
        hold on
        grid on
        plot3(deputy_rel_LVLH_alltimes(1,:,j), deputy_rel_LVLH_alltimes(2,:,j), deputy_rel_LVLH_alltimes(3,:,j), 'color', colors(j+1,:))
        
        xlabel('x ($km$)')
        ylabel('y ($km$)')
        zlabel('z ($km$)')
    
    end
    
    legend(arrayfun(@(N_dep) sprintf('Deputy %d', N_dep), 1:N_deputy, 'UniformOutput', false))
    
    if params.Control
        
        %%%%%   Third Subplot    %%%%%
        figure('units', 'normalized', 'outerposition', [0 0 1 1], 'Name', 'Maneuver Performance: Delta V and norm of the position error')
        hold on
        
        for j = 1:N_deputy
            
            % Control effort
            if strcmpi(params.ControlStrategy, 'Continuous')
                
                % Control Acceleration
                ax(1) = subplot(2,2,1);
                hold on
                grid on
                plot(time_plot, u_out(:,:,j)*M_d*1000, 'color', colors(j+1,:))
                ttl = title('Thrust Profile');
                set(ttl, 'FontSize', 16, 'fontweight', 'bold');
                xlabel(time_label)
                ylabel('Thrust (N)')
                legend('$u_{radial}$', '$u_{alongtrack}$','$u_{crosstrack}$')
                
            elseif strcmpi(params.ControlStrategy, 'Impulsive')
                
                % Impulsive Delta V
                subplot(2,2,1);
                hold on
                grid on
                ttl = title('Firings');
                set(ttl, 'FontSize', 16, 'fontweight', 'bold');
                bar(time(1:end-1)/params.DT_burn, module_V2_1(params.DV(:,:,j))*1000, 100, 'FaceColor', colors(j+1,:), 'FaceAlpha', 0.7, 'EdgeColor', 'black')
                xlabel('Number of thrusting opportunities')
                ylabel('$\Delta V$ (m/s)')
                
            end
            
            % Delta V
            ax(2) = subplot(2,2,3);
            hold on
            grid on
            plot(time_plot, DeltaV_inst(j,:)*1000, 'color', colors(j+1,:))
            ttl = title('$\Delta V \, (m/s)$');
            set(ttl, 'FontSize', 16, 'fontweight', 'bold');
            xlabel(time_label)
            ylabel('$\Delta V\,(m/s)$')
            
            % Norm of the Error
            ax(3) = subplot(2,2,2);
            hold on
            grid on
            plot(time_plot, error_norm(j,:), 'color', colors(j+1,:))
            ttl = title('Norm of the Error');
            set(ttl, 'FontSize', 16, 'fontweight', 'bold');
            xlabel(time_label)
            ylabel('Error $(km)$')
            
            % 3D Plot
            subplot(2,2,4);
            axis equal
            hold on
            plot3(deputy_rel_LVLH_alltimes(1,:,j), deputy_rel_LVLH_alltimes(2,:,j), deputy_rel_LVLH_alltimes(3,:,j), 'color', colors(j+1,:))
            
            xlabel('x ($km$)')
            ylabel('y ($km$)')
            zlabel('z ($km$)')
            
        end
        
        legend(arrayfun(@(N_dep) sprintf('Deputy %d', N_dep), 1:N_deputy, 'UniformOutput', false))
        
    end
    
end

% TASI-like 3D plot
figure('Name', 'Trajectory with TASI-like axes')
axis equal
hold on
grid on

for j = 1:N_deputy

    plot3(deputy_rel_LVLH_alltimes(2,:,j), -deputy_rel_LVLH_alltimes(3,:,j), deputy_rel_LVLH_alltimes(1,:,j), 'color', colors(j+1,:))
    
end

vECI = quiver3(0, 0, 0, 0.5*max(max(max(deputy_rel_LVLH_alltimes))), 0, 0, 'LineWidth', 2, 'color', 'g');
Nadir = quiver3(0, 0, 0, 0, 0, -0.5*max(max(max(deputy_rel_LVLH_alltimes))), 'LineWidth', 2, 'color', 'm');

xlabel('x ($km$)')
ylabel('y ($km$)')
zlabel('z ($km$)')

str = {'$\hat{x} \parallel v_{ECI}$','$\hat{z} \parallel r_{ECI}$', '$\hat{y} \parallel -h_{ECI}$'};
box = annotation('textbox', [0.2 0.5 0.3 0.3], 'String', str, 'FitBoxToText', 'on', 'FontSize', 16, 'Backgroundcolor', 'w', 'Margin', 10);

legend([arrayfun(@(N_dep) sprintf('Deputy %d', N_dep), 1:N_deputy, 'UniformOutput', false), {'$v_{ECI}$'}, {'Nadir'}])


