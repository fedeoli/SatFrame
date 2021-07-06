vect_r = zeros(3, tlength);
vect_v = zeros(3, tlength);
R_ECI2Hill = zeros(3, 3, tlength);
omega_Hill2ECI_ECI = zeros(3, tlength);
omega_Body2Hill_Body = zeros(3*N_deputy+1, tlength);
eul_Hill2Body = zeros(3*N_deputy+1, length(time));

for i = 1:N_deputy + 1
    
    for j = 1:tlength
        
        vect_r = satellites_iner_ECI_out(1:3,j);
        vect_v = satellites_iner_ECI_out(4:6,j);
        R_ECI2Hill = RECI2Hill(vect_r, vect_v);
        omega_Hill2ECI_ECI = OmegaHill2ECI(vect_r, vect_v, params.mi);
        omega_Body2ECI_Body = satellites_attitude_out(5 + 7*(i-1): 7 + 7*(i-1),j);
        R_ECI2Body = quat2dcm(satellites_attitude_out(1 + 7*(i-1): 4 + 7*(i-1),j)');
        omega_Body2Hill_Body(1 + 3*(i-1):3 + 3*(i-1), j) = omega_Body2ECI_Body - R_ECI2Body*omega_Hill2ECI_ECI;
        R_Hill2Body = R_ECI2Body*R_ECI2Hill';
        [eul_Hill2Body(3*(i-1)+1,j), eul_Hill2Body(3*(i-1)+2,j), eul_Hill2Body(3*(i),j)] = dcm2angle(R_Hill2Body, 'ZYX');
        
    end
    
end

if strcmpi(params.PlotWindowFormat, 'single') || strcmpi(params.PlotWindowFormat, 'mixed')
    
    % Create Window
    figure('units', 'normalized', 'outerposition', [0 0 1 1], 'Name', 'Attitude Plot')
    hold on
    
    for i = 1:N_deputy+1
        
        % Euler Angles Yaw 'phi'
        subplot(3,3,1)
        hold on 
        grid on
        ttl = title('Yaw ($\varphi$) Euler Angle');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        plot(time_plot, eul_Hill2Body(3*i,:)*180/pi, 'color', colors(i,:));
        plot(time_plot, unwrap(DesiredAttitude_out(3*i,:))*180/pi, 'color', colors(i,:), 'LineStyle', ':')
        xlabel(time_label)
        ylabel('Yaw ($\varphi$) (deg)')
        legend('Current', 'Desired')
        
        % Euler Angles Roll 'theta'
        subplot(3,3,2)
        hold on
        grid on
        ttl = title('Roll ($\theta$) Euler Angle');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        plot(time_plot, eul_Hill2Body(3*(i-1)+2,:)*180/pi, 'color', colors(i,:))
        plot(time_plot, unwrap(DesiredAttitude_out(3*(i-1)+2,:))*180/pi, 'color', colors(i,:), 'LineStyle', ':')
        xlabel(time_label)
        ylabel('Roll ($\theta$) (deg)')
        legend('Current', 'Desired')
        
        % Euler Angles Pitch 'psi'
        subplot(3,3,3)
        hold on
        grid on
        ttl = title('Pitch ($\psi$) Euler Angle');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        plot(time_plot, unwrap(eul_Hill2Body(3*(i-1)+1,:))*180/pi, 'color', colors(i,:))
        plot(time_plot, unwrap(DesiredAttitude_out(3*(i-1)+1,:))*180/pi, 'color', colors(i,:), 'LineStyle', ':')
        xlabel(time_label)
        ylabel('Pitch ($\psi$) (deg)')
        legend('Current', 'Desired')
        
        % Angular velocity x component
        subplot(3,3,4)
        hold on
        grid on
        ttl = title('Body-to-Hill Angular Velocity (x)');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        plot(time_plot, omega_Body2Hill_Body(1+3*(i-1), :)*180/pi, 'color', colors(i,:))
        xlabel(time_label)
        ylabel('$\omega_{B/H,x}^{BRF}$ ($deg/s$)')
        
        % Angular velocity y component
        subplot(3,3,5)
        hold on
        grid on
        ttl = title('Body-to-Hill Angular Velocity (y)');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        plot(time_plot, omega_Body2Hill_Body(2+3*(i-1), :)*180/pi, 'color', colors(i,:))
        xlabel(time_label)
        ylabel('$\omega_{B/H,y}^{BRF}$ ($deg/s$)')
        
        % Angular velocity z component
        subplot(3,3,6)
        hold on
        grid on
        ttl = title('Body-to-Hill Angular Velocity (z)');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        plot(time_plot, omega_Body2Hill_Body(3*i, :)*180/pi, 'color', colors(i,:))
        xlabel(time_label)
        ylabel('$\omega_{B/H,z}^{BRF}$ ($deg/s$)')
        
        % Control Torque (x)
        subplot(3,3,7)
        hold on
        grid on
        ttl = title('Control torque (x)');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        plot(time_plot, tau_PD_out(1,:,i),'color', colors(i,:))
        xlabel(time_label)
        ylabel('$\tau_{PD,x}$ ($N\,m$)')
        
        % Control Torque (y)
        subplot(3,3,8)
        hold on
        grid on
        ttl = title('Control torque (y)');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        plot(time_plot, tau_PD_out(2,:,i), 'color', colors(i,:))
        xlabel(time_label)
        ylabel('$\tau_{PD,y}$ ($N\,m$)')
        
        % Control Torque (z)
        subplot(3,3,9)
        hold on
        grid on
        ttl = title('Control torque (z)');
        set(ttl, 'FontSize', 16, 'fontweight', 'bold');
        plot(time_plot, tau_PD_out(3,:,i), 'color', colors(i,:))
        xlabel(time_label)
        ylabel('$\tau_{PD,z}$ ($N\,m$)')
        
        
    end
    
    legend(['Chief', arrayfun(@(N_dep) sprintf('Deputy %d', N_dep), 1:N_deputy, 'UniformOutput', false)])
    
elseif strcmpi(params.PlotWindowFormat, 'multiple')
    
    for i = 1:N_deputy+1
        
        figure('Name', ['Euler Angles Deputy ', num2str(i-1)])
        hold on
        grid on
        plot(time_plot, eul_Hill2Body(3*i,:)*180/pi, 'r')
        plot(time_plot, eul_Hill2Body(3*(i-1)+2,:)*180/pi, 'g')
        plot(time_plot, (eul_Hill2Body(3*(i-1)+1,:))*180/pi, 'b')
        plot(time_plot, (DesiredAttitude_out(3*i,:))*180/pi, 'r:')
        plot(time_plot, (DesiredAttitude_out(3*(i-1)+2,:))*180/pi, 'g:')
        plot(time_plot, (DesiredAttitude_out(3*(i-1)+1,:))*180/pi, 'b:')
        xlabel(time_label)
        ylabel('Euler Angles (deg)')
        legend('Yaw ($\varphi$)', 'Roll ($\theta$)', 'Pitch ($\psi$)')
        
    end
    
    for i = 1:N_deputy + 1
        
        figure('Name', ['Angular Velocities Deputy ', num2str(i-1)])
        hold on
        grid on
        plot(time_plot, omega_Body2Hill_Body(1+3*(i-1), :)*180/pi, 'r')
        plot(time_plot, omega_Body2Hill_Body(2+3*(i-1), :)*180/pi, 'g')
        plot(time_plot, omega_Body2Hill_Body(3+3*(i-1), :)*180/pi, 'b')
        xlabel(time_label)
        ylabel('$\omega_{B/H}^{BRF}$ (deg/s)')
        legend('$\omega_x$', '$\omega_y$', '$\omega_z$')
        
    end
    
    
    % figure('Name', 'Gravity Gradient Torque')
    % hold on
    % grid on
    % plot(time(1:end-1), GG_torque_out(1,:), 'r')
    % plot(time(1:end-1), GG_torque_out(2,:), 'g')
    % plot(time(1:end-1), GG_torque_out(3,:), 'b')
    % xlabel(time_label)
    % ylabel('$\tau_{GG}$ ($N\,m$)')
    % legend('$\tau_x$', '$\tau_y$', '$\tau_z$')
    %
    % figure('Name', 'Aerodynamic Torque')
    % grid on
    % hold on
    % plot(time(1:end-1), air_drag_torque_out(1,:), 'r')
    % plot(time(1:end-1), air_drag_torque_out(2,:), 'g')
    % plot(time(1:end-1), air_drag_torque_out(3,:), 'b')
    % xlabel(time_label)
    % ylabel('$\tau_{D}$ ($N\,m$)')
    % legend('$\tau_x$', '$\tau_y$', '$\tau_z$')
    
    for i = 1:N_deputy + 1
        
        figure('Name', ['Control Torque Deputy ', num2str(i-1)])
        hold on
        grid on
        plot(time_plot, tau_PD_out(1,:,i), 'r')
        plot(time_plot, tau_PD_out(2,:,i), 'g')
        plot(time_plot, tau_PD_out(3,:,i), 'b')
        xlabel(time_label)
        ylabel('$\tau_{PD}$ ($N\,m$)')
        legend('$\tau_x$', '$\tau_y$', '$\tau_z$')
        
    end
    
end