% figure('Name', 'Omega x')
% hold on
% for i = 1:N_deputy + 1
%
%     plot(time, satellites_attitude_out(7*(i-1)+5,:)*180/pi)
%
% end
% xlabel('Time (s)')
% ylabel('$\omega_x$ B2E Body (deg/s)')
%
% figure('Name', 'Omega y')
% hold on
% for i = 1:N_deputy + 1
%
%     plot(time, satellites_attitude_out(7*(i-1)+6,:)*180/pi)
%
% end
% xlabel('Time (s)')
% ylabel('$\omega_y$ B2E Body (deg/s)')
% %
% %
% figure('Name', 'Omega z')
% hold on
% for i = 1:N_deputy + 1
%
%     plot(time, satellites_attitude_out(7*(i-1)+7,:)*180/pi)
%
% end
% xlabel('Time (s)')
% ylabel('$\omega_z$ B2E Body (deg/s)')


vect_r = satellites_iner_ECI(1:3);
vect_v = satellites_iner_ECI(4:6);
omega_Hill2ECI_ECI = OmegaHill2ECI(vect_r, vect_v, params.mi);
omega_Body2Hill_Body = zeros(3*N_deputy, tlength);

for i = 2:N_deputy + 1
    
    for j = 1:length(time)
        
        vect_r = satellites_iner_ECI_out(1:3,j);
        vect_v = satellites_iner_ECI_out(4:6,j);
        R_ECI2Hill = RECI2Hill(vect_r, vect_v);
        
        omega_Body2ECI_Body = satellites_attitude_out(5 + 7*(i-1): 7 + 7*(i-1),j);
        R_ECI2Body = quat2dcm(satellites_attitude_out(1 + 7*(i-1): 4 + 7*(i-1),j)');
        
        omega_Hill2ECI_Body = R_ECI2Body*omega_Hill2ECI_ECI(1:3);
        omega_Body2Hill_Body(1 + 3*(i-1):3 + 3*(i-1), j) = omega_Body2ECI_Body - R_ECI2Body*omega_Hill2ECI_ECI;
        
    end
    
end

if 0
    
    figure('Name', 'Omega x')
    hold on
    grid on
    
    for i = 2:N_deputy + 1
        
        plot(time, omega_Body2Hill_Body(1+3*(i-1), :)*180/pi)
        
    end
    
    xlabel('Time (s)')
    ylabel('$\omega_x$ B2H Body (deg/s)')
    
    figure('Name', 'Omega y')
    hold on
    grid on
    
    for i = 2:N_deputy + 1
        
        plot(time, omega_Body2Hill_Body(2+3*(i-1), :)*180/pi)
        
    end
    
    xlabel('Time (s)')
    ylabel('$\omega_y$ B2H Body (deg/s)')
    
    figure('Name', 'Omega z')
    hold on
    grid on
    
    for i = 2:N_deputy + 1
        
        plot(time, omega_Body2Hill_Body(3+3*(i-1), :)*180/pi)
        
    end
    
    xlabel('Time (s)')
    ylabel('$\omega_z$ B2H Body (deg/s)')
    
end

eul_ECI2Body = zeros(3*N_deputy, length(time));
eul_Hill2Body = eul_ECI2Body;

for j = 1:length(time)
    
    R_ECI2Hill = RECI2Hill(satellites_iner_ECI_out(1:3,j), satellites_iner_ECI_out(4:6,j));
    
    for i = 1:N_deputy + 1
        
        quat_ECI2Body = satellites_attitude_out(7*(i-1)+1 : 7*(i-1)+4, j)';
        eul_ECI2Body(3*(i-1)+1:3*i,j) = quat2eul(quat_ECI2Body, 'ZYX');
        R_ECI2Body = quat2dcm(quat_ECI2Body);
        R_Hill2Body = R_ECI2Body*R_ECI2Hill';
        [eul_Hill2Body(3*(i-1)+1,j), eul_Hill2Body(3*(i-1)+2,j), eul_Hill2Body(3*i,j)] = dcm2angle(R_Hill2Body, 'ZYX');
        
    end
    
end

if 0
    
    figure('Name', 'Pitch angle (psi)')
    hold on
    grid on
    
    for i = 2:N_deputy + 1
        
        plot(time, eul_Hill2Body(3*(i-1)+1,:)*180/pi)
        plot(time, DesiredAttitude_out(1 + 3*(i-2),:)*180/pi,'k:')
        
    end
    xlabel('Time (s)')
    ylabel('Pitch ($\psi$) (deg)')
    
    
    figure('Name', 'Roll angle (theta)')
    hold on
    grid on
    
    for i = 2:N_deputy + 1
        
        plot(time, eul_Hill2Body(3*(i-1)+2,:)*180/pi)
        plot(time, DesiredAttitude_out(2 + 3*(i-2),:)*180/pi,'k:')
    end
    xlabel('Time (s)')
    ylabel('Roll ($\theta$)  (deg)')
    
    figure('Name', 'Yaw angle (phi)')
    hold on
    grid on
    
    for i = 2:N_deputy + 1
        
        plot(time, eul_Hill2Body(3*i,:)*180/pi)
        plot(time, DesiredAttitude_out(3 + 3*(i-2),:)*180/pi,'k:')
        
    end
    xlabel('Time (s)')
    ylabel('Yaw ($\phi$) (deg)')
    
    if 0
        
        figure('Name', 'Attitude Control Torques around x')
        hold on
        grid on
        
        for i = 2:N_deputy + 1
            
            plot(time, tau_out(1,:,i))
            
        end
        xlabel('Time $(s)$')
        ylabel('$\tau_x$ $(Nm)$')
        
        figure('Name', 'Attitude Control Torques around y')
        hold on
        grid on
        for i = 2:N_deputy + 1
            
            plot(time, tau_out(2,:,i))
            
        end
        xlabel('Time $(s)$')
        ylabel('$\tau_y$ $(Nm)$')
        
        figure('Name', 'Attitude Control Torques around z')
        hold on
        grid on
        for i = 2:N_deputy + 1
            
            plot(time, tau_out(3,:,i))
            
        end
        xlabel('Time $(s)$')
        ylabel('$\tau_z$ $(Nm)$')
        
    end
    
end

for i = 2:N_deputy+1
    
    figure('Name', ['Euler Angles Deputy ', num2str(i-1)])
    hold on
    grid on
    plot(time, eul_Hill2Body(3*i,:)*180/pi, 'r')
    plot(time, eul_Hill2Body(3*(i-1)+2,:)*180/pi, 'g')
    plot(time, unwrap(eul_Hill2Body(3*(i-1)+1,:))*180/pi, 'b')
    plot(time, unwrap(DesiredAttitude_out(3*i,:))*180/pi, 'r:')
    plot(time, unwrap(DesiredAttitude_out(3*(i-1)+2,:))*180/pi, 'g:')
    plot(time, unwrap(DesiredAttitude_out(3*(i-1)+1,:))*180/pi, 'b:')
    xlabel('Time (s)')
    ylabel('Euler Angles (deg)')
    legend('Yaw ($\varphi$)', 'Roll ($\theta$)','Pitch ($\psi$)')
    
end

for i = 2:N_deputy+1
    
    figure('Name', ['Angular Velocities Deputy ', num2str(i-1)])
    hold on
    grid on
    plot(time, omega_Body2Hill_Body(1+3*(i-1), :)*180/pi)
    plot(time, omega_Body2Hill_Body(2+3*(i-1), :)*180/pi)
    plot(time, omega_Body2Hill_Body(3+3*(i-1), :)*180/pi)
    xlabel('Time (s)')
    ylabel('$\omega_{B/H}^{BRF}$ (deg/s)')
    legend('$\omega_x$', '$\omega_y$', '$\omega_z$')
    
end


% figure('Name', 'Gravity Gradient Torque')
% hold on
% grid on
% plot(time(1:end-1), GG_torque_out(1,:), 'r')
% plot(time(1:end-1), GG_torque_out(2,:), 'g')
% plot(time(1:end-1), GG_torque_out(3,:), 'b')
% xlabel('Time (s)')
% ylabel('$\tau_{GG}$ ($N\,km$)')
% legend('$\tau_x$', '$\tau_y$', '$\tau_z$')
% 
% figure('Name', 'Aerodynamic Torque')
% grid on
% hold on
% plot(time(1:end-1), air_drag_torque_out(1,:), 'r')
% plot(time(1:end-1), air_drag_torque_out(2,:), 'g')
% plot(time(1:end-1), air_drag_torque_out(3,:), 'b')
% xlabel('Time (s)')
% ylabel('$\tau_{D}$ ($N\,km$)')
% legend('$\tau_x$', '$\tau_y$', '$\tau_z$')

for i = 2:N_deputy + 1
    
    figure('Name', ['Control Torque Deputy ', num2str(i-1)])
    hold on
    grid on
    plot(time, tau_PD_out(1,:,i), 'r')
    plot(time, tau_PD_out(2,:,i), 'g')
    plot(time, tau_PD_out(3,:,i), 'b')
    xlabel('Time (s)')
    ylabel('$\tau_{PD}$ ($N\,km$)')
    legend('$\tau_x$', '$\tau_y$', '$\tau_z$')
    
end