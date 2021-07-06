for j = 1:N_deputy
    
    fig_name = ['Deputy ', num2str(j), ' real vs reference trajectory'];
    
    x_refj = zeros(6, tlength);
    
    for i = 1:tlength
        
        x_refj(:,i) = params.sat(j+1).TrajectoryProfile(time(i), params.sat(j+1));
        
    end
    
    figure('Name', fig_name)
    
    % Radial Position
    subplot(3,1,1)
    hold on
    plot(time_plot, deputy_rel_LVLH_alltimes(1,:,j), 'color', colors(j+1,:))
    plot(time_plot, x_refj(1,:), 'color', colors(j+1,:), 'LineStyle', ':')
    xlabel(time_label)
    ylabel('$x$ $(km)$')

    subplot(3,1,2)
    hold on
    plot(time_plot, deputy_rel_LVLH_alltimes(2,:,j), 'color', colors(j+1,:))
    plot(time_plot, x_refj(2,:), 'color', colors(j+1,:), 'LineStyle', ':')
    xlabel(time_label)
    ylabel('$y$ $(km)$')
    
    subplot(3,1,3)
    hold on
    plot(time_plot, deputy_rel_LVLH_alltimes(3,:,j), 'color', colors(j+1,:))
    plot(time_plot, x_refj(3,:), 'color', colors(j+1,:), 'LineStyle', ':')
    xlabel(time_label)
    ylabel('$z$ $(km)$')
    
end
