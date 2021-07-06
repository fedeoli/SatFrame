close all
for j = 1:N_deputy
    for i = 1: max(size(time))
        t = time(i);
        x_ref(1:6,i) = params.sat(j+1).TrajectoryProfile(t, params.sat(j+1));
    end
    % Radial Position
    figure(1)
    subplot(N_deputy,1,j)
    hold on
    plot(time_plot, deputy_rel_LVLH_alltimes(1,:,j),'r')
    plot(time_plot, x_ref(1,:),'k:')
    
    figure(2)
     subplot(N_deputy,1,j)
    hold on
    plot(time_plot, deputy_rel_LVLH_alltimes(2,:,j),'r')
    plot(time_plot, x_ref(2,:),'k:')
    
    figure(3)
     subplot(N_deputy,1,j)
    hold on
    plot(time_plot, deputy_rel_LVLH_alltimes(3,:,j),'r')
    plot(time_plot, x_ref(3,:),'k:')
end