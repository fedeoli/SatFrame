%%%%%%% Collision Probability Plot %%%%%%%
figure('units', 'normalized', 'outerposition', [0 0 1 1], 'Name', 'Collision Probability and relative distances')
grid on
hold on


%%%%%%%     Coll prob between deps      %%%%%%%
subplot(2,2,1)
hold on 
grid on
        
for j = 1:N_deputy
    
    for k = j+1:N_deputy
        
%         plot(time_plot_CP(1:end-1), reshape(CollisionProb(j,k+1,:)*100, [1 length(time_plot)-1]), 'color', colors(j,:))

        CollPlot = reshape(CollisionProb(j,k,:)*100, [1 length(time_plot)-1]);
        color_idx = j;
        
        for l = 1 : 1 : length(time) - 2
            
            if color_idx == j
                
                color_idx = k;
                
            else
                
                color_idx = j;
                
            end
            
            line('XData', time_plot_CP(l:l+1), 'YData', CollPlot(l:l+1), 'Color', colors(color_idx+1, :));
            
        end
        
    end
    
end

ttl = title('Collision Prob between deputies');
set(ttl, 'FontSize', 16, 'fontweight', 'bold');
xlabel(time_label)
ylabel('Collision Probability $(\%)$')


%%%%%%     Coll prob between deps and chief     %%%%%%%
subplot(2,2,2)
hold on
grid on
for j = 1:N_deputy
    
     plot(time_plot_CP(1:end-1), CollisionProb_vs_chief(j,:)*100, 'color', colors(j+1,:))
     
end

ttl = title('Collision Prob between deps and chief');
set(ttl, 'FontSize', 16, 'fontweight', 'bold');
xlabel(time_label)
ylabel('Collision Probability $(\%)$')
    

%%%%%%     Distances between deputies      %%%%%%%
subplot(2,2,3)
hold on
grid on
dist_rel = zeros(length(time), N_deputy*(N_deputy - 1)/2);
index = 1;

for j = 1:N_deputy
    
    for k = j+1 : N_deputy
        
        for i = 1:length(time)
            
            dist_rel(i,index) = norm(deputy_rel_LVLH_alltimes(1:3,i,j) - deputy_rel_LVLH_alltimes(1:3,i,k));
            
        end
        
        color_idx = j;
        
        for l = 1 : 20 : length(time) - 20
            
            if color_idx == j
                
                color_idx = k;
                
            else
                
                color_idx = j;
                
            end
            
            line('XData', time_plot(l:l+20), 'YData', dist_rel(l:l+20, index), 'Color', colors(color_idx+1, :));
            
        end
        
%         plot(time_plot, dist_rel(:,index), 'color', colors())
        index = index + 1;
        hold on
        
    end
    
end

ttl = title('Distance Between Deputies');
set(ttl, 'FontSize', 16, 'fontweight', 'bold');
xlabel(time_label)
ylabel('Relative Distances $(km)$')


%%%%%%     Distances between deps and chief      %%%%%%%
subplot(2,2,4)
hold on
grid on
dist_rel_chief = zeros(length(time), N_deputy*(N_deputy - 1)/2);
index = 1;

for j = 1:N_deputy
    
    for i = 1: length(time)
        
        dist_rel_chief(i,index) = norm(deputy_rel_LVLH_alltimes(1:3,i,j) );
        
    end
    
    plot(time_plot, dist_rel_chief(:,index), 'color', colors(j+1,:))
    index = index + 1;
    hold on
    
end

ttl = title('Distance between deps and chief');
set(ttl, 'FontSize', 16, 'fontweight', 'bold');
xlabel(time_label)
ylabel('Relative Distances $(km)$')
legend(arrayfun(@(N_dep) sprintf('Deputy %d', N_dep), 1:N_deputy, 'UniformOutput', false))