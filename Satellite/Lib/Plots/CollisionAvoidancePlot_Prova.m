%%%%%%% Collision Probability Plot %%%%%%%
figure('Name', 'Collision Probability between deputies')
grid on
hold on

subplot(2,2,1)
for j = 1:N_deputy
    
    for k = j:N_deputy-1
        
        plot(time_plot_CP(1:end-1), reshape(CollisionProb(j,k+1,:)*100, [1 length(time_plot)-1]), 'k')
%         plot(time_plot_CP(1:end-1), reshape(prob_appr(j,k+1,:)*100, [1 length(time_plot)-1]), 'r')
%         plot(time_plot_CP(1:end-1), reshape(prob_appr2(j,k+1,:)*100, [1 length(time_plot)-1]), 'r')

%                         plot(time_plot_CP(1:end-1), reshape(CollisionProb(j,k+1,:)*100 - prob_appr(j,k+1,:)*100, [1 length(time_plot)-1]), 'k')
        
    end
    
end

xlabel(time_label)
ylabel('Collision Probability $(\%)$')

subplot(2,2,2)
for j = 1:N_deputy
    
    for k = j:N_deputy-1
        
%         plot(time_plot_CP(1:end-1), reshape(CollisionProb(j,k+1,:)*100, [1 length(time_plot)-1]), 'k')
        plot(time_plot_CP(1:end-1), reshape(prob_appr(j,k+1,:)*100, [1 length(time_plot)-1]), 'r')
%         plot(time_plot_CP(1:end-1), reshape(prob_appr2(j,k+1,:)*100, [1 length(time_plot)-1]), 'r')

%                         plot(time_plot_CP(1:end-1), reshape(CollisionProb(j,k+1,:)*100 - prob_appr(j,k+1,:)*100, [1 length(time_plot)-1]), 'k')
        
    end
    
end

xlabel(time_label)
ylabel('Collision Probability $(\%)$')


% figure('Name', 'Collision Probability vs chief')
% grid on
% hold on
subplot(2,2,3)
for j = 1:N_deputy
    
     plot(time_plot_CP(1:end-1), CollisionProb_vs_chief(j,:)*100, 'k')
%      plot(time_plot_CP(1:end-1), prob_appr_chief(j,:)*100, 'r')
%      plot(time_plot_CP(1:end-1), prob_appr_chief2(j,:)*100, 'r')

%      plot(time_plot_CP(1:end-1), CollisionProb_vs_chief(j,:)*100 - prob_appr_chief(j,:)*100, 'k')

     
end

xlabel(time_label)
ylabel('Collision Probability $(\%)$')

subplot(2,2,4)
for j = 1:N_deputy
    
%      plot(time_plot_CP(1:end-1), CollisionProb_vs_chief(j,:)*100, 'k')
     plot(time_plot_CP(1:end-1), prob_appr_chief(j,:)*100, 'r')
%      plot(time_plot_CP(1:end-1), prob_appr_chief2(j,:)*100, 'r')

%      plot(time_plot_CP(1:end-1), CollisionProb_vs_chief(j,:)*100 - prob_appr_chief(j,:)*100, 'k')

     
end

xlabel(time_label)
ylabel('Collision Probability $(\%)$')

%%%%%%% Relative Distances Plot %%%%%%%
figure('Name', 'Deputies RelativeDistances')
dist_rel = zeros(length(time), N_deputy*(N_deputy - 1)/2);
index = 1;

for j = 1:N_deputy
    
    for k = j : N_deputy - 1
        
        for i = 1: length(time)
            
            dist_rel(i,index) = norm(deputy_rel_LVLH_alltimes(1:3,i,j) - deputy_rel_LVLH_alltimes(1:3,i,k+1));
            
        end
        
        plot(time_plot, dist_rel(:,index))
        index = index + 1;
        hold on
        
    end
    
end

xlabel(time_label)
ylabel('Relative Distances $(km)$')


% Distance of each deputy from the chief
figure('Name', 'Chief Relative Distances')
dist_rel = zeros(length(time), N_deputy*(N_deputy - 1)/2);
index = 1;

for j = 1:N_deputy
    
    for i = 1: length(time)
        
        dist_rel(i,index) = norm(deputy_rel_LVLH_alltimes(1:3,i,j) );
        
    end
    
    plot(time_plot, dist_rel(:,index))
    index = index + 1;
    hold on
    
end

xlabel(time_label)
ylabel('Relative Distances $(km)$')