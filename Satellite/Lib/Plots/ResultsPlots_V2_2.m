%%%%%%% Set the plot preferences %%%%%%%

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

% Time vector format
if strcmpi(params.PlotTimeFormat, 'hours')
    
    time_plot = time./3600;
    time_plot_CP = time_plot + params.DT_CollisionCheck/3600;
    time_label = 'Time $(h)$';
    
elseif strcmpi(params.PlotTimeFormat, 'seconds')
    
    time_plot = time;
    time_plot_CP = time_plot + params.DT_CollisionCheck;
    time_label = 'Time $(s)$';
    
end


%%%%%%% Orbital Plots %%%%%%%
OrbitalPlot_V2_1;


%%%%%%% Attitude Plots %%%%%%%
if params.Attitude
    
    AttitudePlot_V2_2;
    
end


%%%%%%% Collision Probability Plot %%%%%%%
figure('Name', 'Collision Probability')
grid on
hold on

for j = 1:N_deputy
    
    for k = j:N_deputy-1
        
        plot(time_plot_CP(1:end-1), reshape(CollisionProb(j,k+1,:)*100, [1 length(time_plot)-1]), 'r')
        
    end
    
     plot(time_plot_CP(1:end-1), CollisionProb_vs_chief(j,:)*100, 'k')
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


%%%%%%% True vs Reference Trajectories %%%%%%%
DeputyReferenceTrajPlot_V2_1;


%%%%%%% 3D Plot %%%%%%%
figure('Name', '3D Plot')
hold on
axis equal

for k = 1:N_deputy
    
    plot3(deputy_rel_LVLH_alltimes(1,:,k), deputy_rel_LVLH_alltimes(2,:,k), deputy_rel_LVLH_alltimes(3,:,k))
    
end

xlabel('x ($km$)')
ylabel('y ($km$)')
zlabel('z ($km$)')


%%%%%%% Animation %%%%%%%
Animation_V1_1;
