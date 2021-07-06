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

if animation_on
    
    figure('Name', 'Animation', 'units', 'normalized', 'outerposition', [0 0 1 1])
    dim_x = max(max(abs(deputy_rel_LVLH_alltimes(1,:,:)))*1.2);
    dim_y = max(max(abs(deputy_rel_LVLH_alltimes(2,:,:)))*1.2);
    dim_z = max(max(abs(deputy_rel_LVLH_alltimes(3,:,:)))*1.2);
    dim_quiver = min([dim_x, dim_y, dim_z]);
    colors = abs(rand(N_deputy, 3));
    hold on
    axis equal    
    axis([-dim_x, dim_x, -dim_y, dim_y, -dim_z, dim_z])
    view(-63,51);
    quiver3(0, 0, 0, dim_quiver, 0, 0, 'color', 'r', 'LineWidth', 2)
    quiver3(0, 0, 0, 0, dim_quiver, 0, 'color', 'g', 'LineWidth', 2)
    quiver3(0, 0, 0, 0, 0, dim_quiver, 'color', 'b', 'LineWidth', 2)
    
    for k = 1:3:length(time)
        
        
        for i = 1:N_deputy
            
            if k <= 50
                
                Ntail = k - 1;
                
            else
                
                Ntail = 50;
                
            end
            
            tail(i) = plot3(deputy_rel_LVLH_alltimes(1, k-Ntail:k, i), deputy_rel_LVLH_alltimes(2, k-Ntail:k, i), deputy_rel_LVLH_alltimes(3, k-Ntail:k, i), 'color', colors(i,:));
            deputy(i) = plot3(deputy_rel_LVLH_alltimes(1, k:k, i), deputy_rel_LVLH_alltimes(2, k:k, i), deputy_rel_LVLH_alltimes(3, k:k, i), 'd', 'color', colors(i,:));
            plot3(0, 0, 0, 'ko');
            
            xlabel('Radial distance (km)')
            ylabel('Along track distance (km)')
            zlabel('Cross track distance (km)')
            
        end
        
        pause(0.01)
        delete(tail);
        delete(deputy);
        
    end
    
end
