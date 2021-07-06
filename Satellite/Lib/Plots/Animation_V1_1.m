if animation_on
    
    % Plot layout
    filename = 'gif\testAnimated.gif';
    set(0, 'defaultAxesFontSize', 18)
    set(0, 'defaultAxesFontWeight', 'bold')
    set(0, 'defaultAxesGridAlpha', 1)
    set(0, 'defaultFigureColor', 'white')
    set(0, 'defaultLineLineWidth', 2)
    set(groot, 'defaulttextinterpreter', 'latex');
    set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
    set(groot, 'defaultLegendInterpreter', 'latex');
    DV_plot = [];

    % Compute vectors dimensions and colors
    if strcmpi(params.ControlStrategy, 'Impulsive')
        
        u_plot = params.DV;
        
    elseif strcmpi(params.ControlStrategy, 'Continuous')
        
        u_plot = u_out;
        
    end
    
    dim_x = max(max(abs(deputy_rel_LVLH_alltimes(1,:,:)))*1.2);
    dim_y = max(max(abs(deputy_rel_LVLH_alltimes(2,:,:)))*1.2);
    dim_z = max(max(abs(deputy_rel_LVLH_alltimes(3,:,:)))*1.2);
    dim_quiver = min([dim_x, dim_y, dim_z]);  
    dim_dv = dim_quiver/max(max(max(abs(u_plot))));
    colors = abs(rand(N_deputy, 3));
    colors(1:3,:) = [3.170994800608605e-01     4.387443596563982e-01     7.951999011370632e-01;
                    9.502220488383549e-01     3.815584570930084e-01     1.868726045543786e-01;
                    9.592914252054443e-01     1.492940055590575e-01     2.542821789715310e-01];
    max_tail = 50;
    step_anim = 10;
    DV_non_zero = 0;
    delay = 0.02;
    contr_flag = zeros(N_deputy, tlength);
    
    % Initialize Figure
    h = figure('Name', 'Animation', 'units', 'normalized', 'outerposition', [0 0 1 1]);
    hold on
    axis equal
    axis([-dim_x, dim_x, -dim_y, dim_y, -dim_z, dim_z])
    view(-63,51);
    plot3(0, 0, 0, 'ko');
    xlabel('Radial distance (km)')
    ylabel('Along track distance (km)')
    zlabel('Cross track distance (km)')
    quiver3(0, 0, 0, dim_quiver, 0, 0, 'color', 'r', 'LineWidth', 2)
    quiver3(0, 0, 0, 0, dim_quiver, 0, 'color', 'g', 'LineWidth', 2)
    quiver3(0, 0, 0, 0, 0, dim_quiver, 'color', 'b', 'LineWidth', 2)

    for k = 1:step_anim:length(time)
        
        for i = 1:N_deputy
            
            if k <= max_tail
                
                Ntail = k - 1;
                
            else
                
                Ntail = max_tail;
                
            end
            
            tail(i) = plot3(deputy_rel_LVLH_alltimes(1, k-Ntail:k, i), deputy_rel_LVLH_alltimes(2, k-Ntail:k, i), deputy_rel_LVLH_alltimes(3, k-Ntail:k, i), 'color', colors(i,:));
            deputy(i) = plot3(deputy_rel_LVLH_alltimes(1, k, i), deputy_rel_LVLH_alltimes(2, k, i), deputy_rel_LVLH_alltimes(3, k, i), 'd', 'color', colors(i,:));
            control_plot(i,k) = quiver3(deputy_rel_LVLH_alltimes(1, k, i), deputy_rel_LVLH_alltimes(2, k, i), deputy_rel_LVLH_alltimes(3, k, i), dim_dv*u_plot(1, k, i), dim_dv*u_plot(2, k, i) , dim_dv*u_plot(3, k, i), 'color', 'k', 'LineWidth', 2);
            
            if norm(u_plot(:,k,i)) ~= 0
                
                contr_flag(i,k) = 1;
                
            end
            
        end
        
        pause(0.00001);
        
        % Write to the GIF File
        
        if createGIF == 1
            
            frame = getframe(h);
            im = frame2im(frame);
            [imind,cm] = rgb2ind(im,256);
        
            if k == 1
                
                imwrite(imind, cm, filename, 'gif', 'LoopCount', Inf, 'DelayTime', delay);
                
            else
                
                imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', delay);
                
            end
            
        end
        
        delete(tail);
        delete(deputy);
        
        for i = 1:N_deputy
            
            if strcmpi(params.ControlStrategy, 'Impulsive') && contr_flag(i,k-(k>50)*50) == 1
                
                delete(control_plot(i,k-(k>50)*50));
                
            elseif strcmpi(params.ControlStrategy, 'Continuous') || norm(u_plot(:,k,i)) == 0
                
                delete(control_plot(i,k));
                
            end
            
        end
        
    end
    
end
