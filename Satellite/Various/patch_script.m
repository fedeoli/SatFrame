%% plot formation
function patch_script(DynOpt)
    X = zeros(DynOpt.ObserverTest.Nagents,1);
    Y = zeros(DynOpt.ObserverTest.Nagents,1);
    Z = zeros(DynOpt.ObserverTest.Nagents,1);
    for i=1:DynOpt.ObserverTest.Nagents
        x = DynOpt.Xstory_pos_true(1+(i-1)*6,1);
        y = DynOpt.Xstory_pos_true(2+(i-1)*6,1);
        z = DynOpt.Xstory_pos_true(3+(i-1)*6,1);
        X(i) = x;
        Y(i) = y;
        Z(i) = z;
    end
    
    %%% plotting %%%
    xyz = [X,Y,Z];
  
  
    xyzc = mean(xyz,1);
    P = xyz - xyzc;
    [~,~,V] = svd(P,0);
    [~,is] = sort(atan2(P*V(:,1),P*V(:,2)));
    xyz = xyz(is([1:end 1]),:);
    
    box on
    grid on
    hold on
    fill3(xyz(:,1),xyz(:,2),xyz(:,3),[0.1 0.5 0.5]);
    plot3(xyz(:,1),xyz(:,2),xyz(:,3),'*b','LineWidth',2);
    alpha 0.3
    xlabel('x [Km]');
    ylabel('y [Km]');
    zlabel('z [Km]');
end