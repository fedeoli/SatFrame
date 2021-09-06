%% test sigma analysis in theory

%%%%%%%%%%%%%%% 2 %%%%%%%%%%%%%%%%%
% surf the simulations
dirname = 'simulations/Singleshot/position/GPSsigma/';
fileinfo = dir(strcat(dirname,'*.mat'));
fnames = {fileinfo.name};
for s = 1:length(fnames)
    load(strcat(dirname,fnames{s}));
    
    sigma_p_mean = mean(DynOpt.sigma_p_mean_all);
    sigma_p_mean_exp = mean(DynOpt.sigma_p_mean_all_exp);

    x(s,1) = DynOpt.ObserverTest.Nagents;
    y(s,1) = DynOpt.ObserverTest.theta;
    z(s,1) = sigma_p_mean;
    z_exp(s,1) = sigma_p_mean_exp;
end

f = fit( [x, y], z, 'poly41' );
f_exp = fit( [x, y], z_exp, 'poly41' );

freezeColors;
plot(f, [x,y], z);
colormap autumn
hold on

freezeColors;
plot(f_exp, [x,y], z_exp);
colormap winter


legend('Theo','Exp','Data')
xlabel('N')
ylabel('\theta')
zlabel('\Sigma')
