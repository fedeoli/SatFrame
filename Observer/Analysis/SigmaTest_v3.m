%% test sigma analysis in theory
function SigmaTest_v3(dirname)
    %%%%%%%%%%%%%%% 2 %%%%%%%%%%%%%%%%%
    % surf the simulations
    fileinfo = dir(strcat(dirname,'*.mat'));
    fnames = {fileinfo.name};

    x= zeros(length(fnames),1);
    y= zeros(length(fnames),1);
    z= zeros(length(fnames),3);
    z_exp= zeros(length(fnames),3);

    for s = 1:length(fnames)
        load(strcat(dirname,fnames{s}));

        sigma_p_mean = sort(DynOpt.sigma_p_mean_all);
        sigma_p_mean_exp = sort(DynOpt.sigma_p_mean_all_exp);

        x(s,1) = DynOpt.ObserverTest.Nagents;
        y(s,1) = DynOpt.ObserverTest.theta;
        z(s,:) = sigma_p_mean;
        z_exp(s,:) = sigma_p_mean_exp;
    end

    figure()
    label_y = {'\Sigma_x','\Sigma_y','\Sigma_z'};
    for i=1:3
        subplot(3,1,i)
        plot(y, z(:,i), 'b-');
        hold on
        plot(y, z_exp(:,i), 'b--+');
        ylim([2.5e-3, 6e-3])

        legend('Theo','Exp')
        xlabel('\theta')
        ylabel(label_y{i})
    end
end
