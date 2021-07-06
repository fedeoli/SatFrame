%% Montecarlo simulations 
function Montecarlo(nsim,pathname)
    path = pathname;

    counter = 1;
    while counter <= nsim
        file = strcat('/simulation_',int2str(counter));
        final_path = strcat(path,file);
        save temp
        
        clc
        disp(['Simulation: ',num2str(counter),'/',num2str(nsim)]);

        try
            SatFrame_init
            [DynOpt, params] = SatFrame(DynOpt);
        
            load temp
            save(final_path);
            clear
            load temp
            counter = counter + 1;
        catch
            load temp
        end
    end
    delete temp.mat
end
