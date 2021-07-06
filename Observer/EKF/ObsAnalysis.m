%% observability analysis on the nonlinear system (attitude)
function [theta,dtheta,dtheta_num] = ObsAnalysis(DynOpt,n_der,x,y,calc)

    if calc == 1
        
        % init
        X = DynOpt.obs.X;
        f = DynOpt.obs.f;
        h = DynOpt.obs.h;
        dimState = length(X);
        dimOut = length(h);

        %%%%%%% COMPUTE THETA %%%%%%%
        n_iter = n_der;
        for k=1:n_iter
            if k==1
               % init theta
               theta = h; 
            else
               for z=1:dimOut
                   for i=1:dimState
                        theta_temp(z,i) = diff(theta(z+(k-2)*dimOut),X(i));
                   end
               end
               theta = vpa([theta; theta_temp*f],2); 
            end
        end

        %%%%%% COMPUTE DTHETA %%%%%%%%
        dimTheta = length(theta);
        for k=1:dimTheta
            for i=1:dimState
                dtheta(i,k) = diff(theta(k),X(i));
            end
        end
        dtheta = vpa(dtheta,2);
        dtheta_num = 0;
    else
        dimOut = 3*DynOpt.ObserverTest.nMagneto;
        
        %%%%% COMPUTE RANK %%%%%%%%%
        dtheta_num = subs(DynOpt.obs.dtheta,DynOpt.obs.X,transpose(x));
        dtheta_num = subs(dtheta_num,DynOpt.obs.Magneto(1:dimOut),transpose(y(1:dimOut)));
        dtheta_num = double(dtheta_num);
        theta = 0;
        dtheta = 0;
    end
    
    
end

