%% function 
function [DynOpt, params] = SatFrame_observer_pos(DynOpt, params)
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% Observer - integration of position dynamics - init xhatUKF %%%%%%%
    X = DynOpt.ode(@(t,x)DynOpt.model_inertial(t, x, params, DynOpt), params.tspan, DynOpt.Xstory_pos_est(:,DynOpt.iter-1));
    for k = 1:DynOpt.ObserverTest.Nagents  
        DynOpt.Xstory_pos_est(1+6*(k-1):6+6*(k-1),DynOpt.iter) =  X.y(1+6*(k-1):6+6*(k-1),end);
        DynOpt.ObserverTest.APrioriEstimationXYZ(1+3*(k-1):3+3*(k-1)) = DynOpt.Xstory_pos_est(1+6*(k-1):3+6*(k-1),DynOpt.iter);
        DynOpt.ObserverTest.APrioriEstimationVel(1+3*(k-1):3+3*(k-1)) = DynOpt.Xstory_pos_est(4+6*(k-1):6+6*(k-1),DynOpt.iter);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%% DYNAMIC WEIGHTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if 1 || DynOpt.ObserverTest.dcond_mean(end) >= DynOpt.ObserverTest.dcond_thresh
        DynOpt.ObserverTest.theta = 0.1;
        DynOpt.ObserverTest.beta = 0;
    else
        DynOpt.ObserverTest.theta = 0.005;
        DynOpt.ObserverTest.beta = 0;
    end

    % save story
    DynOpt.ObserverTest.beta_story = [DynOpt.ObserverTest.beta_story, DynOpt.ObserverTest.beta];
    DynOpt.ObserverTest.theta_story = [DynOpt.ObserverTest.theta_story, DynOpt.ObserverTest.theta];
   
    %%%%%%%%%%%%%%% GPS OPTIMIZATION %%%%%%%%%%%%%%%%%%%%%%%%%
    if DynOpt.ObserverTest.GPSopt_flag == 1
        tic
        DynOpt = GPS_Optimization_V2_geometric(DynOpt);
        DynOpt.ObserverTest.GPStime(DynOpt.iter) = toc;
    end

    %%%%%%%%%%%%%%%%%% KALMAN FILTERING %%%%%%%%%%%%%%%%%%%%%%%
    if DynOpt.ObserverTest.KF_flag == 1
        tic
        if strcmp(DynOpt.ObserverTest.KF_pos,'UKF')
            [DynOpt,params] = Position_UKF_V1_6(DynOpt,params);
        elseif strcmp(DynOpt.ObserverTest.KF_pos,'EKF')
            [DynOpt, params] = Observer_EKF_pos_v1(DynOpt, params);
        else
            disp('wrong filter selection')
            return
        end
        DynOpt.ObserverTest.KFtime_pos(DynOpt.iter) = toc;
    else
        [DynOpt, params] = UseOpt(DynOpt, params);
    end
    
    %%%%%%%%%%%%%%% SIGMA ANALYSIS %%%%%%%%%%%%%%%%%
    
    
end
