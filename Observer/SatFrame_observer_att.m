%% function 
function [DynOpt, params] = SatFrame_observer_att(DynOpt, params)

    %%%%%%%%%%%%%%%%%% KALMAN FILTERING %%%%%%%%%%%%%%%%%%%%%%%
    if DynOpt.ObserverTest.KF_flag == 1
        tic
        [DynOpt, params] = Observer_EKF_att_v1(DynOpt, params);
        DynOpt.ObserverTest.KFtime_att(DynOpt.iter) = toc;
    end
    
end
