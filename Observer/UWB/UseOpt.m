%% use GPS opt if no KF
function [DynOpt, params] = UseOpt(DynOpt, params)

    for k = 1:DynOpt.ObserverTest.Nagents
        DynOpt.Xstory_pos_est(1+6*(k-1):6+6*(k-1),DynOpt.iter) = DynOpt.y_GPS(1+6*(k-1):6+6*(k-1),DynOpt.iter);
    end
end