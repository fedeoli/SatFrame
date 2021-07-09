%% function
function x = quatnormalize_fleet(x,DynOpt)
    for i=1:DynOpt.ObserverTest.Nagents
        x(1+7*(i-1):4+7*(i-1),end) = quatnormalize(transpose(x(1+7*(i-1):4+7*(i-1),end)));
    end
end