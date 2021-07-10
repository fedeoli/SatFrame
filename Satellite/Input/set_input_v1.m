%% Set input function
function [DynOpt,params] = set_input_v1(DynOpt,params)
    
    
    for n=1:DynOpt.ObserverTest.Nagents 
        
        %%% attitude input
        n_act = length(params.DesiredAttitude_default(:,n));
        t = DynOpt.time(DynOpt.iter);

        % set: T
        T = ceil(1/DynOpt.ObserverTest.u_freq);
        % duty cycle (ceil to avoid k=1???)
        k = floor((DynOpt.ObserverTest.d).*T);
        % module
        mod_u = mod(floor(t),T);

        % assign new one
        if mod_u < k
            DynOpt.ObserverTest.switch_pwm = 1;
        else
            DynOpt.ObserverTest.switch_pwm = -1;
        end

        % desired attitude
        temp_att = params.DesiredAttitude_default(:,n) + DynOpt.control*DynOpt.ObserverTest.u_amp.*DynOpt.ObserverTest.switch_pwm.*ones(n_act,1);        
        params.DesiredAttitude(:,n) =  temp_att;
        DynOpt.ObserverTest.target_attitude(DynOpt.iter,:,n) = temp_att;
    end
        
end