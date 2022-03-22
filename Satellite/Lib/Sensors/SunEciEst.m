%% function 
function [S_meas, S_est, S_mean] = SunEciEst(x_att, pos, S, DynOpt, params, ECI)

    if DynOpt.ObserverTest.Sun == 1
        % convert from ECI to latitude, longitude,  altitude
        pos = pos(1:3);
        quat = x_att(1:4);

        % handle attitude
        q_ECI2Body =  quat; 
        R_ECI2Body = quat2dcm(transpose(q_ECI2Body));

        S_body = Sun_measure(x_att,pos,DynOpt,params);
        S_body = S_body/norm(S_body);
        
        if ECI
            S_est = pinv(R_ECI2Body)*S_body;
            
            % S ECI from measure
            S_meas = pinv(R_ECI2Body) * S;

            % store
            S_mean = mean([S_est, S_meas],2);
            
        else
            
            S_est = S_body;
            
            % S ECI from measure
            S_meas = S;

            % store
            S_mean = mean([S_est, S],2);
        end        
    else
        S_est = zeros(1,3);
        S_meas = zeros(1,3);
        S_mean = zeros(1,3);
    end

end