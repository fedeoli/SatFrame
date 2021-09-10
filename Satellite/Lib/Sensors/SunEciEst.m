%% function 
function [SEci_est, SEci_inv, SEci_mean] = SunEciEst(x_att, pos, S, DynOpt, params)

    if DynOpt.ObserverTest.Sun == 1
        % convert from ECI to latitude, longitude,  altitude
        pos = pos(1:3);
        quat = x_att(1:4);

        % handle attitude
        q_ECI2Body =  quat; 
        R_ECI2Body = quat2dcm(transpose(q_ECI2Body));

        S_body = Sun_measure(x_att,pos,DynOpt,params);
        SEci_est = pinv(R_ECI2Body)*S_body;


        % S ECI from measure
        SEci_inv = pinv(R_ECI2Body) * S;

        % store
        SEci_mean = mean([SEci_est; SEci_inv],2);
    else
        SEci_est = [];
        SEci_inv = [];
        SEci_mean = [];
    end
end