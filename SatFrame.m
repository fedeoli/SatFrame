function [DynOpt, params] = SatFrame(DynOpt)

%% Init Section
close all
if (DynOpt.print) && (~DynOpt.montecarlo)
    clc
end

% new random seed[DynOpt, params] = SatFrame(struct)
if DynOpt.randstart
    rng('shuffle');
else
    rng('default');
end

%%% global variables %%%
params = [];

% wrap function
DynOpt.wrap = @wrapTo180;

% ode
DynOpt.ode = @ode23;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% satellite init
[DynOpt, params,~,~] = satellite_init_function(DynOpt, params, DynOpt);

%%% DEFINE MODEL NAME %%%
DynOpt.model_attitude = @AttitudeDynamics_v1_function;
DynOpt.model_inertial = @InertialDynamicsIntegrator_V2_2_function;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Niter = length(DynOpt.time);
for i = 1:Niter
    
    % Display iteration step
    if (~DynOpt.montecarlo) && mod(i,10) == 0
        clc
        disp(['Iteration Number: ', num2str(i),'/',num2str(Niter)])
    end
    
    % DynOpt current iter 
    DynOpt.iter = i;
    params.t = DynOpt.time(i);
    
    %%%%%%%%%%%%%%%%%%%% MODEL INTEGRATION %%%%%%%%%%%%%%%%%%%%
    if DynOpt.iter > 1
        % Control allocation inside "params" structure
        params.u = DynOpt.control*params.u;
        [DynOpt,params] = set_input_v1(DynOpt,params);
        params.tau = DynOpt.control*AttitudeControl_V2_5(DynOpt.Xstory_att_est(:,DynOpt.iter-1), DynOpt.Xstory_pos_est(:,DynOpt.iter-1), DynOpt.time(DynOpt.iter), params);
        params.tspan = DynOpt.time(i):DynOpt.time_step:DynOpt.time(i) + DynOpt.time_step;
    end
    
    %%%%%%%%%%%%%%%%%%%% ORBITAL PROPAGATION %%%%%%%%%%%%%%%%%%%%
    % forward propagation of the previous estimate
    if(DynOpt.iter > 1)
        X = DynOpt.ode(@(t,x)DynOpt.model_inertial(t, x, params, DynOpt), params.tspan, DynOpt.Xstory_pos_true(:,DynOpt.iter-1));   
        DynOpt.Xstory_pos_true(:,DynOpt.iter) = X.y(:,end);
        
        X = DynOpt.ode(@(t,x)DynOpt.model_inertial(t, x, params, DynOpt), params.tspan, DynOpt.Xstory_pos_est(:,DynOpt.iter-1));
        DynOpt.Xstory_pos_est(:,DynOpt.iter) = X.y(:,end);
        DynOpt.Xstory_pos_wrong(:,DynOpt.iter) = X.y(:,end);
        
        %%% general info - true %%%
        satellites_iner_ECI = DynOpt.Xstory_pos_est(:,DynOpt.iter);
        params.SatellitesCoordinates = satellites_iner_ECI;        
    end
    
    %%%%%%%%%%%%%%%%%%%% ATTITUDE PROPAGATION %%%%%%%%%%%%%%%%%%%
    % forward propagation of the previous estimate
    if(DynOpt.iter > 1)
        X = DynOpt.ode(@(t,x)DynOpt.model_attitude(t, x, DynOpt.Xstory_pos_true(:,DynOpt.iter), params, DynOpt), params.tspan, DynOpt.Xstory_att_true(:,DynOpt.iter-1));
        X.y = quatnormalize_fleet(X.y,DynOpt);
        DynOpt.Xstory_att_true(:,DynOpt.iter) = X.y(:,end);
        
        X = DynOpt.ode(@(t,x)DynOpt.model_attitude(t, x, DynOpt.Xstory_pos_est(:,DynOpt.iter), params, DynOpt), params.tspan, DynOpt.Xstory_att_est(:,DynOpt.iter-1));
        X.y = quatnormalize_fleet(X.y,DynOpt);
        DynOpt.Xstory_att_est(:,DynOpt.iter) = X.y(:,end);
        DynOpt.Xstory_att_wrong(:,DynOpt.iter) = X.y(:,end);
        
        %%% general info - true %%%
        satellites_attitude = DynOpt.Xstory_att_est(:,DynOpt.iter);
        params.satellites_attitude = satellites_attitude;        
    end
    
    %%%%%%%%%%%%%%%%%%% REAL MEASUREMENT %%%%%%%%%%%%%%%%%%%%%%%    
    DynOpt = Observer_Measurements_v4(DynOpt.Xstory_pos_true(:,DynOpt.iter), params, DynOpt);
    DynOpt = Observer_Measurements_attitude_v1(DynOpt.Xstory_pos_true(:,DynOpt.iter),DynOpt.Xstory_att_true(:,DynOpt.iter), DynOpt, params);
    
    %%%%%%%%%%%%%%%%%%%%%% OBSERVER %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% position %%%
    if (DynOpt.ObserverOn_pos) && (DynOpt.iter > 1)
        [DynOpt, params] = SatFrame_observer_pos(DynOpt, params);
    end
    %%% attitude %%%
    if (DynOpt.ObserverOn_att) && (DynOpt.iter > 1)
        [DynOpt, params] = SatFrame_observer_att(DynOpt, params);
    end

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% data analysis %%%
DynOpt = store_observer_v3(DynOpt,params,0.4,0);

end

