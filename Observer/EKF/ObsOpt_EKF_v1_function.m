%% ObsOpt implementation
function [DynOpt_out, params_out] = ObsOpt_EKF_v1_function(DynOpt,params)

%%% init EKF %%%
disp('INIT EKF')
[DynOpt,params] = SymAnalysis_RL_v3;

%%% Covariance matrix init %%%
DynOpt.P0 = 1e-2*eye(7); 
DynOpt.P = DynOpt.P0;
DynOpt.Preset = 0;

%%% Process noise %%%
DynOpt.Q = 1e-2*eye(7);
% DynOpt.Q = 1e0*[1e-3*ones(1,4), 1e-3*ones(1,3)].*eye(7);

%%% Measurement noise %%%
% DynOpt.R = 1*diag(DynOpt.measure_amp);
DynOpt.R = 1e-3*eye(DynOpt.dim_out);

%%% manage input %%%
DynOpt.recollect_input = 1;

%%%%%%%%%% Simulation process %%%%%%%%%%%
disp('Processing data with the optimization-based observer...')
run_time = tic;
for k=1:length(DynOpt.time)
    
    % update actual index
    DynOpt.ActualTimeIndex = k;

    % reference state - used for noise
    DynOpt.Xtrue = [DynOpt.state(:,k);DynOpt.param_story(:,k)];

    %forward propagation of the previous estimate
    if(k>1)
        %%% true state - just for test %%%
        DynOpt.Xtrue_past = [DynOpt.state(:,k-1);DynOpt.param_story(:,k-1)];
        
        % INTEGRATION OF BOTH POSITION AND ATTITUDE - STACK 
        % Control allocation inside "params" structure         
        x_start = DynOpt.OptXstory(:,DynOpt.ActualTimeIndex-1);
        [DynOpt.X, params] = DynOpt.model_propagate(DynOpt.ActualTimeIndex,DynOpt.Ts,x_start,params);
        DynOpt.OptXstory(:,k) = DynOpt.X; 
        DynOpt.OptXstory_runtime(:,k) = DynOpt.X;

        [temp_Xstory, params] = DynOpt.model_propagate(DynOpt.ActualTimeIndex,DynOpt.Ts,DynOpt.Xstory(:,DynOpt.ActualTimeIndex-1),params);
        DynOpt.Xstory(:,k) = temp_Xstory; 
    else
        DynOpt.OptXstory(:,k) = DynOpt.X_init;
    end

    %%%%%%%%%%%%%%%%%%% REAL MEASUREMENT %%%%%%%%%%%%%%%%%%%%%%%    
    % read measure 
    measure_forward = 1;
    [DynOpt.buf_dy,Y_true] = DynOpt.get_measure(DynOpt.Xtrue,0,measure_forward,DynOpt.buf_dy,DynOpt.intY_full_story,params,DynOpt.ActualTimeIndex);
    % copy to Y noise and corrupt only the measure 
    Y_noise = noise_model_v1(Y_true,DynOpt,params);     

    % no filtering
    Y_filter = Y_noise;

    % store total memory
    DynOpt.Ytrue_full_story(:,end+1) = Y_true(:,1);
    DynOpt.Y_full_story(:,end+1) = Y_filter(:,1);
    DynOpt.dY_full_story(:,end+1) = Y_filter(:,2);
    DynOpt.intY_full_story(:,end+1) = Y_filter(:,3);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%% ESTIMATED MEASUREMENT %%%%%%%%%%%%%%%%%%
    measure_forward = 1;
    [DynOpt.buf_dyhat, Yhat] = DynOpt.get_measure(DynOpt.OptXstory(:,DynOpt.ActualTimeIndex),0,measure_forward,DynOpt.buf_dyhat,DynOpt.intYhat_full_story,params,DynOpt.ActualTimeIndex);
    DynOpt.Yhat_full_story(:,end+1) = Yhat(:,1);
    DynOpt.dYhat_full_story(:,end+1) = Yhat(:,2);
    DynOpt.intYhat_full_story(:,end+1) = Yhat(:,3);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN FILTER %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if 1 && k>1
        pos = DynOpt.ActualTimeIndex;
        z = DynOpt.Y_full_story(:,pos);
        z_hat = DynOpt.Yhat_full_story(:,pos);
        %%%%%%%%%%%% TEST %%%%%%%%%%%%%
        z_hat = [z(1:3);z_hat(4:end)];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        u = params.tau;
        x_past = DynOpt.OptXstory(:,pos-1);
        x_now = DynOpt.OptXstory(:,pos);
        [xnew, Pnew, DynOpt] = Observer_EKF_v3(DynOpt, params, x_past, x_now, z, z_hat, u);
        DynOpt.OptXstory(:,pos) = xnew;
        DynOpt.P = Pnew;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%% PERFORMANCE EVALUATION %%%%%%%%%%%%%%%%%
    if k>1 
        % fisrt bunch of data - read Y every Nts and check if the signal is
        dJ_cond_v2(DynOpt.theta,DynOpt.beta,DynOpt.gamma);

        % clean 
        clc    
        % Display iteration slengthtep
        disp(['Iteration Number: ', num2str(k),'/',num2str(length(DynOpt.time))])
        disp(['Last DJcond: ', num2str(DynOpt.dJ_cond)]);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
end
DynOpt.run_time = toc(run_time);

% output
DynOpt_out = DynOpt;
params_out = params;

end