%% Attitude_Obsever_V2_1: estimates the i-th satellite attitude using MAHONY
function  [DynOpt, params] = Observer_EKF_att_v1(DynOpt, params)

    %%% a priori estimate
    x_past_full = DynOpt.Xstory_att_est(:,DynOpt.iter-1);
    pos_full = DynOpt.Xstory_pos_est(:,DynOpt.iter);
    tmp_full = DynOpt.ode(@(t,x)DynOpt.model_attitude(t, x, pos_full, params, DynOpt), params.tspan, x_past_full);
    tmp_full.y = quatnormalize_fleet(tmp_full.y,DynOpt);
    xhat_now_full = tmp_full.y(:,end);
    

    for k = 1:DynOpt.ObserverTest.Nagents

        %%%% INIT SECTION %%%%

        %%%%%%%%%%%%%%%%%%%% CURRENT GPS %%%%%%%%%%%%%%%%%
        Mag_1 = reshape(DynOpt.y_Mag(1+6*(k-1):3+6*(k-1),DynOpt.iter),3,1);
        Mag_2 = reshape(DynOpt.y_Mag(4+6*(k-1):6+6*(k-1),DynOpt.iter),3,1);
        Gyro = reshape(DynOpt.y_Gyro(1+3*(k-1):3+3*(k-1),DynOpt.iter),3,1);
        
        if DynOpt.ObserverTest.nMagneto == 0
            z_now = Gyro;
        elseif DynOpt.ObserverTest.nMagneto == 1
            z_now = [Mag_1; Gyro];
        elseif DynOpt.ObserverTest.nMagneto == 2
            z_now = [Mag_1; Mag_2; Gyro];
        end
        
        %%% past state %%%
        x_past = x_past_full(1+7*(k-1):7+7*(k-1));
        
        %%% current state estimated a priori %%%        
        xhat_now = xhat_now_full(1+7*(k-1):7+7*(k-1));
        
        %%% current position %%%
        pos = pos_full(1+6*(k-1):6+6*(k-1));
        
        %%% get B ECI from est and measure %%%
        [B_inv, B_est, B_mean] = BeciEst(xhat_now, pos, z_now, DynOpt);
        
        %%%% Linearisation %%%%
        % Linearized State equation in xk-1
        G = Gmatrix_EKF_att_v1(DynOpt,x_past,k);

        % Linearized State equation in xk 
        H = Hmatrix_EKF_att_v1(DynOpt,xhat_now,B_mean,k);
        
        % use estimation to get measures
        z_hat = hmap_attitude_v1(xhat_now,B_mean,DynOpt, k);

        %%%% reset covariance %%%%
        if (DynOpt.ObserverTest.reset_P == 1) && (mod(DynOpt.iter,DynOpt.ObserverTest.position_P_reset_aftersamples)==0)
            DynOpt.KF(k).P = DynOpt.ObserverTest.AttitudeP;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%% A priori covariance - S3 %%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         Pbar = G*DynOpt.KF(k).AttitudeP*G'+ DynOpt.KF(k).AttitudeQ;
        phi = eye(size(G)) + G*DynOpt.Ts;
        Pbar = phi*DynOpt.KF(k).AttitudeP*transpose(phi) + DynOpt.KF(k).AttitudeQ;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%% Kalman gain - S4 %%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        K = Pbar*transpose(H)*(pinv(H*Pbar*transpose(H) + DynOpt.KF(k).AttitudeR));
        K = double(K);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%% state estimate - S5 %%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        innovation = K*(z_now - z_hat);
        mismatch = (z_now - z_hat);
        x_hat_new = xhat_now + innovation;

        % storage       
        DynOpt.ObserverTest.KF_mem(k).predict(:,DynOpt.iter) = innovation;
        DynOpt.ObserverTest.KF_mem(k).mismatch(:,DynOpt.iter) = mismatch;

        % normalisation
        x_hat_new(1:4) = quatnormalize(transpose(x_hat_new(1:4)));

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% covariance estimation - S6 %%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Pnew = (eye(size(G,1)) - K*H)*Pbar;    

        % update estimation state + measures
        xnew = x_hat_new;
        
        %prediction of the estimate
        DynOpt.KF(k).AttitudeP = Pnew;
        DynOpt.Xstory_att_est(1+7*(k-1):7+7*(k-1),DynOpt.iter) = xnew;
        
        %%%%%%%%%%%%%%%%% OBSERVABILITY ANALYSIS %%%%%%%%%%%%%%%%%%
        if (DynOpt.ObserverTest.obsAnalysis == 1) && (k == DynOpt.obs.SelAgent)
           [~,~,dtheta_num] = ObsAnalysis(DynOpt,DynOpt.obs.nder,xnew,z_hat,0); 
           DynOpt.obs.ObsRank(DynOpt.iter) = rank(dtheta_num);
           DynOpt.obs.ObsCond(DynOpt.iter) = z_hat(2)^2 + z_hat(3)^2;
           DynOpt.obs.ObsEig(DynOpt.iter) = sqrt(min(eig(dtheta_num*transpose(dtheta_num))));
        end

    end

end

