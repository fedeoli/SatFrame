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
        
        % handle sun sensor
        Sun = reshape(DynOpt.y_Sun(1+3*(k-1):3+3*(k-1),DynOpt.iter),3,1);
        
        if (DynOpt.ObserverTest.Sun == 1) && (DynOpt.ObserverTest.Eclipse == 0)
            if DynOpt.ObserverTest.nMagneto == 0
                z_now = [Gyro; Sun];
            elseif DynOpt.ObserverTest.nMagneto == 1
                z_now = [Mag_1; Gyro; Sun];
            elseif DynOpt.ObserverTest.nMagneto == 2
                z_now = [Mag_1; Mag_2; Gyro; Sun];
            end
        else
            if DynOpt.ObserverTest.nMagneto == 0
                z_now = Gyro;
            elseif DynOpt.ObserverTest.nMagneto == 1
                z_now = [Mag_1; Gyro];
            elseif DynOpt.ObserverTest.nMagneto == 2
                z_now = [Mag_1; Mag_2; Gyro];
            end
        end
        
        %%% past state %%%
        x_past = x_past_full(1+7*(k-1):7+7*(k-1));
        
        %%% current state estimated a priori %%%        
        xhat_now = xhat_now_full(1+7*(k-1):7+7*(k-1));
        
        %%% current position %%%
        pos = pos_full(1+6*(k-1):6+6*(k-1));
        
        %%% get B ECI from est and measure %%%
        [B_inv, B_est, B_mean] = BeciEst(xhat_now, pos, z_now, DynOpt);
        
        %%% get B ECI from est and measure %%%
        [SEci_est, SEci_inv, SEci_mean] = SunEciEst(xhat_now, pos, Sun, DynOpt, params);
        
        %%%% Linearisation %%%%
        % Linearized State equation in xk-1
        G = Gmatrix_EKF_att_v1(DynOpt,x_past,k);

        % Linearized State equation in xk 
        H = Hmatrix_EKF_att_v1(DynOpt,xhat_now,B_est,SEci_est,k);
        
        % use estimation to get measures
        z_hat = hmap_attitude_v1(xhat_now,B_est,SEci_est,DynOpt, k);

        %%%% reset covariance %%%%
        if (DynOpt.ObserverTest.reset_P == 1) && (mod(DynOpt.iter,DynOpt.ObserverTest.position_P_reset_aftersamples)==0)
            DynOpt.KF(k).P = DynOpt.ObserverTest.AttitudeP;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%% A priori covariance - S3 %%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         Pbar = G*DynOpt.KF(k).AttitudeP*G'+ DynOpt.KF(k).AttitudeQ;
        phi = eye(size(G)) + G*DynOpt.Ts;
        Q = DynOpt.KF(k).AttitudeQ;
        Pbar = phi*DynOpt.KF(k).AttitudeP*transpose(phi) + Q;
        
        % save P eigenvalues
        DynOpt.ObserverTest.Peig_att(:,DynOpt.iter) = eig(Pbar);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%% Kalman gain - S4 %%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        R = DynOpt.KF(k).AttitudeR(1:length(z_hat),1:length(z_hat));
        K = Pbar*transpose(H)*(pinv(H*Pbar*transpose(H) + R));
        K = double(K);
        
        % save K
        DynOpt.ObserverTest.att_Kmean(1:length(z_hat),DynOpt.iter) = mean(K,1);
        
        for i=1:length(z_hat)
            DynOpt.ObserverTest.att_Knorm(i,DynOpt.iter) = norm(K(:,i));
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%% state estimate - S5 %%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        mismatch = (z_now - z_hat);
        innovation = K*mismatch;
        x_hat_new = xhat_now + innovation;
        
        % save mismatch
        DynOpt.ObserverTest.att_mismatch(:,DynOpt.iter) = innovation;

        % storage       
        DynOpt.ObserverTest.KF_mem(k).predict(:,DynOpt.iter) = innovation;
        DynOpt.ObserverTest.KF_mem(k).mismatch(1:length(z_hat),DynOpt.iter) = mismatch;

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
           DynOpt.obs.ObsCond(DynOpt.iter) = (z_hat(4)+z_hat(2))^2 + (z_hat(5)-z_hat(1))^2;
           DynOpt.obs.ObsEig(DynOpt.iter) = sqrt(min(eig(dtheta_num*transpose(dtheta_num))));
           
           u = DynOpt.mag_field_norm_true(:,DynOpt.iter);
           v = z_hat(1:3);
           DynOpt.obs.ObsCondB(1,DynOpt.iter) = atan2(norm(cross(u,v)),dot(u,v));
           if DynOpt.ObserverTest.nMagneto == 2
               u = DynOpt.mag_field_norm_true(:,DynOpt.iter);
               v = z_hat(4:6);
               DynOpt.obs.ObsCondB(2,DynOpt.iter) = atan2(norm(cross(u,v)),dot(u,v));
           end
        end

    end

end

