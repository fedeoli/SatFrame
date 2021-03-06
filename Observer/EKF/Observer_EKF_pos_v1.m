%% Attitude_Obsever_V2_1: estimates the i-th satellite attitude using MAHONY
function  [DynOpt, params] = Observer_EKF_pos_v1(DynOpt, params)

    %%% past state %%%
    x_past_total = DynOpt.Xstory_pos_est(:,DynOpt.iter-1);
    
    %%% current state estimated a priori %%%
    tmp = DynOpt.ode(@(t,x)DynOpt.model_inertial(t, x, params, DynOpt), params.tspan, x_past_total);
    xhat_now_total = tmp.y(:,end);
    
    %%% adjmat from CHI
    adjmat_Chi = setAdjacencyMatrixNorm(xhat_now_total(DynOpt.ObserverTest.PositionArrayIndex),DynOpt.ObserverTest.Nagents);
    adjmat_GPS = setAdjacencyMatrixNorm(DynOpt.y_GPS(DynOpt.ObserverTest.PositionArrayIndex),DynOpt.ObserverTest.Nagents);
    adjmat_UWB = DynOpt.y_UWB;


    for k = 1:DynOpt.ObserverTest.Nagents

        %%%% INIT SECTION %%%%

        %%%%%%%%%%%%%%%%%%%% CURRENT GPS %%%%%%%%%%%%%%%%%
        myGPS = reshape(DynOpt.y_GPS(1+6*(k-1):3+6*(k-1),DynOpt.iter),3,1);
        myGPSpeed = reshape(DynOpt.y_GPS(4+6*(k-1):6+6*(k-1),DynOpt.iter),3,1);
        
        if DynOpt.ObserverTest.EKF_withRD == 0
            z_now = [myGPS; myGPSpeed];
        else
            d_vec_UWB = nonzeros(adjmat_UWB(:,k));
            z_now = [myGPS; myGPSpeed; d_vec_UWB]; 
        end
        
        %%% past state %%%
        x_past = x_past_total(1+6*(k-1):6+6*(k-1));

        %%% a priori state %%%
        xhat_now = xhat_now_total(1+6*(k-1):6+6*(k-1));
        
        %%% x_RD 
        x_RD = DynOpt.y_GPS(:,DynOpt.iter);
        x_RD(1+6*(k-1):6+6*(k-1)) = xhat_now;
        
        if DynOpt.ObserverTest.EKF_withRD == 0
            z_hat = xhat_now;
        else
            adjmat = setAdjacencyMatrixNorm(x_RD(DynOpt.ObserverTest.PositionArrayIndex),DynOpt.ObserverTest.Nagents);
            d_vec_hat = nonzeros(adjmat(:,k));
            z_hat = [xhat_now; d_vec_hat]; 
        end
        
        %%%% Linearisation %%%%
        % Linearized State equation in xk-1
        G = Gmatrix_EKF_v2(DynOpt,params,x_past); 

        % Linearized State equation in xk 
        if DynOpt.ObserverTest.GPS_flag         
            H = Hmatrix_EKF_v4(DynOpt,xhat_now_total,k);
        else
            if DynOpt.ObserverTest.EKF_withRD
                H = Hmatrix_EKF_withRD(DynOpt,x_RD,k);
            else
                H = double(DynOpt.sym.H_pos);
            end
        end

        %%%% reset covariance %%%%
        if (DynOpt.ObserverTest.reset_P == 1) && (mod(DynOpt.iter,DynOpt.ObserverTest.position_P_reset_aftersamples)==0)
            DynOpt.KF(k).P = DynOpt.ObserverTest.Pi;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%% A priori covariance - S3 %%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         Pbar = G*DynOpt.KF(k).P*G'+ DynOpt.KF(k).Q; 
        phi = eye(size(G)) + G*DynOpt.Ts;
        Pbar = phi*DynOpt.KF(k).P*transpose(phi) + DynOpt.KF(k).Q;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%% Kalman gain - S4 %%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        K = Pbar*transpose(H)*(pinv(H*Pbar*transpose(H) + DynOpt.KF(k).R));
        K = double(K);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%% state estimate - S5 %%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        x_hat_new = xhat_now + K*(z_now - z_hat);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% covariance estimation - S6 %%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Pnew = (eye(size(G,1)) - K*H)*Pbar;    

        % update estimation state + measures
        xnew = x_hat_new;

        %prediction of the estimate
        DynOpt.KF(k).P = Pnew;
        if DynOpt.ObserverTest.EKF_withRD
            DynOpt.KF(k).H(:,DynOpt.iter) = sqrt(eig(transpose(H)*H));
            DynOpt.KF(k).K(:,DynOpt.iter) = sqrt(eig(transpose(K)*K));
        else
            DynOpt.KF(k).H(:,DynOpt.iter) = eig(H);
            DynOpt.KF(k).K(:,DynOpt.iter) = eig(K);
        end
        
        DynOpt.Xstory_pos_est(1+6*(k-1):6+6*(k-1),DynOpt.iter) = xnew;
        
        % P evolution index
        DynOpt.ObserverTest.cond_numb(k,DynOpt.iter) = min(eig(Pnew))/max(eig(Pnew));
        DynOpt.ObserverTest.Peig_max(k,DynOpt.iter) = max(eig(Pnew));
        DynOpt.ObserverTest.Peig_min(k,DynOpt.iter) = min(eig(Pnew));
        
        [DynOpt.ObserverTest.buf_dymin(k,:), DynOpt.ObserverTest.dymin(k,DynOpt.iter)] =  IterativePseudoDerivative(DynOpt.Ts,DynOpt.ObserverTest.Peig_min(k,DynOpt.iter),...
                                                                                DynOpt.ObserverTest.c1_derivative,DynOpt.ObserverTest.d1_derivative,0,DynOpt.ObserverTest.buf_dymin(k,:));

        [DynOpt.ObserverTest.buf_dymax(k,:), DynOpt.ObserverTest.dymax(k,DynOpt.iter)] =  IterativePseudoDerivative(DynOpt.Ts,DynOpt.ObserverTest.Peig_max(k,DynOpt.iter),...
                                                                                DynOpt.ObserverTest.c1_derivative,DynOpt.ObserverTest.d1_derivative,0,DynOpt.ObserverTest.buf_dymax(k,:));

        [DynOpt.ObserverTest.buf_dcond(k,:), DynOpt.ObserverTest.dcond(k,DynOpt.iter)] =  IterativePseudoDerivative(DynOpt.Ts,DynOpt.ObserverTest.cond_numb(k,DynOpt.iter),...
                                                                                DynOpt.ObserverTest.c1_derivative,DynOpt.ObserverTest.d1_derivative,0,DynOpt.ObserverTest.buf_dcond(k,:));

        DynOpt.ObserverTest.dymin_mean(DynOpt.iter) = mean(DynOpt.ObserverTest.dymin(:,DynOpt.iter));
        DynOpt.ObserverTest.dymax_mean(DynOpt.iter) = mean(DynOpt.ObserverTest.dymax(:,DynOpt.iter));
        DynOpt.ObserverTest.dcond_mean(DynOpt.iter) = abs(mean(DynOpt.ObserverTest.dcond(:,DynOpt.iter)));
        DynOpt.ObserverTest.cond_numb_mean(DynOpt.iter) = mean(DynOpt.ObserverTest.cond_numb(:,DynOpt.iter));

    end

end

