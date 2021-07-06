%% UKF to estimate position and speeds of the agents
function [DynOpt,params] = Position_UKF_V1_6(DynOpt,params)

    for k = 1:DynOpt.ObserverTest.Nagents

        %%%%%%%%%%%%%%%%%%%% CURRENT GPS %%%%%%%%%%%%%%%%%
        myGPS = reshape(DynOpt.y_GPS(1+6*(k-1):3+6*(k-1),DynOpt.iter),3,1);
        myGPSpeed = reshape(DynOpt.y_GPS(4+6*(k-1):6+6*(k-1),DynOpt.iter),3,1);
        

        % RESET OF P after a given amount of samples
        if (DynOpt.ObserverTest.reset_P == 1) && (mod(DynOpt.iter,DynOpt.ObserverTest.position_P_reset_aftersamples)==0)
            DynOpt.KF(k).P = DynOpt.ObserverTest.Pi;
        end

        % augmented covariance matrix (state + noise)
        DynOpt.ObserverTest.Pa = blkdiag(DynOpt.KF(k).P,DynOpt.KF(k).Q);
        
        % generate sigma points
        DynOpt = sigmaPoints_sat(DynOpt.Xstory_pos_est(1+6*(k-1):6+6*(k-1),DynOpt.iter-1),DynOpt.ObserverTest.Pa,params,DynOpt);
        DynOpt.ObserverTest.Pa = DynOpt.ObserverTest.Psat;
        
        % Sigma point evolution 
        DynOpt.ObserverTest.CSI_X_meno = 0*DynOpt.ObserverTest.CSI(1:6,:);
        for j = 1:2 * DynOpt.ObserverTest.Na+1
            DynOpt.ObserverTest.CSI_jth = transpose(DynOpt.ObserverTest.CSI(:,j));
            X = DynOpt.ode(@(t,x)DynOpt.model_inertial(t, x, params, DynOpt), params.tspan, DynOpt.ObserverTest.CSI_jth(1:6));
            DynOpt.ObserverTest.CSI_X_meno(:,j) = X.y(:,end);
        end
        
        % stima valor medio
        xHatMeno = DynOpt.ObserverTest.CSI_X_meno * DynOpt.ObserverTest.W;

        % Aggiornamento matrice di covarianza
        if DynOpt.ObserverTest.startzero == 1
            MatCOV = zeros(size(DynOpt.KF(k).P));
        else
            MatCOV = DynOpt.KF(k).P;
        end
        for j = 1:2 * DynOpt.ObserverTest.Na+1
            MatCOV = MatCOV + DynOpt.ObserverTest.W(j) * (DynOpt.ObserverTest.CSI_X_meno(:,j)-xHatMeno) * transpose((DynOpt.ObserverTest.CSI_X_meno(:,j)-xHatMeno));
        end
        Pmeno = MatCOV;

        % GPS + UWB attesi per ogni sigma point
        % ZETA: the first three are the three coordinates of the GPS while
        % the other Nagents-1 are the distance measured with the other
        % ones (the own distance is cleared off).
        for j = 1:2*DynOpt.ObserverTest.Na+1            

            %%%%%%%% compute the true h function %%%%%%%    
            sigma_opt = DynOpt.ObserverTest.CSI_X_meno(1:3,j);

            % store vals
            ZETA(:,j) = [sigma_opt;DynOpt.ObserverTest.CSI_X_meno(4:6,j)];
        end

        % expected value of the measures (GPS) and speed
        zHatMeno = ZETA * DynOpt.ObserverTest.W;

        % matrice covarianza Pzeta
        MatCOV = zeros(length(ZETA(:,1)),length(ZETA(:,1)));
        for j = 1:2 * DynOpt.ObserverTest.Na+1
            MatCOV = MatCOV + DynOpt.ObserverTest.W(j) * (ZETA(:,j)-zHatMeno) * transpose((ZETA(:,j)-zHatMeno));
        end
        
        Pzeta = MatCOV + DynOpt.KF(k).R;

        % matrice covarianza mista Pxz
        MatCOV = zeros(length(DynOpt.ObserverTest.CSI_X_meno(:,1)),length(ZETA(:,1)));
        for j = 1:2 * DynOpt.ObserverTest.Na+1
            MatCOV = MatCOV + DynOpt.ObserverTest.W(j) * (DynOpt.ObserverTest.CSI_X_meno(:,j)-xHatMeno)*(ZETA(:,j)-zHatMeno)';
        end
        Pxz = MatCOV;

        % innovation ?
        innovation = [myGPS; myGPSpeed] - zHatMeno;

        % guadagno UKF
        K = Pxz * pinv(Pzeta);
        
        %This is the corrected estimation at time i+1-Sensor_delay
        xHat = xHatMeno + K*innovation;


        % Aggiornamento matrice di covarianza at time i+1-Sensor_delay
        P = Pmeno - K*transpose(Pxz);

        %prediction of the estimate
        DynOpt.KF(k).P = P;
        DynOpt.Xstory_pos_est(1+6*(k-1):6+6*(k-1),DynOpt.iter) = xHat;
        
        % P evolution index
        DynOpt.ObserverTest.cond_numb(k,DynOpt.iter) = min(eig(Pmeno))/max(eig(Pmeno));
        DynOpt.ObserverTest.Peig_max(k,DynOpt.iter) = max(eig(Pmeno));
        DynOpt.ObserverTest.Peig_min(k,DynOpt.iter) = min(eig(Pmeno));
        
        [DynOpt.ObserverTest.buf_dymin(k,:), DynOpt.ObserverTest.dymin(k,DynOpt.iter)] =  IterativePseudoDerivative(DynOpt.Ts,DynOpt.ObserverTest.Peig_min(k,DynOpt.iter),...
                                                                                DynOpt.ObserverTest.c1_derivative,DynOpt.ObserverTest.d1_derivative,0,DynOpt.ObserverTest.buf_dymin(k,:));

        [DynOpt.ObserverTest.buf_dymax(k,:), DynOpt.ObserverTest.dymax(k,DynOpt.iter)] =  IterativePseudoDerivative(DynOpt.Ts,DynOpt.ObserverTest.Peig_max(k,DynOpt.iter),...
                                                                                DynOpt.ObserverTest.c1_derivative,DynOpt.ObserverTest.d1_derivative,0,DynOpt.ObserverTest.buf_dymax(k,:));

        [DynOpt.ObserverTest.buf_dcond(k,:), DynOpt.ObserverTest.dcond(k,DynOpt.iter)] =  IterativePseudoDerivative(DynOpt.Ts,DynOpt.ObserverTest.cond_numb(k,DynOpt.iter),...
                                                                                DynOpt.ObserverTest.c1_derivative,DynOpt.ObserverTest.d1_derivative,0,DynOpt.ObserverTest.buf_dcond(k,:));

        DynOpt.ObserverTest.dymin_mean(DynOpt.iter) = mean(DynOpt.ObserverTest.dymin(:,DynOpt.iter));
        DynOpt.ObserverTest.dymax_mean(DynOpt.iter) = mean(DynOpt.ObserverTest.dymax(:,DynOpt.iter));
        DynOpt.ObserverTest.dcond_mean(DynOpt.iter) = mean(DynOpt.ObserverTest.dcond(:,DynOpt.iter));
        DynOpt.ObserverTest.cond_numb_mean(DynOpt.iter) = mean(DynOpt.ObserverTest.cond_numb(:,DynOpt.iter));
    end
    
end