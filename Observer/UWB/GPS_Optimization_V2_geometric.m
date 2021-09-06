%% GPS OTPIMIZATON - GEOMETRIC METHOD
function DynOpt = GPS_Optimization_V2_geometric(DynOpt)

    % flag and iteration init 
    DynOpt.ObserverTest.UWBoptimizationOn = 1;

    % loop over the agents
    for  k = 1:DynOpt.ObserverTest.Nagents

        %%%%%%%%%%%%%%%% GPS OPTIMIZATION VIA UWB %%%%%%%%%

        %%%%%%%%%%%%%%%%%%%% CURRENT GPS %%%%%%%%%%%%%%%%%
        myGPS = reshape(DynOpt.y_GPS(1+6*(k-1):3+6*(k-1),DynOpt.iter),3,1);
        myGPSpeed = reshape(DynOpt.y_GPS(4+6*(k-1):6+6*(k-1),DynOpt.iter),3,1);

        %%%%%%%%%%%%%%%%%% DATA INIT %%%%%%%%%%%%%%%%%%%%%%
        DynOpt.ObserverTest.CurrentAgent = k;

        % flag for GPS optimization
        GPS_flag = DynOpt.iter > DynOpt.ObserverTest.UWBOptimizationNoBeforeThan;
        DynOpt.ObserverTest.GPS_flag = GPS_flag;

        % check if GPS optimization has to be done
        if 0 || GPS_flag 
            

            % SETTING THE INITIAL CONDITIONS FOR OPTIMIZATION
            % set GPS optimization flag for the current iteration
            DynOpt.ObserverTest.UWBcorrectionActive(k, DynOpt.iter) = 1;

            %%%%%%%%%%%%% GPS OPTIMIZATION %%%%%%%%%%%%% 
            % Get apriori estimate 
            if strcmp(DynOpt.ObserverTest.projection,'Chi')
                Chi = transpose(reshape(DynOpt.ObserverTest.APrioriEstimationXYZ,3,DynOpt.ObserverTest.Nagents));
            elseif strcmp(DynOpt.ObserverTest.projection,'GPS')
                Chi = transpose(reshape(DynOpt.y_GPS(:,DynOpt.iter),6,DynOpt.ObserverTest.Nagents));
                Chi(:,4:6) = [];
            else
                disp('wrong projection method')
            end
            
            % Get relative distances
            adjmat_UWB = DynOpt.y_UWB;

            % Get GPS measurements
            GPS = reshape(myGPS,1,3);
            
            theta = DynOpt.ObserverTest.theta;
            beta = DynOpt.ObserverTest.beta;

            % handle packet loss for UWB
            if DynOpt.ObserverTest.UWBDropMessages
                packet_UWB = reshape(DynOpt.ObserverTest.SuccessfullyReadUWB(DynOpt.iter,:,:), DynOpt.ObserverTest.Nagents, DynOpt.ObserverTest.Nagents);
            else
                packet_UWB = ones(DynOpt.ObserverTest.Nagents) - eye(DynOpt.ObserverTest.Nagents);
            end
            
            % optimize GPS
            opt = Position_opt_cloud_num_v10_dec(Chi, GPS, adjmat_UWB, k, theta, beta, DynOpt.ObserverTest.check_distance,...
                    packet_UWB, DynOpt.ObserverTest.APrioriEstimationXYZ, DynOpt);
            NewGPS = [reshape(opt.Chi_est,1,3), transpose(myGPSpeed)];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            

            %%%%% SAVE THE DATA %%%%%
            DynOpt.out(k).OnlyGPSopt(:,DynOpt.iter) = opt.Chi_est;
            DynOpt.out(k).sigma_p(:,DynOpt.iter) = sqrt(opt.sigma_p);
            DynOpt.y_GPS(1+6*(k-1):6+6*(k-1),DynOpt.iter) = NewGPS;
            
    
        else  

            DynOpt.y_GPS(1+6*(k-1):6+6*(k-1),DynOpt.iter) = [myGPS; myGPSpeed];

        end
        
    end
end

