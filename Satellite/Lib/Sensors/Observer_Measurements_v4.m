%% revised measurement function
function DynOpt = Observer_Measurements_v4(satellites_iner_ECI, params, DynOpt)

%%%% Observer - integration of position dynamics - used for the packet loss %%%%%%%
if (DynOpt.iter > 1)
    X = DynOpt.ode(@(t,x)DynOpt.model_inertial(t, x, params, DynOpt), params.tspan, DynOpt.Xstory_pos_est(:,DynOpt.iter-1));
    for k = 1:DynOpt.ObserverTest.Nagents        
        Xstory_pos_est(1+6*(k-1):6+6*(k-1)) =  X.y(1+6*(k-1):6+6*(k-1),end);
        APrioriEstimationXYZ(1+3*(k-1):3+3*(k-1)) = Xstory_pos_est(1+6*(k-1):3+6*(k-1));
        APrioriEstimationVel(1+3*(k-1):3+3*(k-1)) = Xstory_pos_est(4+6*(k-1):6+6*(k-1));
    end
else
    APrioriEstimationXYZ = DynOpt.ObserverTest.APrioriEstimationXYZ;
    APrioriEstimationVel = DynOpt.ObserverTest.APrioriEstimationVel;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Transform inertial position and velocity into relative LVLH frame
if params.Ndeputy
    params.deputy_rel_LVLH = AbsECI2RelHill_V1_2(satellites_iner_ECI, params.mi, params);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%% RANGING MEASURES %%%%%%%%%%%%%%%%%%%%%%%%%%
% Set the AdjacencyMatrix (distence matrices, simmetric (zeros the lower triangular part, non zero the upper triangular one))
% the distance measurament and data exchanged is assumed to be "instantaneous"
DynOpt.ObserverTest.AdjacencyMatrix = setAdjacencyMatrixNorm(satellites_iner_ECI(DynOpt.ObserverTest.PositionArrayIndex),DynOpt.ObserverTest.Nagents);

% check for correct packet transmission - UWB
% here the measure is not replaced, simply won't be counted in the
% geometric method (see Position_opt_cloud)
if (DynOpt.ObserverTest.UWBDropMessages) && (DynOpt.iter > DynOpt.ObserverTest.lossafter)
    tmp_adj = zeros(DynOpt.ObserverTest.Nagents);
    for n = 1:DynOpt.ObserverTest.Nagents
       for nn = n+1:1:DynOpt.ObserverTest.Nagents
           tmp_adj(n,nn) = PacketLoss_v1(DynOpt.ObserverTest.SuccessfullyReadUWB(DynOpt.iter-1,n,nn),DynOpt.ObserverTest.UWBDropMessagesP,DynOpt.ObserverTest.UWBDropMessagesR);
       end
    end
    tmp_adj = tmp_adj + tmp_adj';
    DynOpt.ObserverTest.SuccessfullyReadUWB(DynOpt.iter,:,:) = tmp_adj;
else
    DynOpt.ObserverTest.SuccessfullyReadUWB(DynOpt.iter,:,:) = DynOpt.ObserverTest.SuccessfullyReadUWB(1,:,:);
end
                        
% relative distances
DynOpt.ObserverTest.MeasuredDistances = zeros(DynOpt.ObserverTest.Nagents);
for n = 1:DynOpt.ObserverTest.Nagents
    % Measured distances: adding the measurement error to UWB (relative distances)
    for jj = n+1:DynOpt.ObserverTest.Nagents
        UWBnoise = DynOpt.noise_enable*DynOpt.ObserverTest.ErrorAmplitudeUWB*randn;
        DynOpt.ObserverTest.MeasuredDistances(n,jj) = UWBnoise + DynOpt.ObserverTest.AdjacencyMatrix(n,jj);
    end
end

% set the final UWB measurement
DynOpt.ObserverTest.MeasuredDistances = DynOpt.ObserverTest.MeasuredDistances + DynOpt.ObserverTest.MeasuredDistances';
DynOpt.y_UWB = DynOpt.ObserverTest.MeasuredDistances;


%%%%%%%%%%%%%%%%%%%%%%%%%%%% GNSS measurements %%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% get position %%%
for n = 1:DynOpt.ObserverTest.Nagents

    % Satellite's inertial position vector expressed in ECI (chief)
    satellite_pos_ECI = satellites_iner_ECI(1+6*(n-1):3+6*(n-1));       

    % Satellite's inertial velocity vector expressed in ECI (chief)
    satellite_vel_ECI = satellites_iner_ECI(4+6*(n-1):6+6*(n-1));    
    
    % check for correct packet transmission
    if (DynOpt.ObserverTest.GPSDropMessages) && (DynOpt.iter > DynOpt.ObserverTest.lossafter)
        DynOpt.ObserverTest.SuccessfullyReadGPS(n,DynOpt.iter) = PacketLoss_v1(DynOpt.ObserverTest.SuccessfullyReadGPS(n,DynOpt.iter-1),...
                                                                                   DynOpt.ObserverTest.GPSDropMessagesP,DynOpt.ObserverTest.GPSDropMessagesR);
    else
        DynOpt.ObserverTest.SuccessfullyReadGPS(n,DynOpt.iter) = 1;
    end
    
    % Compute the Gaussian noise based on the prescribed covariances and
    % mean value (edit F.Oliva 07/10)
    GPSnoise = DynOpt.noise_enable*DynOpt.ObserverTest.GPSGaussianCovariance.*randn(6,1);
    DynOpt.ObserverTest.GPSnoise_story(n).data(:,DynOpt.iter) = GPSnoise;
   
    myGPS = satellite_pos_ECI + GPSnoise(1:3);
    myGPSpeed = satellite_vel_ECI + GPSnoise(4:6);
       
    %%% output assignment - GPS is replaced with a priori if not measured %%%
    if DynOpt.ObserverTest.SuccessfullyReadGPS(n,DynOpt.iter)
        DynOpt.y_GPS(1+6*(n-1):6+6*(n-1),DynOpt.iter) = [myGPS; myGPSpeed]; 
    else
        DynOpt.y_GPS(1+6*(n-1):6+6*(n-1),DynOpt.iter) = [APrioriEstimationXYZ(1+3*(n-1):3+3*(n-1)), APrioriEstimationVel(1+3*(n-1):3+3*(n-1))];
    end
    
end
