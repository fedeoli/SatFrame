%% revised measurement function
function DynOpt = Observer_Measurements_v2(satellites_iner_ECI, params, DynOpt)

% Transform inertial position and velocity into relative LVLH frame
params.deputy_rel_LVLH = AbsECI2RelHill_V1_2(satellites_iner_ECI, params.mi, params);

%%%%%%%%%%%%%%%%%%%%%%%%%%%% RANGING MEASURES %%%%%%%%%%%%%%%%%%%%%%%%%%
% Set the AdjacencyMatrix (distence matrices, simmetric (zeros the lower triangular part, non zero the upper triangular one))
% the distance measurament and data exchanged is assumed to be "instantaneous"
DynOpt.ObserverTest.AdjacencyMatrix = setAdjacencyMatrixNorm(satellites_iner_ECI(DynOpt.ObserverTest.PositionArrayIndex),DynOpt.ObserverTest.Nagents);

%AdjacencyMatrix obtained by the apriori sent GPS
DynOpt.ObserverTest.AprioriAdjacencyMatrix =  setAdjacencyMatrixNorm(DynOpt.ObserverTest.APrioriEstimationXYZ,DynOpt.ObserverTest.Nagents);
                        
% relative distances
DynOpt.ObserverTest.MeasuredDistances = zeros(DynOpt.ObserverTest.Nagents);
for n = 1:DynOpt.ObserverTest.Nagents
    % Measured distances: adding the measurement error to UWB (relative distances)
    for jj = n+1:DynOpt.ObserverTest.Nagents
        UWBnoise = DynOpt.noise_enable*DynOpt.ObserverTest.ErrorAmplitudeUWB*2*(0.5*(1-rand));
        DynOpt.ObserverTest.MeasuredDistances(n,jj) = UWBnoise + DynOpt.ObserverTest.AdjacencyMatrix(n,jj);
    end
end
DynOpt.ObserverTest.MeasuredDistances = DynOpt.ObserverTest.MeasuredDistances + DynOpt.ObserverTest.MeasuredDistances';
DynOpt.y_UWB = DynOpt.ObserverTest.MeasuredDistances;


%%%%%%%%%%%%%%%%%%%%%%%%%%%% GNSS measurements %%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% get position %%%
for n = 1:DynOpt.ObserverTest.Nagents

    % Satellite's inertial position vector expressed in ECI (chief)
    satellite_pos_ECI = satellites_iner_ECI(1+6*(n-1):3+6*(n-1));       

    % Satellite's inertial velocity vector expressed in ECI (chief)
    satellite_vel_ECI = satellites_iner_ECI(4+6*(n-1):6+6*(n-1));    
    
    % Compute the Gaussian noise based on the prescribed covariances and
    % mean value (edit F.Oliva 07/10)
    GPSnoise = DynOpt.noise_enable*DynOpt.ObserverTest.GPSGaussianCovariance.*randn(6,1);
    DynOpt.ObserverTest.GPSnoise_story(n).data(:,DynOpt.iter) = GPSnoise;
   
    % add noise - on true measures
    if(n == 1) %chief
        GPS_LVLH = GPSnoise;
    else
        deputy_rel_LVLH = AbsECI2RelHill_V1_2(satellites_iner_ECI, params.mi, params);
        GPS_LVLH = deputy_rel_LVLH(:,n-1) + GPSnoise; %GPS given wrt Chief's LHLV
    end
    coe = rv2coe_V1_1(satellite_pos_ECI(:,1), satellite_vel_ECI(:,1), params.mi);
    chief_omega_LVLH = [params.fh_c*params.r_c/params.h_c; 0; params.h_c/params.r_c^2];
    myGPS = LVLH2ECI_V1_1(GPS_LVLH(1:3),coe(3), coe(5), coe(4) + coe(6)) + satellite_pos_ECI(:,1);
    myGPSpeed = LVLH2ECI_V1_1(GPS_LVLH(4:6) + cross(chief_omega_LVLH, GPS_LVLH(1:3)),coe(3), coe(5), coe(4) + coe(6)) + satellite_vel_ECI(:,1);
%     myGPS = satellite_pos_ECI + GPSnoise(1:3);
%     myGPSpeed = satellite_vel_ECI + GPSnoise(4:6);
    
    %PROJECTION OF THE GPS 
    if 0 && (DynOpt.iter > DynOpt.ObserverTest.UWBOptimizationNoBeforeThan)
        
        DynOpt.ObserverTest.MismatchAprioriDistances(DynOpt.iter) = sum(abs(DynOpt.ObserverTest.MeasuredDistances(n,:) - DynOpt.ObserverTest.AprioriAdjacencyMatrix(n,:)));
        dynamicSphere = DynOpt.ObserverTest.GPSprojectionOnAprioriSphereRadius*max(1,DynOpt.ObserverTest.MismatchAprioriDistances(n) / DynOpt.ObserverTest.AprioriMismatch4Projection);
        
        X = rk4_V1_1(DynOpt.model_inertial, params.tspan, DynOpt.Xstory_est(1+6*(n-1):6+6*(n-1),DynOpt.iter-1), params);  
        xhat_now_pos =  X(1:3,end);
      
        if(norm(xhat_now_pos - myGPS) > dynamicSphere)     %too far!
            franorm = norm(xhat_now_pos - myGPS)/dynamicSphere;
            myGPS = xhat_now_pos + (myGPS - xhat_now_pos)/franorm;
        end
    end
       
    %%% output assignment %%%
    DynOpt.y_GPS(1+6*(n-1):6+6*(n-1),DynOpt.iter) = [myGPS; myGPSpeed]; 
    
end
