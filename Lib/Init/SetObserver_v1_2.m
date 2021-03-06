%% setup
function ObserverTest = SetObserver_v1_2(satellites_iner_ECI,satellites_attitude,params,DynOpt)
%%load all observer parameters

load('Lib/Init/ObserverTest.mat')

%%%%%%%% 05/10 %%%%%%%
ObserverTest.Nagents = params.Ndeputy + 1;

%%%%%% FLAG + NOISE SETUP %%%%%%%

% Position Observer flags
ObserverTest.ObserverON_pos = DynOpt.ObserverOn_pos;
ObserverTest.obsAnalysis = 0;

% attitude Observer flags
ObserverTest.ObserverON_att = DynOpt.ObserverOn_att;

% GPS flags
ObserverTest.GPSopt_flag = 1 && (params.Ndeputy>0);
ObserverTest.UWBOptimizationNoBeforeThan = 5; % used when KF is enabled
ObserverTest.GPS_flag = 0;

% KF flags
ObserverTest.EKF_withRD = 0;
ObserverTest.KF_flag = 0;
ObserverTest.KF_pos = DynOpt.Observer_pos;
ObserverTest.KF_att = DynOpt.Observer_att;
ObserverTest.startzero = 0;
ObserverTest.reset_P = 0;
ObserverTest.position_P_reset_aftersamples = 50;

% P evolution index
ObserverTest.cond_numb = ones(ObserverTest.Nagents,1);
ObserverTest.cond_numb_mean = 0;
ObserverTest.Peig_min = ones(ObserverTest.Nagents,1);
ObserverTest.Peig_max = ones(ObserverTest.Nagents,1);
ObserverTest.c1_derivative = 5;
ObserverTest.d1_derivative = 40;
ObserverTest.buf_dymin = zeros(ObserverTest.Nagents,ObserverTest.d1_derivative);
ObserverTest.dymin = zeros(ObserverTest.Nagents,1);
ObserverTest.dymin_mean = 0;
ObserverTest.buf_dymax = zeros(ObserverTest.Nagents,ObserverTest.d1_derivative);
ObserverTest.dymax = zeros(ObserverTest.Nagents,1);
ObserverTest.dymax_mean = 0;
ObserverTest.buf_dcond = zeros(ObserverTest.Nagents,ObserverTest.d1_derivative);
ObserverTest.dcond = zeros(ObserverTest.Nagents,1);
ObserverTest.dcond_mean = 0;
ObserverTest.dcond_thresh = 1e-8;
ObserverTest.beta_story = [];
ObserverTest.theta_story = [];

% Attitude observer flags
ObserverTest.input = DynOpt.control;
ObserverTest.nMagneto = 2;  % number of Magnetometers (max. 2)
ObserverTest.Sun = 1;       % 0: no ObserverTest.Sun Sensor; 1: with ObserverTest.Sun Sensor
ObserverTest.albedo = 1;
ObserverTest.ObsTol = 5e-2;
ObserverTest.SunEclipse = 1;
ObserverTest.SunEclipseStart = 0.1;
ObserverTest.SunEclipseStop = 0.9;

%%% magetometers misalignment %%%
ObserverTest.RPYbetweenMagSensors = 1*[0,0,90]*pi/180;
Dtheta = ObserverTest.RPYbetweenMagSensors;
dcm_v2 = angle2dcm(Dtheta(1),Dtheta(2),Dtheta(3)); 
ObserverTest.dcm_v2 = dcm_v2;

% GPS optimization parameters - geometric method
ObserverTest.theta = 0.005;
ObserverTest.beta = 0;
ObserverTest.fixed_theta_flag = 1;
ObserverTest.fixed_theta = 0.005;
ObserverTest.geometric_transient = 300;
ObserverTest.check_distance = 0;
ObserverTest.projection = 'Chi';

% Measurement bias
error_enable = DynOpt.noise_enable;

%%% gyroscope %%%
ObserverTest.GyroGaussianCovariance = (error_enable*[1; 1; 1]*1e-3); % [rad/s]
ObserverTest.ErrorAmplitudeGyro = 1e-3;
ObserverTest.GyroBias = 1*error_enable*(5e-3*randn(3,1) + 1e-2);

%%% magnetometer %%%
M = 5e-2;
ObserverTest.MagGaussianCovariance = (error_enable*[1; 1; 1]*M); % [nT]
ObserverTest.ErrorAmplitudeMag = M;
ObserverTest.MagBias = 1*error_enable*(0.5*M*randn(6,1) + M);

%%% GPS %%%
ObserverTest.GPSGaussianCovariance = (error_enable*[5; 5; 5; 5e-2; 4e-2; 2e-2]*1e-3); % [Km]
ObserverTest.ErrorAmplitudeGPS = error_enable*5e-3;
ObserverTest.ErrorAmplitudeSpeed = error_enable*5e-6;

%%% UWB %%%
ObserverTest.ErrorAmplitudeUWB = (error_enable*2e-4);

%%% Sun Sensor %%%
S = 5e-2;
ObserverTest.SunGaussianCovariance = (error_enable*[1; 1; 1]*S); % [rad]
ObserverTest.ErrorAmplitudeSun = S;
ObserverTest.SunBias = 1*error_enable*(5e-3*randn + 1e-2);

%%% Sigma analysis
ObserverTest.SigmaAnalysis = 1;


%%%%% COVARIANCE ATTITUDE %%%%%
ObserverTest.statedim_att = 7;
ObserverTest.Ndisturbance_att = 7;
ObserverTest.AttitudeQ = 1*1e-3*eye(ObserverTest.statedim_att);
ObserverTest.AttitudeP = 1*1e-2*eye(ObserverTest.Ndisturbance_att);
ObserverTest.AttitudeR = (blkdiag(ObserverTest.ErrorAmplitudeMag*eye(3*ObserverTest.nMagneto), ...
                                 ObserverTest.ErrorAmplitudeGyro*eye(3), ...
                                 ObserverTest.ErrorAmplitudeSun*eye(3*ObserverTest.Sun))).^2;

%%%%% COVARIANCE POSITION %%%%%
ObserverTest.statedim_pos = 6;
ObserverTest.Ndisturbance_pos = 6;
ObserverTest.Pi = eye(ObserverTest.statedim_pos)*1e-2;                                      %P of the UKF estimator, so on below
ObserverTest.Qi = eye(ObserverTest.Ndisturbance_pos).*1e-3;                                 %ObserverTest.GPSGaussianCovariance;
% with GPS
if ObserverTest.EKF_withRD == 0
    ObserverTest.Ri = (blkdiag(eye(ObserverTest.Ngps)*ObserverTest.ErrorAmplitudeGPS,...
                               eye(ObserverTest.Nspeed)*ObserverTest.ErrorAmplitudeSpeed)).^2;
else
    ObserverTest.Ri = (blkdiag(eye(ObserverTest.Ngps)*ObserverTest.ErrorAmplitudeGPS,...
                               eye(ObserverTest.Nspeed)*ObserverTest.ErrorAmplitudeSpeed,...
                               eye(ObserverTest.Nagents-1)*ObserverTest.ErrorAmplitudeUWB)).^2;
end
% with UWB
% ObserverTest.Ri_UWB = (blkdiag(eye(ObserverTest.Ngps)*ObserverTest.ErrorAmplitudeUWB,eye(ObserverTest.Nspeed))).^2;
ObserverTest.Na = (ObserverTest.statedim_pos+ObserverTest.Ndisturbance_pos); %extended state
%%%%%%%%%%%%%%%%%%%%%%

% Packet loss
ObserverTest.lossafter = 1;

ObserverTest.UWBDropMessages = 1;
ObserverTest.UWBDropMessagesP = 0.3; %probability of losing an UWB message if the previous was sent
ObserverTest.UWBDropMessagesR = 0.85; %probability of sending an UWB message if the previous was lost

ObserverTest.GPSDropMessages = 1;
ObserverTest.GPSDropMessagesP = 0.3; %probability of losing GPS data if the previous was sent
ObserverTest.GPSDropMessagesR = 0.95; %probability of getting   message if the previous was lost
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%% INITIAL CONDITION %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial condition noise
ObserverTest.TrueInitialPositions = DynOpt.true_pos;
ObserverTest.attitudeTrueInitialPositions = DynOpt.true_att;

%initial percentage error
ObserverTest.IntialConditionPercentage = 1*[1 1 1 1 1 1]; 

%initial additive error
ObserverTest.IntialConditionAdditive = [5e2, 5e2, 5e2, 5e-2, 5e-2, 5e-2]*1e-1;    %[Km]
ObserverTest.IntialConditionAdditive = error_enable*ObserverTest.IntialConditionAdditive;
ObserverTest.IntialConditionPercentage = 1*ObserverTest.IntialConditionPercentage;

% attitude initial error
ObserverTest.EulerAddAmplitude = 10;
ObserverTest.AttitudeInitialConditionAdditive_EulerAngles = 10*randn(1,3);
ObserverTest.AttitudeInitialConditionAdditive_Omega = 5e-3*randn(1,3);
ObserverTest.AttitudeInitialConditionAdditive_EulerAngles = error_enable*ObserverTest.AttitudeInitialConditionAdditive_EulerAngles;
ObserverTest.AttitudeInitialConditionAdditive_Omega = error_enable*ObserverTest.AttitudeInitialConditionAdditive_Omega;

% Settings variables before the estimation process selecting the opportune
% initial estimates, the same for all runs
InitialConditionPercentageMatrix = diag(ObserverTest.IntialConditionPercentage);
ObserverTest.satellites_iner_ECI_0 = satellites_iner_ECI;
ObserverTest.satellites_attitude_0 = satellites_attitude;

% position array index
ObserverTest.PositionArrayIndex = zeros(1,3*ObserverTest.Nagents);

% init condition + position array index
for k = 1:ObserverTest.Nagents
    
    % position array index
    ObserverTest.PositionArrayIndex(1+3*(k-1):3+3*(k-1)) = [1 2 3] + 6*(k-1);
    
    ObserverTest.iner_ECI_0(k,:) = ObserverTest.satellites_iner_ECI_0(1+(k-1)*6:6+(k-1)*6); %initial position state
    ObserverTest.attitude_0(k,:) = ObserverTest.satellites_attitude_0(1+(k-1)*7:7+(k-1)*7); %initial attitude
    ObserverTest.attitude_0(k,1:4) = quatnormalize(ObserverTest.attitude_0(k,1:4));
    
    % position init
    if(ObserverTest.TrueInitialPositions==1)
        ObserverTest.xHatUKF_0(k,:) = ObserverTest.iner_ECI_0(k,:);
    else
        ObserverTest.xHatUKF_0(k,:) = InitialConditionPercentageMatrix*ObserverTest.iner_ECI_0(k,:)'+transpose((ObserverTest.IntialConditionAdditive.*2.*(rand(1,6)-0.5))); %estimated initial state
    end
    
    % attitude init
    if(ObserverTest.attitudeTrueInitialPositions==1)
        ObserverTest.attitude_xHatUKF_0(k,1:7) = ObserverTest.attitude_0(k,:); 
    else
        euladd = error_enable*(pi/180)*ObserverTest.EulerAddAmplitude*(1 + 0.5*randn(1,3));
        ObserverTest.EulerAddStart(k,:) = euladd;
        eulquat = quat2eul(ObserverTest.attitude_0(k,1:4));
        eulstart = eulquat + euladd;
        ObserverTest.attitude_xHatUKF_0(k,1:4) = eul2quat(eulstart);
        
        omega_add = randn(1,3);
        ObserverTest.OmegaAddStart(k,:) = omega_add;
        ObserverTest.attitude_xHatUKF_0(k,5:7) = (omega_add + ObserverTest.attitude_0(k,5:7))'; 
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Switching off the warnings risen by the optimization procedure
warning('off','optim:fminunc:SwitchingMethod');

ObserverTest.AdjacencyMatrix = zeros(ObserverTest.Nagents,ObserverTest.Nagents);
ObserverTest.CSI = zeros(ObserverTest.Na,2*ObserverTest.Na+1); % i sigma points
ObserverTest.CSI_X_meno = zeros(6,2*ObserverTest.Na+1);
ObserverTest.W = zeros(2*ObserverTest.Na+1,1); % i loro pesi

ObserverTest.StartIntervalWindowPercentage = 0.5;
ObserverTest.EndIntervalWindowPercentage = 1;
end


