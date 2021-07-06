%% magnetometers function 
function [MagTemp,MagTemp2,GyrosTemp] = AttitudeObserver_GetMeasures_v2_2(Agent_iner_eci,Agent_quaternion,Agent_omega,...
                                                                                                                    Agent_MagBias, Agent_GyroBias,Dtheta,observerRequest,ObserverTest)

% This function provides the magnitude and gyros measurements if observerRequest==0 (data need to be the true one and not the estimated ones)
% adding estimated sensor noise, whereas if observerRequest == 1 it evaluates the shyntetic measures given the estimated vectors
% Dtheta: is the 3 components vector of roll, pitch, yaw in radiants that
% identifies the relative rotation between the first and the second
% magnetometer

MagBias = Agent_MagBias';
GyroBias = Agent_GyroBias';
EulerAngleNoiseOnMag = ObserverTest.EulerAngleNoiseOnMagSigma;
ErrorMu = 1;

if(ObserverTest.AttitudeZeroErrors==1 && observerRequest == 0)
    ErrorMu = 0;
    MagBias = 0*MagBias;
    GyroBias = 0*GyroBias;
    EulerAngleNoiseOnMag = 0;
else
    if(observerRequest == 1)
        %MagBias = Agent_MagBias';%LEAVE THE ZERO!!! ????
        %GyroBias = Agent_GyroBias';%LEAVE THE ZERO!!! ????
        EulerAngleNoiseOnMag = 0;
    end
end

myutc = [2019 12 15 10 20 36]; %CHANGE THIS...??
LatLongAlt = eci2lla(Agent_iner_eci*1E3,myutc); %converto from ECI to latitude, longitude,  altitude
[mag_field_vector,hor_intensity,declinatioon,inclination,total_intensity] ...%IT IS SATURATED FOR MOST TRAJECTORY(ALTITUDE)!!!!
                 = igrfmagm(max(1000,min(LatLongAlt(3),6E5)),LatLongAlt(1),LatLongAlt(2),decyear(2019,12,15),13); %
                %mag_field_vector is in nanotesla, by IGRF11-12
q_ECI2Body =  Agent_quaternion; 
R_ECI2Body = quat2dcm(q_ECI2Body) ;
%ADDING NOISE directly on Euler angles if necessary
if(observerRequest == 0) %noise is added only when the real measurement has to be provided (observerRequest=0)
    tempangle = randn(3,1)*EulerAngleNoiseOnMag;
    R_ECI2Body = angle2dcm(tempangle(1),tempangle(2),tempangle(3))*R_ECI2Body; %Rotation matrix updated with noise
end

MagTemp = (R_ECI2Body)*(mag_field_vector'/(norm(mag_field_vector))) + MagBias;
%add noise with conversion [azimuth,elevation,r] = cart2sph(X,Y,Z) and then back [x,y,z] =sph2cart(azimuth,elevation,r).
if(observerRequest == 0) 
    %[azimuth,elevation,r] = cart2sph(MagTemp(1),MagTemp(2),MagTemp(3));
    %[x,y,z] = sph2cart(azimuth  +ErrorMu*(ObserverTest.headingMagDisplacement + randn(1)*ObserverTest.MagSigmaAzimut),...
     %                                                      elevation+ErrorMu*(ObserverTest.tiltMagDisplacement + randn(1)*ObserverTest.MagSigmaElevation),r);
    %MagTemp =[x,y,z]';
end

% 2nd Magnetometer, evaluate magnetic measurements 
R_ECI2Body = angle2dcm(Dtheta(1),Dtheta(2),Dtheta(3))*quat2dcm(q_ECI2Body) ; %rotation matrix accounting the roll-pitch-yaw rotation that allows to pass from sensor 1 to sensor 2
if(observerRequest == 0) %noise is added only when the real measurement has to be provided (observerRequest=0)
    tempangle = randn(3,1)*EulerAngleNoiseOnMag;
    R_ECI2Body = angle2dcm(tempangle(1),tempangle(2),tempangle(3))*R_ECI2Body; %Rotation matrix updated with noise
end
MagTemp2 = (R_ECI2Body)*(mag_field_vector'/(norm(mag_field_vector)))+ MagBias; %same bias for now....


%Angular velocities measures are simpler
%Agent(k_agent).attitude(5:7,time_i); is the ^c omega(t) of the refererred paper
GyrosTemp = Agent_omega'  + GyroBias + ErrorMu*ObserverTest.GyroGaussianCovariance'.*randn(3,1);
% From: Adaptive Unscented Kalman Filter for Small Satellite Attitude
% Estimation, Soken
%GyrosTemp = (Agent_omega'  + GyroBias + ErrorMu*ObserverTest.GyroGaussianCovariance'.*randn(3,1)) ...
%                        + (R_ECI2Body' )*[0;-sqrt(3.986006130000000e+05/(norm(Agent_iner_eci)^3));0];
if(observerRequest == 0)
   %[azimuth,elevation,r] = cart2sph(GyrosTemp(1),GyrosTemp(2),GyrosTemp(3));
   %[x,y,z] = sph2cart(azimuth  +ErrorMu*ObserverTest.headingGyroDisplacement,...
   %                                                        elevation+ErrorMu*ObserverTest.tiltGyroDisplacement ,r);
   %GyrosTemp =[x,y,z]';
end
                                                       

                                                       
                                                       
               