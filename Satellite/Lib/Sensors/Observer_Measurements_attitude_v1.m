%% magnetometers function 
function DynOpt = Observer_Measurements_attitude_v1(satellites_iner_ECI,satellites_attitude, DynOpt, params)

    % This function provides the magnitude and gyros measurements if observerRequest==0 (data need to be the true one and not the estimated ones)
    % adding estimated sensor noise, whereas if observerRequest == 1 it evaluates the shyntetic measures given the estimated vectors
    % Dtheta: is the 3 components vector of roll, pitch, yaw in radiants that
    % identifies the relative rotation between the first and the second
    % magnetometer

    % general info
    nagent = DynOpt.ObserverTest.Nagents;

    %%% bias definition %%%
    MagBias = DynOpt.ObserverTest.MagBias;
    GyroBias = DynOpt.ObserverTest.GyroBias;
    SunBias = DynOpt.ObserverTest.SunBias;

    % define UTC
    myutc = DynOpt.ObserverTest.myutc;

    % convert from ECI to latitude, longitude,  altitude
    for n=1:nagent
        pos = satellites_iner_ECI(1+6*(n-1):3+6*(n-1));
        quat = satellites_attitude(1+7*(n-1):4+7*(n-1));
        omega = satellites_attitude(5+7*(n-1):7+7*(n-1));
        x_hat = [quat; omega];

        LatLongAlt = eci2lla(transpose(pos)*1E3,myutc); 

        % noise
        mag_noise_1 = DynOpt.noise_enable*MagBias(1:3) + DynOpt.noise_enable*diag(DynOpt.ObserverTest.MagGaussianCovariance)*randn(3,1);
        mag_noise_2 = DynOpt.noise_enable*MagBias(4:6) + DynOpt.noise_enable*diag(DynOpt.ObserverTest.MagGaussianCovariance)*randn(3,1);
        gyro_noise = DynOpt.noise_enable*GyroBias + DynOpt.noise_enable*diag(DynOpt.ObserverTest.GyroGaussianCovariance)*randn(3,1);
        sun_noise = DynOpt.noise_enable*SunBias + DynOpt.noise_enable*diag(DynOpt.ObserverTest.SunGaussianCovariance)*randn(3,1);

        % mag_field_vector is in nanotesla, by IGRF11-12
        [mag_field_vector,~,~,~,~] = igrfmagm(max(1000,min(LatLongAlt(3),6E5)),LatLongAlt(1),LatLongAlt(2),decyear(2019,12,15),13); 
        mag_field_norm = transpose(mag_field_vector/(norm(mag_field_vector)));

        % measures
        Mag1_ECI = mag_field_norm + mag_noise_1;
        Mag2_ECI = mag_field_norm + mag_noise_2;

        % handle attitude
        q_ECI2Body =  quat; 
        R_ECI2Body = quat2dcm(transpose(q_ECI2Body));

        % 1st Magnetometer, evaluate magnetic measurements
        Mag_1 = R_ECI2Body * Mag1_ECI;

        % 2nd Magnetometer, evaluate magnetic measurements 
        % rotation matrix accounting the roll-pitch-yaw rotation that allows to pass from sensor 1 to sensor 
        R_ECI2Body_v2 = DynOpt.ObserverTest.dcm_v2; 
        Mag_2 = R_ECI2Body_v2 * R_ECI2Body * Mag2_ECI;


        % Angular velocities measures are simpler
        Gyro = omega  + gyro_noise;
        
        % Sun sensor
        S_body = Sun_measure(x_hat,pos,DynOpt,params);
        S_body = S_body + sun_noise;
        
        
        % assign to agents
        DynOpt.y_Mag(1+6*(n-1):6+6*(n-1),DynOpt.iter) = [Mag_1; Mag_2];
        DynOpt.y_Gyro(1+3*(n-1):3+3*(n-1),DynOpt.iter) = Gyro;
        DynOpt.y_Sun(1+3*(n-1):3+3*(n-1),DynOpt.iter) = S_body;

    end
end
                                                       

                                                       
                                                       
               