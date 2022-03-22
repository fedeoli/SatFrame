%% function 
function [B_meas, B_est, B_mean] = BeciEst(x_att, pos, z, DynOpt, ECI)

    % define UTC
    myutc = DynOpt.ObserverTest.myutc;

    % state
    pos = pos(1:3);
    quat = x_att(1:4);
    
    % handle attitude
    q_ECI2Body =  quat; 
    R_ECI2Body = quat2dcm(transpose(q_ECI2Body));

    % convert from ECI to latitude, longitude,  altitude
    LatLongAlt = eci2lla(transpose(pos)*1E3,myutc); 

    % mag_field_vector is in nanotesla, by IGRF11-12
    [mag_field_vector,~,~,~,~] = igrfmagm(max(1000,min(LatLongAlt(3),6E5)),LatLongAlt(1),LatLongAlt(2),decyear(2019,12,15),13); 
    Mag_ECI_est = transpose(mag_field_vector)/norm(mag_field_vector);
    Mag_Body_est = R_ECI2Body*Mag_ECI_est;

    
    % 1st Magnetometer, evaluate magnetic measurements
    Mag_1 = z(1:3);
    if ECI
        Mag1_ECI = pinv(R_ECI2Body) * Mag_1;
    end

    if DynOpt.ObserverTest.nMagneto == 2
        % 2nd Magnetometer, evaluate magnetic measurements 
        % rotation matrix accounting the roll-pitch-yaw rotation that allows to pass from sensor 1 to sensor 2
        R_ECI2Body_v2 = DynOpt.ObserverTest.dcm_v2;
        Mag_2 = pinv(R_ECI2Body_v2)*z(4:6);
        
        if ECI
            Mag2_ECI = pinv(R_ECI2Body) * pinv(R_ECI2Body_v2) * Mag_2;
        end
    end
    
    if ECI
        % store ECI
        if DynOpt.ObserverTest.nMagneto == 1
            B_est = [Mag_ECI_est; Mag_ECI_est];
            B_meas = [Mag1_ECI; Mag1_ECI];
            tmp = mean([Mag1_ECI, Mag_ECI_est],2);
            B_mean = [tmp; tmp];
        elseif DynOpt.ObserverTest.nMagneto == 2
            B_est = [Mag_ECI_est; Mag_ECI_est];
            B_meas = [Mag1_ECI; Mag2_ECI];
    %         tmp = [mean([Mag1_ECI, Mag_ECI_est],2); mean([Mag2_ECI, Mag_ECI_est],2)];
            tmp = [mean([Mag1_ECI, Mag2_ECI, Mag_ECI_est],2); mean([Mag1_ECI, Mag2_ECI, Mag_ECI_est],2)];
            B_mean = tmp;
        else
            B_est = zeros(6,1);
            B_meas = zeros(6,1);
            tmp = zeros(6,1);
            B_mean = tmp;
        end
    else
        % store body
        if DynOpt.ObserverTest.nMagneto == 1
            B_est = [Mag_Body_est; Mag_Body_est];
            B_meas = [Mag_1; Mag_1];
            tmp = mean([Mag_1, Mag_Body_est],2);
            B_mean = [tmp; tmp];
        elseif DynOpt.ObserverTest.nMagneto == 2
            B_est = [Mag_Body_est; Mag_Body_est];
            B_meas = [Mag_1; Mag_2];
            tmp = [mean([Mag_1, Mag_2, Mag_Body_est],2); mean([Mag_1, Mag_2, Mag_Body_est],2)];
            B_mean = tmp;
        else
            B_est = zeros(6,1);
            B_meas = zeros(6,1);
            tmp = zeros(6,1);
            B_mean = tmp;
        end
    end

end