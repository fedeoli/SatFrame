%% function 
function [B_inv, B_est, B_mean] = BeciEst(x_att, pos, z, DynOpt)

    % define UTC
    myutc = DynOpt.ObserverTest.myutc;

    % convert from ECI to latitude, longitude,  altitude
    pos = pos(1:3);
    quat = x_att(1:4);

    LatLongAlt = eci2lla(transpose(pos)*1E3,myutc); 

    % mag_field_vector is in nanotesla, by IGRF11-12
    [mag_field_vector,~,~,~,~] = igrfmagm(max(1000,min(LatLongAlt(3),6E5)),LatLongAlt(1),LatLongAlt(2),decyear(2019,12,15),13); 
    Mag_ECI_est = transpose(mag_field_vector/(norm(mag_field_vector)));

    % handle attitude
    q_ECI2Body =  quat; 
    R_ECI2Body = quat2dcm(transpose(q_ECI2Body));

    % 1st Magnetometer, evaluate magnetic measurements
    Mag_1 = z(1:3);
    Mag1_ECI = pinv(R_ECI2Body) * Mag_1;

    % 2nd Magnetometer, evaluate magnetic measurements 
    % rotation matrix accounting the roll-pitch-yaw rotation that allows to pass from sensor 1 to sensor 2
    Mag_2 = z(4:6);
    R_ECI2Body_v2 = DynOpt.ObserverTest.dcm_v2;
    
    Mag2_ECI = pinv(R_ECI2Body) * pinv(R_ECI2Body_v2) * Mag_2;
    
    % store
    B_est = Mag_ECI_est;
    B_inv = [Mag1_ECI; Mag2_ECI];
    B_mean = mean([Mag1_ECI, Mag2_ECI, Mag_ECI_est],2);

end