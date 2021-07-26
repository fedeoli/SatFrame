%% %%%%%%%%%%%%%%%%%%%%% SUN SENSOR INIT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [DynOpt, params] = Sun_init(DynOpt, params)
    % Determine Sun Vec in Inertial Frame - see https://en.m.wikipedia.org/wiki/Position_of_the_Sun

    %%%% ECLIPTIC COORDINATES %%%%
    % # days since greenwich noon 01/01/2000
    T = 7551; % 3 settembre 2020
    CONST.ua = 1.496e+08;
    params.CONST.ua = CONST.ua;

    % mean anomaly (ref 4.1)
    Msun=357.528+0.9856003*T;  

    %ecliptic longitude (ref 4.2)
    Vsun=280.461+0.9856474*T+1.915*sind(Msun)+0.020*sind(2*Msun); 

    %distance sun-Earth in ua
    R = 1.000014-0.01671*cosd(Msun)-0.00014*cosd(2*Msun); 

    %%%% EQUATORIAL COORDINATES %%%%
    % ref 4.3
    obliquity=23.4393-0.0000004*T;

    %%% Conversion in equatorial coordinates %%%
    right_asc = atan2(cos(obliquity)*sind(Vsun), cosd(Vsun));
    decl = asin(sind(obliquity)*sind(Vsun));

    %%% Conversion in Rectangular equatorial coordinates %%%
    % ref 4.4
    Xsun=R*cosd(Vsun);
    Ysun=R*cosd(obliquity)*sind(Vsun);
    Zsun=R*sind(obliquity)*sind(Vsun);
    Si=[Xsun Ysun Zsun];
    Si=Si./norm(Si);


    % store
    params.Si = Si;
    DynOpt.ObserverTest.Si = Si;


    % symbolic definition - sun position in ECI
    params.Psun = DynOpt.ObserverTest.Psun;

    % symbolic - sun position in Body (ref 4.5)
    DynOpt.ObserverTest.Pbody = DynOpt.ObserverTest.dcm*transpose(DynOpt.ObserverTest.Psun);

    %%%%%%%%%%%%%%%%%%%%%%% ALBEDO EFFECT INIT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %data from "Coarse Sun Sensing for Attitude Determination of a CubeSat"
    % John Gaebler Summa Cum Laude Thesis Bachelor of Science in Aerospace Engineering

    % Area of the cell - albedo model (m^2)
    A=30.16/100^2;

    % reflectivity of the cell - max value
    eff=0.251; 

    % W/m^2 Sun incidence energy avg - EM0 in Bhanderi
    K=1367; 

    % Max theoretical reflected intensity - sun normal to sensor (0 deg)
    params.I0 = A*K*cos(0)*eff; % ref 4.6

    % dcm matrix from ECI to ECI Fixed
    dcmecef = dcmeci2ecef('IAU-2000/2006',DynOpt.ObserverTest.myutc);
    params.dcmecef = dcmecef;
    
    % toolbox
    params.path = [pwd '/Satellite/Lib/Sensors/AlbedoToolbox-1.0/AlbedoToolbox-1.0/refl-data'];

    % reflectivity matric (180x288)
    params.refl = load([params.path '/2005/ga050101-051231.mat']);
    
    % eclipse flag
    DynOpt.ObserverTest.Eclipse = 0;
end