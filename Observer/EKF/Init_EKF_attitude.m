%% EKF PROJECT 
% Author: Daniele Giorgi, Federico Oliva
% Date: 6/09/2020
%
% POSSIBILI UPDATE/PROBLEMI
%
% 4) Il modello Albedo sembra funzionare bene, per testarlo però con la
% configurazione attuale del satellite_iner_ECI parte dall'eclissi, quindi
% mentre se si usa come coordinate sferiche del sole quelle date dalla tesi
% di riferimento:
%
%ObserverTest.Sunsph = [0 1.17 CONST.ua]; 
%
%allora si può apprezzare l'effetto Albedo che sporca effettivamente le
%misure. Altrimenti bisogna simulare perlomeno un'orbita completa.
%Bisogna inoltre aggiungere i plot per differenti altitudini e o longitudini,
% ma può essere fatto facilmente attraverso la funzione ecef2lla(sat_ecef),
%in quanto è stato tutto convertito in Earth-Centered-Earth_Fixed coordinates. 
%% Init Section
fprintf('Setting Observer parameters\n');
syms Ixx Iyy Izz;       % System Inertia
syms wx wy wz;          % System angular velocity
syms q1 q2 q3 q0;       % Quaternion
syms Bx1 By1 Bz1 Bx2 By2 Bz2;          % Magnetometer measures

% Global vars definition
global ObserverTest map Agent
 
%% INITIAL CONDITIONS
%  x_m: previous estimated state [quaternion, omegas, bias_mag, bias_gyro],  the first component of the quaternion is the scalar part
%  P_x: previous covariance matrices

% Kinematic and measures init 

% angular velocity measured by gyro - Body frame
omega_Body2ECI_Body = [wx wy wz];
w_body = omega_Body2ECI_Body;

% quaternion in Body frame
q_ECI2Body = [q0 q1 q2 q3];
q_sym = [q0 q1 q2 q3];
% attitude from quaternion - vector elements
attitude_ECI = q_ECI2Body(2:4);
% junk assignment (?)
qin = q_ECI2Body;

% Magneto measures - Inertial frame
Magneto1 = [Bx1 By1 Bz1];
BI_1 = Magneto1;
params.BI1_sym = BI_1;

if ObserverTest.nMagneto == 2
    Magneto2 = [Bx2 By2 Bz2];
    BI_2 = Magneto2;
    params.BI2_sym = BI_2;
end

% Dynamics init - inertia and mass
In = [Ixx, Iyy, Izz];

% Cosines matrix from quaternion
dcm = sym('dcm', [3,3]);

% MATLAB VERSION
dcm(1,1,:) = qin(:,1).^2 + qin(:,2).^2 - qin(:,3).^2 - qin(:,4).^2;
dcm(1,2,:) = 2.*(qin(:,2).*qin(:,3) + qin(:,1).*qin(:,4));  
dcm(1,3,:) = 2.*(qin(:,2).*qin(:,4) - qin(:,1).*qin(:,3));
dcm(2,1,:) = 2.*(qin(:,2).*qin(:,3) - qin(:,1).*qin(:,4));
dcm(2,2,:) = qin(:,1).^2 - qin(:,2).^2 + qin(:,3).^2 - qin(:,4).^2;
dcm(2,3,:) = 2.*(qin(:,3).*qin(:,4) + qin(:,1).*qin(:,2));
dcm(3,1,:) = 2.*(qin(:,2).*qin(:,4) + qin(:,1).*qin(:,3));
dcm(3,2,:) = 2.*(qin(:,3).*qin(:,4) - qin(:,1).*qin(:,2));
dcm(3,3,:) = qin(:,1).^2 - qin(:,2).^2 - qin(:,3).^2 + qin(:,4).^2;

% Magneto measures - Body frame
B_body_1 = dcm*transpose(BI_1);

if ObserverTest.nMagneto == 2
    B_body_2 = dcm*transpose(BI_2);
end

% Angular velocity - Inertial frame
wI = transpose(dcm)*transpose(omega_Body2ECI_Body);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ObserverTest.Sun == 1
    Sun_init
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Vett. di stato
X = [q_ECI2Body, omega_Body2ECI_Body]; 

if ObserverTest.Sun == 0
    if ObserverTest.nMagneto == 1 
        h = [B_body_1; transpose(w_body)];
    elseif ObserverTest.nMagneto == 2
        h = [B_body_1 ; B_body_2; transpose(w_body)];
    end
elseif ObserverTest.Sun == 1
    if ObserverTest.nMagneto == 1 
        h = [B_body_1; hss; transpose(w_body)];
    elseif ObserverTest.nMagneto == 2
        h = [B_body_1 ; B_body_2; hss; transpose(w_body)];
    else
        h = [hss; transpose(w_body)];
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% MANCANO LE MATRICI DI COVARIANZA DI PROCESSO E MISURE %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Dynamics setup section
% satellite equation
f = attitude_Kin_eqs(omega_Body2ECI_Body, q_ECI2Body.', In, params, dcm); 

% Numerical substitution
% f = subs(f, M, [0.89, .87, .61]);
f = subs(f, In, [params.sat(1).I(1,1) params.sat(1).I(2,2) params.sat(1).I(3,3)]);

% linearization of satellite equations
A = jacobian(f,X);    
H = jacobian(h,X);  

%% disturbance initial conditions
%%%%% Fedeoli edit %%%%%
ObserverTest.Ndisturbance_att = 7; %acting on the traslational dynamics of each agent
ObserverTest.Ndisturbance_pos = 6; %acting on the traslational dynamics of each agent
ObserverTest.Qi_att = eye(ObserverTest.Ndisturbance_att)*1E-2;
ObserverTest.Pi_att = eye(7)*1E-2;  %P of the UKF estimator, so on below

ObserverTest.Pi_pos = eye(ObserverTest.Ndisturbance_pos)*1E-2;
ObserverTest.Qi_pos = eye(ObserverTest.Ndisturbance_pos)*1E-2;


% measurement noise
m = size(h,1); 
ObserverTest.Ri_att = eye(m)*1E-3;%relativa alla misura del GPS e delle velocit? stimate
ObserverTest.Ri_pos = eye(3)*1E-3;

for k = 1:ObserverTest.Nagents
   Agent(k).Q_att =  ObserverTest.Qi_att;
   Agent(k).P_att =  ObserverTest.Pi_att;
   Agent(k).R_att =  ObserverTest.Ri_att;
   Agent(k).P_pos =  ObserverTest.Pi_pos;
   Agent(k).Q_pos =  ObserverTest.Qi_pos;
   Agent(k).R_pos =  ObserverTest.Ri_pos;   
end

%%% MAPPING STRUCTURE %%%
map.f = f;
map.h = h;
map.A = A;
map.H = H;
map.omega_Body2ECI_Body = omega_Body2ECI_Body;
map.q_sym = q_sym;
map.X = X;


if ObserverTest.nMagneto == 1
   map.Magneto = transpose(Magneto1); 
else
   map.Magneto = transpose([Magneto1, Magneto2]);
end

dimOut = length(map.Magneto);
map.f_obs = [map.f; zeros(dimOut,1)];
map.X_obs = [map.X, transpose(map.Magneto)];
ObserverTest.n_der = 1;

if ObserverTest.obsAnalysis == 1
    [theta,dtheta,~] = ObsAnalysis(map,ObserverTest.n_der,0,0,1);
    ObserverTest.dtheta = dtheta;
end



