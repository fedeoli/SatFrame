function Stima = ObserverUKF(i,satellites_iner_ECI,params)

global myGPS ExtractGPS CSIcompleto ErrorAmplitudeGPS diagPi Qi Ri Npassi xHatUKF satellites_iner_ECI_alltimes...
    satellites_iner_ECI_EstimationError_alltimes NormEstimationErrorEachXYZ Na xaUKF ZETA Pzeta Pxz P Q R Pa CSI ...
    W  time_step Ndisturbance N_deputy

satellites_iner_ECI_alltimes(:,i+1) = satellites_iner_ECI;

% Generazione misure fittizie
myGPS = (ErrorAmplitudeGPS*2*(0.5*ones(1,3*(N_deputy+1))-rand(1,3*(N_deputy+1))))' + satellites_iner_ECI(ExtractGPS); %satellites (x,y,z)

% Calolo dei sigma points
[W,CSI] = sigmaPoints(xaUKF, Pa);

%Sigma point evolution (boh)
CSI_X_meno = 0*CSI(1:(6*(N_deputy+1)),:);

for j = 1:2*Na+1
    
    CSIcompleto = CSI(:,j);
    DX = InertialDynamicsIntegratorV11_Est1(CSI(1:(6*(N_deputy+1)),j), params);
    CSI_X_meno(1:(6*(N_deputy+1)),j) = CSI(1:(6*(N_deputy+1)),j) + DX*time_step ;
    %X = rk4_V1_1(@InertialDynamicsIntegratorV11_Est1, tspan,CSI(1:(6*(N_deputy+1)),j) , params);
    %CSI_X_meno(1:(6*(N_deputy+1)),j) = X(:,end);
    
end

CSIcompleto = 0*CSI(:,1);

% stima valor medio
xHatMeno = CSI_X_meno*W;

% stima a priori
xHatUKF(:,i+1) = xHatMeno(1:(6*(N_deputy+1)));

% Aggiornamento matrice di covarianza
MatCOV = 0*P;

for j = 1:2*Na+1
    
    MatCOV = MatCOV + W(j)*(CSI_X_meno(:,j)-xHatMeno)*(CSI_X_meno(:,j)-xHatMeno)';
    
end

Pmeno = MatCOV;

% GPS attesi per ogni sigma point
for j = 1:2*Na+1
    
    ZETA(:,j) = CSI_X_meno(ExtractGPS,j);
    
end

% GPS media attesa
zHatMeno = ZETA*W;

% matrice covarianza Pzeta
MatCOV = 0*Pzeta;

for j = 1:2*Na+1
    
    MatCOV = MatCOV + W(j)*(ZETA(:,j)-zHatMeno)*(ZETA(:,j)-zHatMeno)';
    
end

Pzeta = MatCOV + R;

% matrice covarianza mista Pxz
MatCOV = zeros(length(CSI_X_meno(:,1)),length(ZETA(:,1)));
for j = 1:2*Na+1
    
    MatCOV = MatCOV + W(j)*(CSI_X_meno(:,j)-xHatMeno)*(ZETA(:,j)-zHatMeno)';
    
end

Pxz = MatCOV;

% innovazione
innovation = myGPS - zHatMeno;

% guadagno UKF
K = Pxz*pinv(Pzeta);

% Update stima
xHat = xHatMeno + K*innovation;

% Aggiornamento matrice di covarianza
P = Pmeno - K*Pzeta*K';

xHatUKF(:,i+1) = xHat;
xaUKF = [xHatUKF(:,i+1); zeros((Ndisturbance)*(N_deputy+1),1)];
Pa = blkdiag(P,Q);
satellites_iner_ECI_EstimationError_alltimes(:,i+1) = satellites_iner_ECI - xHatUKF(:,i+1);

for k = 0:N_deputy
    
    NormEstimationErrorEachXYZ(k+1,i+1) = norm(satellites_iner_ECI_EstimationError_alltimes(1+k*(6):3+k*(6),i+1));
    
end

Stima = xHat;

end