function [W, CSI] = sigmaPoints(x_m, P_x)
% Rif.:  Julier, Uhlmann: “A New Extension of the Kalman Filter to Nonlinear 
% Systems,” SPIE AeroSense Symposium, April 21–24, 1997. Orlando, FL, SPIE
% e articolo con Manuel Cugliari

n = numel(x_m);
k = 0;

% x_m
% P_x
% pause


CSI = zeros(n,2*n+1);
W = zeros(2*n+1,1);
deltaCSI = zeros(n,2*n+1);

CSI(:,2*n+1) = x_m;
W(2*n+1) = k/(k + n);

A = chol(P_x);

for ind = 1:n
    
    CSI(:,ind) = x_m + sqrt(n+k)*A(ind,:)';
    CSI(:,ind+n) = x_m - sqrt(n+k)*A(ind,:)';    
    W(ind) = 1/(2*(n + k));
    W(ind+n) = 1/(2*(n + k));
    
end

if abs(sum(W)-1) > 0.001
    
    disp('errore pesi: stoppa u programmone');
    pause;
    
end

if sum(abs(CSI*W - x_m)) > 0.001
    
    disp('errore media: stoppa u programmone');
    pause;
    
end

for ind = 1:n
    
    deltaCSI(:,ind) = CSI(:,ind) - x_m;
    deltaCSI(:,ind+n) = CSI(:,ind+n) - x_m;    
    
end

COV = zeros(n);

for ind = 1:2*n
    
    COV = COV + W(ind)*deltaCSI(:,ind)*deltaCSI(:,ind)';
    
end

%  COV
%  P_x

if sum(abs(COV-P_x)) > 0.001
    disp('errore covarianza: stoppa u programmone');
    pause;
end

end