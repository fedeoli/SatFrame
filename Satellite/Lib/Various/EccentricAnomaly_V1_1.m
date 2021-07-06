function E = EccentricAnomaly_V1_1(M, e, E0)

%   moe2ooe_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Uses Newton_rhapson method to compute satellite's Eccentric anomaly E from eccentricity e, mean anomaly M and an initial guess E0.
%
%   INPUT
%   M: Satellite's mean anomaly.
%   e: Satellite's eccentricity.
%   E0: First guess of eccentric anomaly.
%
%   OUTPUT
%   E: Satellite's eccentric anomaly.
%
%   VERSION
%   20190304 V1_1:
%   -  First Release


% Initial settings for the Newton-Rhapson method
E = E0;
err = 1000;
toll = 1e-5;
i = 0;

% Newton-Rhapson method
while (err > toll) && (i < 10000)
    
    Esuc = E-(E-e*sin(E)-M)/(1-e*cos(E));
    err = abs(E-Esuc);
    E = Esuc;
    i = i + 1;
    
end

% Threshold on the maximum iteration number allowed
if i == 10000
    
    disp('Error in EccentricAnomaly_V1_1.m: Wring initial guess. Cannot compute E. Maximum number of iterations reached.')
    
end

end