function u_imp = PWM_V2_1(t, u, NominalAcceleration, RealAcceleration, dutyPeriod)

%   PWM_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Computes the Pulse Width Mosulation (PWM) of a given continuous control "u". 
%
%   INPUT
%   t: time instant
%   u: continuous control to be modulated
%   params: structure containing the following fields
%       - params.EngineThrust = thrust exerted by the engines
%       - params.dutyValue = width of the sawteeth to be compared to the control
%
%   OUTPUT
%   u_imp: modulated (impulsive) control
%
%   VERSION
%   20190326 V1_1:
%   - First release
%   20200113 V2_1
%   - chenged to receive real acceleration as input

% Extraction of needed constants from the "params" structure
amplValue = NominalAcceleration;

abs_u = abs(u);
threshold = 1e-10;

if abs_u < threshold
    
    abs_u = 0;
    
end

% Sawteeth value at time t
sawteeth = amplValue*(t/dutyPeriod - floor(t/dutyPeriod));

u_imp = 0;

% Comparison between control value and sawteeth profile at time t
if  abs_u >= sawteeth
    
    u_imp = RealAcceleration*sign(u);
    
end

        
end