function u_imp = PWM_V1_1(t, u, params)

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

% Extraction of needed constants from the "params" structure
amplValue = params.MaxThrust;
dutyPeriod = params.dutyPeriod;

abs_u = abs(u);
threshold = 1e-8;

if abs_u < threshold
    
    abs_u = 0;
    
end

% Sawteeth value at time t
sawteeth = amplValue*(t/dutyPeriod - floor(t/dutyPeriod));

u_imp = 0;

% Comparison between control value and sawteeth profile at time t
if  abs_u > sawteeth
    
    u_imp = amplValue*sign(u);
    
end

        
end