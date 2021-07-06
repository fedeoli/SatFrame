function deputy_iner_ECI = dcoe2iner_V1_1(Delta_par, chief_coe, params)

%   dcoe2iner_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Transforms a single deputy satellite's differential orbital elements into inertial coordinates expressed in ECI reference frame.
%
%   INPUT
%   Delta_par: Array (1 x 6) containing deputy's differential orbital elements.
%   chief_coe: Array (1 x 6) containing chief's orbital elements.
%   params: Structure containing the following fields:
%       - params.mi: Earth's planetary constant [km^3/s^2]
%
%   OUTPUT
%   deputy_iner_ECI = array (6 x 1) containing deputy's inertial position and velocity expressed in ECI reference frame.


mi = params.mi; % Earth's planetary constant

% Extraction of deputy's differential orbital elements
Da = Delta_par(1);
De = Delta_par(2); 
Di = Delta_par(3);
Dom = Delta_par(4);
DOM = Delta_par(5);
Dth = Delta_par(6);
Delta_par = [Da, De, Di, Dom, DOM, Dth];    % array of deputy's differential orbital elements
coe = chief_coe + Delta_par;                % array of deputy's orbital elements

% Transformation from Classical Orbital Elements to inertial position and velocity expressed in ECI reference frame
deputy_iner_ECI = coe2rv_V1_1(coe, mi);          

end