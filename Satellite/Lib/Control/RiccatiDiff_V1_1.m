function [K, G] = RiccatiDiff_V1_1(params)

%   RiccatiDiff_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Integrates Riccati Differential Equation for the Augmented LQR control strategy and returns control matrices K (feedback) and G (feedforward).
%
%   INPUT
%   params: structure containing the following fields
%       - params.tRiccati = Time step for Riccati differential equation solving [s].
%       - params.tfinal = Simulation final time [s]
%       - params.time_step = Integration time step [s]
%       - params.ALQR_J2 = flag for the activation of the J2 linearized Augmented LQR method
%
%   OUTPUT
%   K: Augmented LQR Feedback control matrix
%   G: Augmented LQR Feedforward control matrix
%
%   VERSION
%   20190326 V1_1:
%   - First release


RiccatiTimeStep = params.tRiccati;                  % Time step for Riccati differential equation solving
tfinal = params.tfinal;                             % Simulation final time
time_step = params.time_step;                       % Integration time step
tfinalRIC = tfinal + time_step;                     % Riccati equation integration final time
params.tfinalRiccati = tfinalRIC;                   % Storing of the integration final time
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);   % Integration options
tau_int = 0:RiccatiTimeStep:tfinalRIC;              % Integration time array
KG0(1:42) = 0;                                      % Initial conditions

if params.ALQR_J2   % Enters here if the J2 Augmented model has been selected
    
    [~, KGtau] = ode45(@RiccatiKG_J2_V1_1, tau_int, KG0, options, params);
    
else    % Enters here if the Augmented model without J2 has been selected
    
    [~, KGtau] = ode45(@RiccatiKG_HCW_V1_1, tau_int, KG0, options, params);
    
end

% Reverting K and G from tau (regressive time) to t (progressive time)
K = KGtau(end:-1:1, 1:36);
G = KGtau(end:-1:1, 37:42);

end