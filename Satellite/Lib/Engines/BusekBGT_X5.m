% Busek BGT-X5
MaxThrust = 0.1*1e-3; % max thrust 0.5 N (nominal) -> 0.1 N (effective)
MinimumImpulseBit = 0.05*1e-3; % minimum impulse bit, 0.05 Ns -> minima durata ?? s
MaximumImpulseBit = []; % not available
Isp = 225;
TotalImpulse = 565; % not available
Resolution = MaxThrust/100; % 5e-3*1e-3; % resolution 5mN
MaximumBurningTime = 60; % [s]

params.PWM = 1;
% All quantities including Newtons (thus meters) must be converted into km*kg*s^-2 (multiply by 1e-3)