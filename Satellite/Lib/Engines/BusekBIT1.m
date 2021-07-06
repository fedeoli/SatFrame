% Busek BIT-1
MaxThrust = 185*1e-6*1e-3; % max thrust 185 microN
MinimumImpulseBit = []; % not available
MaximumImpulseBit = []; % not available
Isp = 1600;
TotalImpulse = []; % not available
Resolution = MaxThrust/100; % not available
MaximumBurningTime = 200; % [s]

params.PWM = 0;
% All quantities including Newtons (thus meters) must be converted into km*kg*s^-2 (multiply by 1e-3)