% Hyperion PM200  
MaxThrust = 0.5*1e-3; % max thrust 0.5 N
MinimumImpulseBit = 0.035*1e-3; % minimum impulse bit, 0.035 Ns -> minima durata 0.7 s
MaximumImpulseBit = 5*1e-3; % maximum impulse bit, 0.5 Ns -> massima durata 10s
Isp = 285;
TotalImpulse = 0; % not available
Resolution = MaxThrust/100; % 5e-3*1e-3; % resolution 5mN
MaximumBurningTime = 30; % [s]

% All quantities including Newtons (thus meters) must be converted into km*kg*s^-2 (multiply by 1e-3)