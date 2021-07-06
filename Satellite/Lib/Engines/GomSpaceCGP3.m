% GomSpace NanoProp CGP3
MaxThrust = 0.001*1e-3; % max thrust 1 mN
MinimumImpulseBit = 0.035*1e-3; % minimum impulse bit, 0.035 Ns
MaximumImpulseBit = []; % maximum impulse bit (not given)
Isp = 110;
TotalImpulse = 40; % not available
Resolution = 10*1e-6*1e-3; % resolution 10 microN
MaximumBurningTime = 120; % [s]

params.PWM = 1;
% All quantities including Newtons (thus meters) must be converted into km*kg*s^-2 (multiply by 1e-3)