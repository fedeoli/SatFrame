% First satellite (chief)
params.sat(1).TrajectoryProfile = [];
params.sat(1).xRef = [];
params.sat(1).yRef = [];
params.sat(1).zRef = [];
params.sat(1).Phase = [];
params.sat(1).n = [];
params.sat(1).WaitingTime = [];
params.sat(1).Burn_executed = [];
params.sat(1).IsBurning = [];
params.sat(1).att_threshold = [];
params.sat(1).Thruster.MaxThrust = [];
params.sat(1).Thruster.MinimumImpulseBit = [];
params.sat(1).Thruster.MaximumImpulseBit = [];
params.sat(1).Thruster.Isp = [];
params.sat(1).Thruster.TotalImpulse = [];
params.sat(1).Thruster.Resolution = [];
params.sat(1).Thruster.MaximumBurningTime = [];
params.sat(1).FiniteBurnTime = [];
params.sat(1).kp = kp_chief;
params.sat(1).kd = kd_chief;
params.sat(1).IsWorking = 1;
params.sat(1).CA_mode = 0;
params.sat(1).CA_ResidualTime = 0;

% Create deputies structure
for j = 1:N_deputy
    
    params.sat(j+1).n = n;
    params.sat(j+1).WaitingTime = WaitingTime;
    params.sat(j+1).Burn_executed = 1;
    params.sat(j+1).IsBurning = 0;
    params.sat(j+1).att_threshold = att_threshold;
    params.sat(j+1).CanManeuverAttitude = 0;
    params.sat(j+1).Thruster.MaxThrust = MaxThrust;
    params.sat(j+1).Thruster.MinimumImpulseBit = MinimumImpulseBit;
    params.sat(j+1).Thruster.MaximumImpulseBit = MaximumImpulseBit;
    params.sat(j+1).Thruster.Isp = Isp;
    params.sat(j+1).Thruster.TotalImpulse = TotalImpulse;
    params.sat(j+1).Thruster.Resolution = Resolution;
    params.sat(j+1).Thruster.MaximumBurningTime = MaximumBurningTime;
    params.sat(j+1).FiniteBurnTime = 0;
    params.sat(j+1).kp = kp_deputy;
    params.sat(j+1).kd = kd_deputy;
    params.sat(j+1).CA_mode = 0;
    params.sat(j+1).CA_ResidualTime = params.CA_ParkingTime;
    
end
