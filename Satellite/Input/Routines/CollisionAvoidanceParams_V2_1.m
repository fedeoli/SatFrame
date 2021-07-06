%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                  COLLISION AVOIDANCE PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

params.DT_CollisionCheck = 2*pi/n/4;
params.CA_ParkingTime = 2*pi/n;                 % Amount of time to be spent in Collision Avoidance mode in the safe parking orbit
processNoise = [0, 0, 0, 1e-18, 1e-18, 1e-18];  %[0, 0, 0, 1e-20, 1e-20, 1e-20];
params.Q_filter = diag(processNoise);
params.CollisionBox = 3*[1, 1, 1]*1e-3;
params.CollisionProbabilityThreshold = 1;       % [%]
params.MinimumSeparationDistance = 0.1;         % [km]