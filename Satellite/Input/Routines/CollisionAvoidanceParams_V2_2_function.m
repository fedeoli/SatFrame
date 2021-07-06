function params = CollisionAvoidanceParams_V2_2_function(params,T)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                  COLLISION AVOIDANCE PARAMETERS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    params.DT_CollisionCheck = T/4;                 % Time interval from current time in which the collision probability is computed by propagation of the system's state
    processNoise = [0, 0, 0, 1e-18, 1e-18, 1e-18];  % Noise associated with the propagation in tiem of each state component (position and velocity) used for the collision probability evaluation [0, 0, 0, 1e-20, 1e-20, 1e-20];
    params.Q_filter = diag(processNoise);
    params.CollisionProbabilityThreshold = 1;       % Collision probability threshold which triggers the actuation of the collision avoidance procedure [%]
    params.CA_Time = T;                           % Amount of time to be spent in Collision Avoidance mode when a collision is detected. Only works if params.CollisionAvoidance = 1
    params.MinimumSeparationDistance = 0.1;         % Minimum distance to be guaranteed at the collision point after the actuation of the collisiona voidance maneuver [km]
    params.CollisionFlag = 0;                       % Initialize the value of the flag which shows the presence of collisions
    params.CollisionHolding = 0;                    % Initialize the value of the flag which tells the presence of a calculated collsiiona voidance maneuver not yet executed
end