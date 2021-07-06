function [error, position_norm, velocity_norm] = ErrorCalculation_V1_2(deputy_rel_LVLH, t, params)

%   ErrorCalculation_V1_2.m
%   Made by Sapienza Gn Lab
%
%   Computes the error between actual (measured) and desired (reference) state.
%
%   INPUT
%   deputy_rel_LVLH: Array (6 x 1) containing deputy's position and velocity at time "t"
%   t: time instant
%   params: structure containing the following fields
%       - params.Control = flag for the activation of the control
%       - params.ReferenceTrajectory = handle to a function which calculates the reference trajectory at each time "t"
%
%   OUTPUT
%   error: Array (6x1) containing the error between actual (measured) and desired (reference) state.
%   position_norm: module of the position error
%   velocity_norm: module of the velocity error
%
%   VERSION
%   20190326 V1_1:
%   - First release
%
%   20190405 V1_2:
%   - Modification in reference trajectory computation procedure


if params.Control   %if the control is applied, compute the errors according to reference trajectory
    
    x_ref = params.ReferenceTrajectory.TrajectoryProfile(t, params.ReferenceTrajectory);      %Compute reference state at time "t"
    error = deputy_rel_LVLH - x_ref;                                                          %Compute the error as difference between actual and referece states
    position_norm = norm(error(1:3));                                                         %Compute the norm of the position error
    velocity_norm = norm(error(4:6));                                                         %Compute the norm of the velocity error
    
else    %if no control is applied set the errors to zero
    
    error = zeros(6,1);
    position_norm = 0;
    velocity_norm = 0;
    
end

end
    