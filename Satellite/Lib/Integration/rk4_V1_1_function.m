function [X, DynOpt] = rk4_V1_1_function(f, tspan, x0, params, DynOpt)

%   rk4_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Numerical integration of a system of first order differential equations through fixed-step, Runge Kutta 4th order method. Gives as output a (N x
%   M) matrix where N is the number of state components and M is the number of time instants.
%
%   INPUT
%   f: function to be integrated
%   tspan: Vector containing all the time instants
%   x0: state vector at t0. Must be a clumn vector.
%   params: Structure which must contain all the fileds needed inside function "f"
%
%   OUTPUT
%   X: Array (N x M) where N is the number of state components and M is the number of time instants. j-th column is the state integrated at time j.
%
%   VERSION
%   20190301 V1_1:
%   -  First Release


N = length(x0);             % number of state's components
M = length(tspan);         % numer of time steps
dt = tspan(2) - tspan(1);   % time step

% Matrices allocation
X = zeros(N,M);
X(:,1) = x0;
K1 = zeros(N,M);

for i = 1:M-1
    
    % State and time at ti
    x = X(:,i);
    
    % Runge Kutta 4
    K1(:,i) = f(x, params,DynOpt);
    K2 = f(x + K1(:,i)*dt/2, params,DynOpt);
    K3 = f(x + K2*dt/2, params,DynOpt);
    [K4, temp] = f(x + K3*dt, params,DynOpt);
    
    % Solution at ti+1
    X(:,i+1) = x + (dt/6)*(K1(:,i) + 2*K2 + 2*K3 + K4);
    
    % update DynOpt
    DynOpt = temp;
    
end


end