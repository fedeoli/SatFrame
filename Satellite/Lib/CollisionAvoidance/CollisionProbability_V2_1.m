function prob = CollisionProbability_V2_1(x_est, P, params)

% Extract quantities from "params" structure
n = params.n;
dt = params.DT_CollisionCheck;

% Number of deputy satellites
N_deputy = size(x_est,2);

% Compute State Transition Matrix
A = [zeros(3,3), eye(3);
     3*n^2, 0, 0, 0, 2*n, 0;
     0, 0, 0, -2*n, 0, 0;
     0, 0, -n^2, 0, 0, 0];
A_block = kron(eye(N_deputy), A);
Phi = expm(A_block*dt);

% Compute Process Noise Matrix
Q = params.Q_filter;
Q_block = kron(eye(N_deputy), Q);

% Propagate state and covariance
x_est = reshape(x_est, [6*N_deputy,1]);
x_prop = Phi*x_est;
P_prop = Phi*P*Phi' + Q_block;

% Compute Collision Probability for each satellite pair (minimum number of operations = n(n-1)/2)
prob = zeros(N_deputy, N_deputy);

for i = 1:N_deputy
    
    for j = 1:N_deputy
       
        if j ~= i
            
            % Compute the relative covariance matrix (i.e., covariance of the subraction rho_j - rho_i)
            idx1 = 6*(i-1);
            P1 = P_prop(idx1+1 : idx1+3, idx1+1 : idx1+3);
            idx2 = 6*(j-1);
            P2 = P_prop(idx2+1 : idx2+3, idx2+1 : idx2+3);
            P12 = P_prop(idx1+1 : idx1+3, idx2+1 : idx2+3);
            P_rel = P1 + P2 - 2*P12;
            
            % Compute the inverse of the relative covariance matrix
            inv_Prel = inv(P_rel);
                       
            % Compute the relative position vector between satellites j and i
            x_rel = x_prop(3*(j-1)+1 : 3*(j-1)+3) - x_prop(3*(i-1)+1 : 3*(i-1)+3);
            
            % Compute the 3-dimensional probability density function associated to the relative position between satellites j and i
            probDensFunc = @(x,y,z) (1/((2*pi)^(3/2)*sqrt(det(P_rel))))*exp(-0.5*(inv_Prel(1,1)*x.^2 + inv_Prel(2,2)*y.^2 + inv_Prel(3,3)*z.^2 + 2*inv_Prel(2,1)*x.*y + 2*inv_Prel(3,1)*x.*z + 2*inv_Prel(3,2)*y.*z));
            
            % Compute the Collision Probability between satellites j and i
            prob(i,j) = integral3(probDensFunc, x_rel(1) - params.CollisionBox(1), x_rel(1) + params.CollisionBox(1),...
                                                x_rel(2) - params.CollisionBox(2), x_rel(2) + params.CollisionBox(2),...
                                                x_rel(3) - params.CollisionBox(3), x_rel(3) + params.CollisionBox(3),'RelTol',1e-9);

        end
        
    end
    
end


end
