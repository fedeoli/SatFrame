function [prob, prob_CA_chief, params] = CollisionProbability_V2_3(x_est, P, params)

% Extract quantities from "params" structure
N_deputy = size(x_est,2);
prob = zeros(N_deputy, N_deputy);
prob_CA_chief = zeros(1,N_deputy);
n = params.n;
t_EndCheck = params.DT_CollisionCheck;
if params.just_fired
    CheckIncrement = 30;
else
    CheckIncrement = t_EndCheck;
end
DT_CollisionCheck = 0; max_prob = 0;

while DT_CollisionCheck < t_EndCheck && max_prob < params.CollisionProbabilityThreshold
    % Compute delta t from current time to collision check time
    % dt = t - params.t;
    DT_CollisionCheck = DT_CollisionCheck + CheckIncrement;
    dt = DT_CollisionCheck;
    
    % Number of deputy satellites
    
    
    % Compute State Transition Matrix
    A = [zeros(3,3), eye(3);
        3*n^2, 0, 0, 0, 2*n, 0;
        0, 0, 0, -2*n, 0, 0;
        0, 0, -n^2, 0, 0, 0];
    A_block = kron(eye(N_deputy), A);
    Phi = expm(A_block*dt);
    
    % Compute Process Noise Matrix
    Q = params.Q_filter;
    Q_prop = [Q(1,1)*dt + Q(4,4)*dt^3/3,                0,                              0,                                      Q(1,1)*3*n^2*dt^2/2 + Q(4,4)*dt^2/2,                    -2*n*Q(4,4)*dt^3/3,                 0;
        0,                                        Q(2,2)*dt + Q(5,5)*dt^3/3,      0,                                      2*n*Q(5,5)*dt^3/3,                                      Q(5,5)*dt^2/2,                      0;
        0,                                        0,                              Q(3,3)*dt + Q(6,6)*dt^3/3,              0,                                                      0,                                  -n^2*Q(3,3)*dt^2/2 + Q(6,6)*dt^2/2;
        3*n^2*Q(1,1)*dt^2/2 + Q(4,4)*dt^2/2,      2*n*Q(5,5)*dt^3/3,              0,                                      3*n^4*Q(1,1)*dt^3 + Q(4,4)*dt + 4*n^2*Q(5,5)*dt^3/3,    (Q(5,5) - Q(4,4))*n*dt^2,           0;
        -2*n*Q(4,4)*dt^3/3,                       Q(5,5)*dt^2/2,                  0,                                      (Q(5,5) - Q(4,4))*n*dt^2,                               4*n^2*Q(4,4)*dt^3/3 + Q(5,5)*dt,    0;
        0,                                        0,                              -n^2*Q(3,3)*dt^2/2 + Q(6,6)*dt^2/2,     0,                                                      0,                                  n^4*Q(3,3)*dt^3/3 + Q(6,6)*dt];
    Q_block = kron(eye(N_deputy), Q_prop);
    
    % Propagate state and covariance
    x_est = reshape(x_est, [6*N_deputy,1]);
    x_prop = Phi*x_est;
    P_prop = Phi*P*Phi' + Q_block;
    
    % Compute Collision Probability for each satellite pair (minimum number of operations = n(n-1)/2)
    prob = zeros(N_deputy, N_deputy);
    
    for i = 1:N_deputy
        
        for j = 1 : N_deputy
            
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
                x_rel = x_prop(idx2+1 : idx2+3) - x_prop(idx1+1 : idx1+3);
                
                
                % Integration volume mesh
                %                 x = x_rel(1) - 3*sqrt(P1(1,1)), x_rel(1) - 2*sqrt(P1(1,1)), x_rel(1) - sqrt(P1(1,1))
                x = x_rel(1) - 3*sqrt(P1(1,1)): 3*sqrt(P1(1,1))/10: x_rel(1) + 3*sqrt(P1(1,1));
                y = x_rel(2) - 3*sqrt(P1(2,2)): 3*sqrt(P1(2,2))/10: x_rel(2) + 3*sqrt(P1(2,2));
                z = x_rel(3) - 3*sqrt(P1(3,3)): 3*sqrt(P1(3,3))/10: x_rel(3) + 3*sqrt(P1(3,3));
                %                 x = x_rel(1) - 3*sqrt(P1(1,1)): sqrt(P1(1,1)): x_rel(1) + 3*sqrt(P1(1,1));
                %                 y = x_rel(2) - 3*sqrt(P1(2,2)): sqrt(P1(2,2)): x_rel(2) + 3*sqrt(P1(2,2));
                %                 z = x_rel(3) - 3*sqrt(P1(3,3)): sqrt(P1(3,3)): x_rel(3) + 3*sqrt(P1(3,3));
                [X, Y, Z] = meshgrid(x, y, z);
                
                % Probability density function value over the integration mesh
                F = (1/((2*pi)^(3/2)*sqrt(det(P_rel))))*exp(-0.5*(inv_Prel(1,1)*X.^2 + inv_Prel(2,2)*Y.^2 + inv_Prel(3,3)*Z.^2 + 2*inv_Prel(2,1)*X.*Y + 2*inv_Prel(3,1)*X.*Z + 2*inv_Prel(3,2)*Y.*Z));
                %                 Fx = exp(-0.5*(inv_Prel(1,1)*x.^2));
                %                 Fy = exp(-0.5*(inv_Prel(2,2)*y.^2));
                %                 Fz = exp(-0.5*(inv_Prel(3,3)*z.^2));
                % F = (1/((2*pi)^(3/2)*sqrt(det(P_rel))))*exp(-0.5*(inv_Prel(1,1)*X.^2 + inv_Prel(2,2)*Y.^2 + inv_Prel(3,3)*Z.^2));
                
                %                 for ii = 1:max(size(x))
                %                     int_x = 3*sqrt(P1(1,1))/10*sum((1/((2*pi)^(3/2)*sqrt(det(P_rel))))*exp(-0.5*(inv_Prel(1,1)*X.^2 + inv_Prel(2,2)*Yjj.^2 + inv_Prel(3,3)*Z.^2 + 2*inv_Prel(2,1)*X.*Y + 2*inv_Prel(3,1)*X.*Z + 2*inv_Prel(3,2)*Y.*Z)));
                % Collision probability over the integrated volume
                prob(i,j) = trapz(z, trapz(y, trapz(x,F,3), 2), 1);
                %                 prob(i,j) = (1/((2*pi)^(3/2)*sqrt(det(P_rel))))*trapz(x, Fx)*trapz(y, Fy)*trapz(z, Fz);
                
            end
            
        end
    end
    %
    prob_CA_chief = zeros(1,N_deputy);
    for i = 1:N_deputy
        % Compute the relative covariance matrix (i.e., covariance of the subraction rho_j - rho_i)
        idx1 = 6*(i-1);
        P1 = P_prop(idx1+1 : idx1+3, idx1+1 : idx1+3);
        
        P_rel = P1;
        
        % Compute the inverse of the relative covariance matrix
        inv_Prel = inv(P_rel);
        
        % Compute the relative position vector between satellites j and i
        x_rel = x_prop(idx1+1 : idx1+3);
        
        % Integration volume mesh
        x = x_rel(1) - 3*sqrt(P1(1,1)): 3*sqrt(P1(1,1))/10: x_rel(1) + 3*sqrt(P1(1,1));
        y = x_rel(2) - 3*sqrt(P1(2,2)): 3*sqrt(P1(2,2))/10: x_rel(2) + 3*sqrt(P1(2,2));
        z = x_rel(3) - 3*sqrt(P1(3,3)): 3*sqrt(P1(3,3))/10: x_rel(3) + 3*sqrt(P1(3,3));
        %         x = x_rel(1) - 3*sqrt(P1(1,1)): sqrt(P1(1,1)): x_rel(1) + 3*sqrt(P1(1,1));
        %         y = x_rel(2) - 3*sqrt(P1(2,2)): sqrt(P1(2,2)): x_rel(2) + 3*sqrt(P1(2,2));
        %         z = x_rel(3) - 3*sqrt(P1(3,3)): sqrt(P1(3,3)): x_rel(3) + 3*sqrt(P1(3,3));
        [X, Y, Z] = meshgrid(x, y, z);
        
        % Probability density function value over the integration mesh
        F = (1/((2*pi)^(3/2)*sqrt(det(P_rel))))*exp(-0.5*(inv_Prel(1,1)*X.^2 + inv_Prel(2,2)*Y.^2 + inv_Prel(3,3)*Z.^2 + 2*inv_Prel(2,1)*X.*Y + 2*inv_Prel(3,1)*X.*Z + 2*inv_Prel(3,2)*Y.*Z));
        %         F = (1/((2*pi)^(3/2)*sqrt(det(P_rel))))*exp(-0.5*(inv_Prel(1,1)*X.^2 + inv_Prel(2,2)*Y.^2 + inv_Prel(3,3)*Z.^2));
        
        % Collision probability over the integrated volume
        prob_CA_chief(i) = trapz(z, trapz(y, trapz(x,F,3), 2), 1);
%         if prob_CA_chief(i)>1e-3
%             stop = 1;
%         end
        %         Fx = exp(-0.5*(inv_Prel(1,1)*x.^2));
        %         Fy = exp(-0.5*(inv_Prel(2,2)*y.^2));
        %         Fz = exp(-0.5*(inv_Prel(3,3)*z.^2));
        %         prob_CA_chief(i) = (1/((2*pi)^(3/2)*sqrt(det(P_rel))))*trapz(x, Fx)*trapz(y, Fy)*trapz(z, Fz);
    end
    max_prob = max(max(prob_CA_chief), max(prob(:)));
end
params.DT_CollisionCheck = DT_CollisionCheck ;
%

