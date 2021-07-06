function [prob, prob_CA_chief, params, prob_appr, prob_appr2, prob_appr_chief, prob_appr_chief2] = CollisionProbability_V2_mala(x_est, P, params, u)

% Extract quantities from "params" structure
N_deputy = size(x_est,2);
prob = zeros(N_deputy, N_deputy);
n = params.n;

if params.t == 0
    
    CheckIncrement = 30;
    
elseif ( max(max(params.DV(:, round(params.t/params.time_step), :))) > 0 || max(max(u(:, round(params.t/params.time_step)+1, :))) > 0 )
    
    CheckIncrement = 30;

else
    
    CheckIncrement = params.DT_CollisionCheck;
    
end

time = 0; 
max_prob = 0;

while time < params.DT_CollisionCheck && max_prob < params.CollisionProbabilityThreshold
    
    % Compute delta t from current time to collision check time
    time = time + CheckIncrement;

    % Compute State Transition Matrix
    A = [zeros(3,3), eye(3);
        3*n^2, 0, 0, 0, 2*n, 0;
        0, 0, 0, -2*n, 0, 0;
        0, 0, -n^2, 0, 0, 0];
    A_block = kron(eye(N_deputy), A);
    Phi = expm(A_block*time);
    
    % Compute Process Noise Matrix
    Q = params.Q_filter;
    Q_prop = [Q(1,1)*time + Q(4,4)*time^3/3,                0,                                  0,                                          Q(1,1)*3*n^2*time^2/2 + Q(4,4)*time^2/2,                        -2*n*Q(4,4)*time^3/3,                   0;
        0,                                                  Q(2,2)*time + Q(5,5)*time^3/3,      0,                                          2*n*Q(5,5)*time^3/3,                                            Q(5,5)*time^2/2,                        0;
        0,                                                  0,                                  Q(3,3)*time + Q(6,6)*time^3/3,              0,                                                              0,                                      -n^2*Q(3,3)*time^2/2 + Q(6,6)*time^2/2;
        3*n^2*Q(1,1)*time^2/2 + Q(4,4)*time^2/2,            2*n*Q(5,5)*time^3/3,                0,                                          3*n^4*Q(1,1)*time^3 + Q(4,4)*time + 4*n^2*Q(5,5)*time^3/3,      (Q(5,5) - Q(4,4))*n*time^2,             0;
        -2*n*Q(4,4)*time^3/3,                               Q(5,5)*time^2/2,                    0,                                          (Q(5,5) - Q(4,4))*n*time^2,                                     4*n^2*Q(4,4)*time^3/3 + Q(5,5)*time,    0;
        0,                                                  0,                                  -n^2*Q(3,3)*time^2/2 + Q(6,6)*time^2/2,     0,                                                              0,                                      n^4*Q(3,3)*time^3/3 + Q(6,6)*time];
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
                x = x_rel(1) - 3*sqrt(P1(1,1)): 3*sqrt(P1(1,1))/10: x_rel(1) + 3*sqrt(P1(1,1));
                y = x_rel(2) - 3*sqrt(P1(2,2)): 3*sqrt(P1(2,2))/10: x_rel(2) + 3*sqrt(P1(2,2));
                z = x_rel(3) - 3*sqrt(P1(3,3)): 3*sqrt(P1(3,3))/10: x_rel(3) + 3*sqrt(P1(3,3));
                [X, Y, Z] = meshgrid(x, y, z);
                
                % Probability density function value over the integration mesh
                F = (1/((2*pi)^(3/2)*sqrt(det(P_rel))))*exp(-0.5*(inv_Prel(1,1)*X.^2 + inv_Prel(2,2)*Y.^2 + inv_Prel(3,3)*Z.^2 + 2*inv_Prel(2,1)*X.*Y + 2*inv_Prel(3,1)*X.*Z + 2*inv_Prel(3,2)*Y.*Z));
                
                % Collision probability over the integrated volume
                prob(i,j) = trapz(z, trapz(y, trapz(x,F,3), 2), 1);
              
%                 if prob(i,j)*100 > 90    
%                     keyboard;
%                 end
                
                % Mahalanobis distance (corrected)
                r_HB2 = params.sat(i+1).r_HB + params.sat(j+1).r_HB;
                r_HB = 4*sqrt( 27*sqrt(P_rel(1,1))*sqrt(P_rel(2,2))*sqrt(P_rel(3,3)) );
%                 r_HB = 3*max([sqrt(P_rel(1,1)), sqrt(P_rel(2,2)), sqrt(P_rel(3,3))]);
                d_MAL = (1 - r_HB/norm(x_rel))*sqrt(x_rel'*inv_Prel*x_rel);
                
                % Upper bound on collision probability
                upper_prob = erfc(d_MAL/sqrt(2));
                
                % Lower bound on collision probability
                lower_prob = (4*pi*r_HB^3/3)/sqrt((2*pi)^3*det(P_rel))*exp(-0.5*x_rel'*inv_Prel*x_rel);
%                 lower_prob = ( 27*sqrt(P_rel(1,1))*sqrt(P_rel(2,2))*sqrt(P_rel(3,3)) )/sqrt((2*pi)^3*det(P_rel))*exp(-0.5*x_rel'*inv_Prel*x_rel); 
%                 r_HB_opt = (prob(i)*sqrt((2*pi)^3*det(P_rel))/exp(-0.5*x_rel'*inv_Prel*x_rel)*3/(4*pi))^(1/3);
                % Approximate collision probability
                prob_appr(i,j) = exp(0.16*log(upper_prob) + 0.8*log(lower_prob));
                
%                 prob_appr(i,j) = lower_prob;
                prob_appr2(i,j) = upper_prob;
                 
            end
            
        end
        
    end
    
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
        [X, Y, Z] = meshgrid(x, y, z);
        
        % Probability density function value over the integration mesh
        F = (1/((2*pi)^(3/2)*sqrt(det(P_rel))))*exp(-0.5*(inv_Prel(1,1)*X.^2 + inv_Prel(2,2)*Y.^2 + inv_Prel(3,3)*Z.^2 + 2*inv_Prel(2,1)*X.*Y + 2*inv_Prel(3,1)*X.*Z + 2*inv_Prel(3,2)*Y.*Z));
        
        % Collision probability over the integrated volume
        prob_CA_chief(i) = trapz(z, trapz(y, trapz(x,F,3), 2), 1);
        
%         if prob_CA_chief(i)*100 > 90    
%             keyboard;
%         end
        
        % Mahalanobis distance (corrected)
        r_HB2 = params.sat(1).r_HB + params.sat(i+1).r_HB;
        r_HB = 4*sqrt( 27*sqrt(P_rel(1,1))*sqrt(P_rel(2,2))*sqrt(P_rel(3,3)) );
%         r_HB = 3*max([sqrt(P_rel(1,1)), sqrt(P_rel(2,2)), sqrt(P_rel(3,3))]);
%         r_HB = 2.2113347e-2;
        
        d_MAL = (1 - r_HB/norm(x_rel))*sqrt(x_rel'*inv_Prel*x_rel);
        
        % Upper bound on collision probability
        upper_prob = erfc(d_MAL/sqrt(2));
        
        % Lower bound on collision probability
        lower_prob = (4*pi*r_HB^3/3)/sqrt((2*pi)^3*det(P_rel))*exp(-0.5*x_rel'*inv_Prel*x_rel); 
%         r_HB_opt = (prob_CA_chief(i)*sqrt((2*pi)^3*det(P_rel))/exp(-0.5*x_rel'*inv_Prel*x_rel)*3/(4*pi))^(1/3);
%         lower_prob = ( 27*sqrt(P_rel(1,1))*sqrt(P_rel(2,2))*sqrt(P_rel(3,3)) )/sqrt((2*pi)^3*det(P_rel))*exp(-0.5*x_rel'*inv_Prel*x_rel);  
%         (4*pi*r_HB_opt^3/3)/sqrt((2*pi)^3*det(P_rel))*exp(-0.5*x_rel'*inv_Prel*x_rel)
        % Approximate collision probability
        prob_appr_chief(1,i) = exp(0.16*log(upper_prob) + 0.8*log(lower_prob));
%         prob_appr_chief(1,i) = lower_prob;
        prob_appr_chief2(1,j) = lower_prob;
        
    end
    
    max_prob = max(max(prob_CA_chief), max(prob(:)));
    
end

params.CollisionTime = time ;

end

