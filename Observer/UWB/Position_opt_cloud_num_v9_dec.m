%% Geometric method
% This method compute the position estimate for a single agent.
% Input: 
%   1) Chi: (nagent x 3) matrix containing the a priori estimate for all
%   the agents.
%   2) GPS: (1 x 3) matrix containing the GPS measures for the agent
%   considered.
%   3) adjmat_UWB: (nagent x nagent) matrix ontaining the set of relative
%   distances measured by UWB. 
%   4) j_select: number of the agent considered.
%   5) theta: first parameter
%   6) beta: second parameter
%   7) Check_dist: boolean flag used to trigger the security check on the
%   final estimate. 
% Output:
%   1) opt: data structure containing the set of nagent position estimates.
function opt = Position_opt_cloud_num_v9_dec(Chi, GPS, adjmat_UWB, j_select, theta, beta, check_dist, packet_UWB, RecoveryPos, DynOpt)
%     fprintf("Optimizing - geometric method\n");
    
    % nagent
    nagent = size(Chi,1);
    
    % safe check
    dist_thresh = 5e-2;
    
    % projection version
    version = 2;
    
    % Versor Vji computation - Chi values: using the Chi values the direction
    % between agent i and agent j is computed. Data are stored in a matrix.
    % Index (i,k) defines the kth component of the vector connecting agent i
    % to agent j_select.
    % REMARK: the unity vector is the same for v1 and v2 because the
    % sign/orientation of the projection is incorporated in the distance
    % which is interpreted as a vector rather than a non-negative scalar
    % value
    direction_matrix_Chi = zeros(nagent,3);
    for i = 1:nagent
      if i ~= j_select
            direction_matrix_Chi(i,:) = (Chi(j_select,:) - Chi(i,:))/norm(Chi(j_select,:) - Chi(i,:));
      end
    end
    
    % relative distances computed from Chi position - create an alternative
    % adjacency matrix
    adjmat_Chi = zeros(1,nagent);
    for i = 1:nagent
      if i ~= j_select
        adjmat_Chi(i) = norm(Chi(j_select,:) - Chi(i,:));
      end
    end
    
    % define distance used (average over different adjmat?)
    
    if version == 1
        % simple UWB measure
        d = adjmat_UWB(j_select,:);
    else
        % average between Chi and UWB
        d = 0.5*(adjmat_UWB(j_select,:)-adjmat_Chi);
    end

    % Position according to UWB measures. Agent i position can be derived
    % from agent j assuming to know Dji and Vji. Thus, each agent's position
    % can be estimated by all the others, resulting in nagent-1 different
    % estimates. Packet loss is implenented as a multiplying factor
    Chi_UWB = zeros(nagent,3);
    for i = 1:nagent
        if i ~= j_select
            for k = 1:3
                if version == 1
                    Chi_UWB(i,k) = Chi(i,k) + direction_matrix_Chi(i,k)*d(i);
                else
                    Chi_UWB(i,k) = Chi(j_select,k) + direction_matrix_Chi(i,k)*d(i);
                end
            end
            % packet loss
            Chi_UWB(i,:) = packet_UWB(j_select,i)*Chi_UWB(i,:);
        end
    end

    % New position estimate. All the estimated positions of agent j_select from agents
    % j are averaged, obtaining the centroid of the UWB position points cloud.
    % This value is averaged once more with agent i GPS position measured by
    % agent itself. The average is on the number of agents correctly
    % received
    Chi_estimate = zeros(1,3);
    centroid = zeros(1,3);
    nagent_received = sum(packet_UWB(j_select,:));
    if nagent_received ~= 0
        for k = 1:3
            centroid(k) = sum(Chi_UWB(:,k))/(nagent_received);
        end  
    else
        % count the GPS only
        centroid = reshape(RecoveryPos(1+3*(j_select-1):3+3*(j_select-1)),1,3);
    end
    
    % single agent position estimate
    for k = 1:3
        Chi_estimate(k) = beta*Chi(j_select,k) + (1-beta)*(((1-theta)*centroid(k) + theta*GPS(k)));          
    end
    
    % check if the new position is too far from the starting point
    if check_dist == 1
       if  (norm(Chi(j_select,:) - Chi_estimate) >= dist_thresh)
               Chi_estimate = Chi(j_select,:);
       end
    end
    
    % cycle over the agents
    sigma_p = zeros(3,nagent);
    for i=1:nagent
        if i ~= j_select
            
            
            %%% used vals %%%
            % MEAS
            dij = adjmat_Chi(i);
            dij_vec = Chi(j_select,:) - Chi(i,:);
            dij_UWB = adjmat_UWB(j_select,i);
            
            xi = Chi(i,:);
            xj = Chi(j_select,:);
            
            %%% SIGMA %%%
            sigma_GPS = DynOpt.ObserverTest.GPSGaussianCovariance(1:3).^2;
            sigma_UWB = ones(1,3).*DynOpt.ObserverTest.ErrorAmplitudeUWB.^2;

            N = nagent;

%             T = zeros(3);
%             for a=1:3
%                 for b=a:3
%                     if a==b
%                         T(a,b) = 0.5*((1-theta)^2/(N-1)*(2*sigma_GPS(a)*dij^2*(dij^2+dij_UWB^2) + (xi(a) - xj(a))^2*(sigma_UWB(a)*dij^2 + 2*sigma_GPS(a)*dij_UWB^2))/(4*dij^4) + theta^2*sigma_GPS(a));
%                     else
%                         T(a,b) = (1-theta)^2/(N-1)*((xi(a) - xj(a))*(xi(b) - xj(b))*(sigma_UWB(a)*dij^2 + 2*sigma_GPS(a)*dij_UWB^2))/(4*dij^4);
%                     end
%                 end
%             end
%             T = diag(T + T');
            
            T1val = DynOpt.ObserverTest.T1(sigma_GPS(1), sigma_UWB(1), dij_UWB, theta, xi(1), xi(2), xi(3), xj(1), xj(2), xj(3));
            T2val = DynOpt.ObserverTest.T2(sigma_GPS(2), sigma_UWB(2), dij_UWB, theta, xi(1), xi(2), xi(3), xj(1), xj(2), xj(3));
            T3val = DynOpt.ObserverTest.T3(sigma_GPS(3), sigma_UWB(3), dij_UWB, theta, xi(1), xi(2), xi(3), xj(1), xj(2), xj(3));
            T = vpa([T1val; T2val; T3val],4);
            
            sigma_p(:,i) = T;
        end
    end   
    sigma_p(:,j_select) = [];
    sigma_p = mean(sigma_p,2);
    opt.sigma_p = sigma_p;
    
    opt.Chi_est = Chi_estimate;    
    
    %%%% test the versors %%%%%
    if 0
        vij = (Chi(j_select,:) - Chi(i,:))/norm(Chi(j_select,:) - Chi(i,:));
        vji = (Chi(i,:) - Chi(j_select,:))/norm(Chi(i,:) - Chi(j_select,:));
        plot3(Chi(j_select,1),Chi(j_select,2),Chi(j_select,3),'b*')
        hold on
        plot3(Chi(j_select,1)+vij(1),Chi(j_select,2)+vij(2),Chi(j_select,3)+vij(3),'r.')
        plot3(Chi(j_select,1)+vji(1),Chi(j_select,2)+vji(2),Chi(j_select,3)+vji(3),'b.')

        if 1
            for i=1:nagent
                if i~=j_select
                plot3(Chi(i,1),Chi(i,2),Chi(i,3),'ro')
                end
            end
            for i=1:nagent
                if i~= j_select
                plot3(Chi_UWB(i,1),Chi_UWB(i,2),Chi_UWB(i,3),'r+')
                end
            end
            plot3(centroid(1),centroid(2),centroid(3),'k*')
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%

end


