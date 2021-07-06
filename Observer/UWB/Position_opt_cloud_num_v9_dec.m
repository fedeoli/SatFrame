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
function opt = Position_opt_cloud_num_v9_dec(Chi, GPS, adjmat_UWB, j_select, theta, beta, check_dist, projection, packet_UWB, RecoveryPos)
%     fprintf("Optimizing - geometric method\n");
    
    % nagent
    nagent = size(Chi,1);
    
    % safe check
    dist_thresh = 5e-2;
    
    % Versor Vij computation - Chi values: using the Chi values the direction
    % between agent i and agent j is computed. Data are stored in a matrix.
    % Index (i,k) defines the kth component of the vector connecting agent i
    % to agent j_select.
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
    version = 2;
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
   
    opt.Chi_est = Chi_estimate;
    % optional 
%     opt.centroid = centroid;
%     opt.Chi_UWB = Chi_UWB;
%     opt.direction_matrix_Chi = direction_matrix_Chi;
end


