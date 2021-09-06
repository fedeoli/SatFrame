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
function opt = Position_opt_cloud_num_v10_dec(Chi, GPS, adjmat_UWB, j_select, theta, beta, check_dist, packet_UWB, RecoveryPos, DynOpt)
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
   
    %%% SIGMA ANALYSIS %%%            
    %%% used vals %%%
    % MEAS
    dij_UWB = adjmat_UWB(j_select,:);
    dij_UWB(j_select) = [];

    xi = Chi;
    xi(j_select,:) = [];
    xj = Chi(j_select,:);

    %%% SIGMA %%%
    sigma_GPS = DynOpt.ObserverTest.GPSGaussianCovariance(1:3).^2;
    sigma_UWB = ones(1,3).*DynOpt.ObserverTest.ErrorAmplitudeUWB.^2;
    
    % lana caprina - xi + +xj + dij_RD
    %%% XI
    xi_line = reshape(xi,(nagent-1)*3,1);
    xi_string = '';
    for i=1:length(xi_line)
        xi_name = strcat(num2str(xi(i)),', ');
        xi_string = strcat(xi_string,xi_name);
    end
    xj_line = reshape(xj,3,1);
    %%% XJ
    xj_string = '';
    for i=1:length(xj_line)
        xj_name = strcat(num2str(xj(i)),', ');
        xj_string = strcat(xj_string,xj_name);
    end
    xj_string(end) = [];
    %%% DIJ
    dij_line = reshape(dij_UWB,nagent-1,1);
    dij_string = '';
    for i=1:length(dij_line)
        dij_name = strcat(num2str(dij_UWB(i)),', ');
        dij_string = strcat(dij_string,dij_name);
    end
    
    %%% functions
    fun = strcat('DynOpt.ObserverTest.T1_',num2str(nagent));
    command = strcat(fun,'(',num2str(sigma_GPS(1)),',',num2str(sigma_UWB(1)),',',dij_string,num2str(theta),',',xi_string,xj_string,');');
    T1val =  eval(command);
    command = strcat('DynOpt.ObserverTest.T2(',num2str(sigma_GPS(2)),',',num2str(sigma_UWB(2)),',',dij_string,num2str(theta),',',xi_string,xj_string,');');
    T2val =  eval(command);
    command = strcat('DynOpt.ObserverTest.T3(',num2str(sigma_GPS(3)),',',num2str(sigma_UWB(3)),',',dij_string,num2str(theta),',',xi_string,xj_string,');');
    T3val =  eval(command);
    T = vpa([T1val; T2val; T3val],4);

    sigma_p = T;
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


