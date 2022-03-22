%%% sigma computation - symbolic
function J = SigmaAnalysis_sym_v4(N,xi,xj,dij,theta,i_select)
    %% PHI functions
    for j=1:N
        if j~=i_select
            for dim=1:3
               phi_j(j,dim) = -(xi(dim)-xj(j,dim))/(norm(xi-xj(j,:)));
            end
        end
    end
    for j=1:N
        if j~=i_select
            for dim=1:3
               phi_i(j,dim) = (xi(dim)-xj(j,dim))/(norm(xi-xj(j,:)));
            end
        end
    end
    
    %% DFDXi
    % first term (relative distances)
    sum_term = zeros(3,3);
    for j=1:N
       if j~=i_select
           for dimrow=1:3
               for dimcol=1:3
                    if dimrow == dimcol
                        sum_term(dimrow,dimcol) = sum_term(dimrow,dimcol) +0.5*dij(i_select,j)*(norm(xi-xj(j,:)) - (xi(dimrow)-xj(j,dimrow))*phi_i(j,dimcol))/(norm(xi-xj(j,:))^2);
                    else
                        sum_term(dimrow,dimcol) = sum_term(dimrow,dimcol) -0.5*dij(i_select,j)*((xi(dimrow)-xj(j,dimrow))*phi_i(j,dimcol))/(norm(xi-xj(j,:))^2);
%                         sum_term(dimrow,dimcol) = 0;
                    end
               end
           end             
       end
    end
    % actual derivative
    dfdxi = (1-theta)/(N-1)*(eye(3)*(N-1)/2 + sum_term) + theta*eye(3);
    
    %% DFDXj
    for j=1:N
        if j~=i_select
            for dimrow=1:3
                for dimcol=1:3
                    if dimrow == dimcol
                        dfdxj(j,dimrow,dimcol) = 0.5 -0.5*dij(i_select,j)*(norm(xi-xj(j,:)) + (xi(dimrow)-xj(j,dimrow))*phi_j(j,dimcol))/(norm(xi-xj(j,:))^2);
                    else
                        dfdxj(j,dimrow,dimcol) = -0.5*dij(i_select,j)*((xi(dimrow)-xj(j,dimrow))*phi_j(j,dimcol))/(norm(xi-xj(j,:))^2);
%                         dfdxj(j,dimrow,dimcol) = 0;
                    end
                end
            end
        end
    end
    % actual derivative
    dfdxj = (1-theta)/(N-1)*(dfdxj);
    
    %% DFDDij
    for j=1:N
        if j~=i_select
            for dimrow=1:3
                dfddij(j,dimrow) = 0.5*(xi(dimrow)-xj(j,dimrow))/(norm(xi-xj(j,:)));
            end            
        end
    end
    % actual derivative
    dfddij = (1-theta)/(N-1)*(dfddij);
    
    %% build Jacobian
    for dimrow=1:3
        % concatenate dfdxj
        tmp_xj = [];
        tmp_dj = [];
        for j=1:N
            if j~=i_select
                data_xj = reshape(dfdxj(j,dimrow,:),1,3);
                data_dj = dfddij(j,dimrow);
                tmp_xj = [tmp_xj, data_xj];
                tmp_dj = [tmp_dj, data_dj];
            end
        end
        J(dimrow,:) = [dfdxi(dimrow,:), tmp_xj, tmp_dj];
    end

    
end

