%% function to analyse the filter dispersion
function sigma_p = SigmaAnalysis(N,theta,sigma_GPS,sigma_UWB,dij,dij_UWB,dist)

    for i= 1:3
        sigma_p(:,:,i) = (1-theta).^2./(N-1)*(sigma_GPS^2*(1 + (dij-dij_UWB)/dij + (dij-dij_UWB)^2/(2*dij^2) + (dij^2*dist(i)^2/(2*dij^4)) + (sigma_UWB^2*dist(i)/(2*dij)))) + theta.^2.*sigma_GPS^2;
    end
    sigma_p = sqrt(mean(sigma_p,3));
%     sigma_p = sqrt((1-theta).^2./(N-1)*(sigma_GPS^2 + sigma_UWB/2) + theta.^2*sigma_GPS^2);

end