%% sigma points
function DynOpt = sigmaPoints_sat(x_m,P_x,params,DynOpt)

    % Modified by Daniele Carnevale (24.01.2020): added Sigma saturation
    % Rif.:  Julier, Uhlmann: ?A New Extension of the Kalman Filter to Nonlinear 
    % Systems,? SPIE AeroSense Symposium, April 21?24, 1997. Orlando, FL, SPIE
    % e articolo con Manuel Cugliari
    n = size(P_x,1);
    n_state = numel(x_m);

    x_m = [x_m; zeros(n-n_state,1)];
    cappa = 0;

    % init and preallocate
    CSI=zeros(n,2*n+1);
    W = zeros(2*n+1,1);

    %%% sigma points
    deltaCSI=zeros(n,2*n+1);
    CSI(:,2*n+1) = x_m;
    W(2*n+1) = cappa/(cappa+n);


    try
        A = chol(P_x);
    catch
        disp('Problemi con la fattorizzazione di Choleskii della P')
        P_x = (P_x + P_x')/4.0; %1E-3*eye(size(P_x))
        A = chol(P_x);
        %eig(A)
    end

    % see Challa
    for ind=1:n

        % sigma points 
        CSI(:,ind) = x_m + sqrt(n+cappa)*A(ind,:)';
        CSI(:,ind+n) = x_m - sqrt(n+cappa)*A(ind,:)';
        coe(1:6) = rv2coe_V1_1(CSI(1:3,ind), CSI(4:6,ind), params.mi);

        % semilatum rectum of the i-th satellite
        p = coe(1)*(1-coe(2)^2);         
        % radius of the i-th satellite
        r = p/(1 + coe(2)*cos(coe(6)));  

        reset_counter = 0;

        while(r < 1.03*params.Re  && reset_counter<100 ) 

            A(ind,:)  = 0.9*A(ind,:);
            CSI(:,ind) = x_m + sqrt(n+cappa)*A(ind,:)';
            CSI(:,ind+n) = x_m - sqrt(n+cappa)*A(ind,:)';

            % semilatum rectum of the i-th satellite
            p = coe(1)*(1-coe(2)^2);         
            % radius of the i-th satellite
            r = p/(1 + coe(2)*cos(coe(6)));  

            reset_counter = reset_counter + 1;
        end

        W(ind) = 1/(2*(n+cappa));
        W(ind+n) = 1/(2*(n+cappa));
    end

    % error handling
    if abs(sum(W)-1)>0.1
        disp('errore pesi: stoppa u programmone');
        %pause;
    end

    if sum(abs(CSI*W - x_m))>0.2
        disp('errore media: stoppa u programmone');
        %pause;
    end

    % weights 
    for ind=1:n
        deltaCSI(:,ind) = CSI(:,ind) - x_m;
        deltaCSI(:,ind+n) = CSI(:,ind+n) - x_m;    
    end

    % covariance
    COV = zeros(n);
    for ind=1:2*n
        COV = COV + W(ind)*deltaCSI(:,ind)*deltaCSI(:,ind)';
    end

    % error handling
    if sum(abs(COV-P_x))>0.001
        disp('errore covarianza: stoppa u programmone');
        %pause;
    end

    Psat = A*A';

    DynOpt.ObserverTest.W = W;
    DynOpt.ObserverTest.CSI = CSI;
    DynOpt.ObserverTest.Psat = Psat;

end