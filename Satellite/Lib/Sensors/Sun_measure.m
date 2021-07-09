%% function        
function [hss_I, hss_body] = Sun_measure(DynOpt,params,x,pos,k)


        % get state
        quat = x(1:4);
        omega = x(5:7);
        
        % rotation matrix ECI to body
        R_ECI2Body = quat2dcm(transpose(q_ECI2Body));

        % convert sun vector from ECI to body
        SunTemp = (R_ECI2Body)*transpose(DynOpt.ObserverTest.Si);

        % from scalar product get angle between sun vector and faces
        theta_x = atan2(norm(cross(SunTemp,[1 0 0])),dot(SunTemp,[1 0 0]));
        theta_y = atan2(norm(cross(SunTemp,[0 1 0])),dot(SunTemp,[0 1 0]));
        theta_z = atan2(norm(cross(SunTemp,[0 0 1])),dot(SunTemp,[0 0 1]));
        theta_mx = atan2(norm(cross(SunTemp,[-1 0 0])),dot(SunTemp,[-1 0 0]));
        theta_my = atan2(norm(cross(SunTemp,[0 -1 0])),dot(SunTemp,[0 -1 0]));
        theta_mz = atan2(norm(cross(SunTemp,[0 0 -1])),dot(SunTemp,[0 0 -1]));

        % alpha_i in eq. 3.2 Bhanderi
        theta = [theta_x, theta_y, theta_z, theta_mx, theta_my, theta_mz];
        
        %%%%%%%%%%%%%%%%%% Accounting ALBEDO %%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % to ECEF in Km;
        sat_ecef = params.dcmecef*pos*1E03;
        sun_ecef = params.dcmecef*transpose(params.Si)*params.CONST.ua;
        
        % load I0 from init
        I0 = params.I0;
        
        % albedo model computation: 
        % 1) a_tot = total albedo Eradiance (ref 4.11)
        % 2) phi_sat = sat inclination wr2 grid 
        [a_tot, phi_sat] = albedo(sat_ecef,sun_ecef,refl); 
         
        %%% Final Albedo Model %%%
        % init section
        I = zeros(1,6);
        I_a = cos(phi_sat);
        I_true = zeros(1,6);
        
        % 
        for i = 1:numel(theta)
            for j = 1:numel(I_a)
             if theta(i) > pi/2
                if (a_tot > 0) 
                    % Albedo measurements
                    I(i) = I0*a_tot*I_a(j); 
                    
                    % real case, no albedo considered
                    I_true(i) = 0;
                    break
                else
                    I(i) = 0;   % ECLIPSE (???)
                end    
             else   
                % Sun measurements (ref 4.13 - 4.14)
                I(i) = I0*cos(theta(i)); 
                I_true(i) = I0*cos(theta(i));
             end
            end
        end
        
        % measurements with and without albedo 
        Idiff = [I(1)-I(4), I(2)-I(5), (I(3)-I(6))];
        Idiff_true = [I_true(1)-I_true(4), I_true(2)-I_true(5), (I_true(3)-I_true(6))];
    
 
        % final observation (ref 4.15)
        S_hat = 1/I0*transpose(Idiff); % SunVector_hat
        S_true = 1/I0*transpose(Idiff_true);
        
        hss_body = S_hat;
        
        hss_I = R_ECI2Body'*hss_body;
%         hss_I = hss_body;

%         Agent(k).hss_I = hss_I;
end