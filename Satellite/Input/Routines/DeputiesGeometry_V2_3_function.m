function params = DeputiesGeometry_V2_3_function(params,N_deputy)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                   DEPUTIES CHARACTERISTICS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    CD_d = 2.2*params.Drag_on;                                                     % deputy's drag coefficient
    M_d = 4;                                                                % deputy's mass [kg]
    w_d = params.InertiaVar(1)*0.1*1e-3;                                                         % Side 1 of a Cubesat [km]
    l_d = params.InertiaVar(2)*w_d;                                                              % Side 2 of a Cubesat [km]
    h_d = params.InertiaVar(3)*0.3*1e-3;                                                         % Side 3 of a Cubesat [km]

    % Inertia moments
    Ix = 1/12*M_d*(w_d^2 + l_d^2);                              % Inertia moment of a cubesat around x axis (firing axis)
    Iy = 1/12*M_d*(l_d^2 + h_d^2);                              % Inertia moment of a cubesat around y axis
    Iz = 1/12*M_d*(w_d^2 + h_d^2);                              % Inertia moment of a cubesat around z axis
    I_d = [Ix, 0, 0;
        0, Iy, 0;
        0, 0, Iz];


    %%%%%% Define chaser satellite's surfaces %%%%%%

    for i = 1:N_deputy

        % Parameters allocation
        params.sat(i+1).I = I_d;
        params.sat(i+1).CD = CD_d;
        params.sat(i+1).M = M_d;
        params.sat(i+1).r_HB = sqrt(2)*max([w_d, l_d, h_d]);

        % 1st surface properties
        params.sat(i+1).aero_prop(1).n = [1 0 0];
        params.sat(i+1).aero_prop(1).A = w_d*l_d;
        params.sat(i+1).aero_prop(1).arm = [h_d/2; 0; 0];

        % 2nd surface properties
        params.sat(i+1).aero_prop(2).n = [-1 0 0];
        params.sat(i+1).aero_prop(2).A = w_d*l_d;
        params.sat(i+1).aero_prop(2).arm = [-h_d/2; 0; 0];

        % 3rd surface properties
        params.sat(i+1).aero_prop(3).n = [0 1 0];
        params.sat(i+1).aero_prop(3).A = w_d*h_d;
        params.sat(i+1).aero_prop(3).arm = [0; l_d/2; 0];

        % 4th surface properties
        params.sat(i+1).aero_prop(4).n = [0 -1 0];
        params.sat(i+1).aero_prop(4).A = w_d*h_d;
        params.sat(i+1).aero_prop(4).arm = [0; -l_d/2; 0];

        % 5th surface properties
        params.sat(i+1).aero_prop(5).n = [0 0 1];
        params.sat(i+1).aero_prop(5).A = l_d*h_d;
        params.sat(i+1).aero_prop(5).arm = [0; 0; w_d/2];

        % 6th surface properties
        params.sat(i+1).aero_prop(6).n = [0 0 -1];
        params.sat(i+1).aero_prop(6).A = l_d*h_d;
        params.sat(i+1).aero_prop(6).arm = [0; 0; -w_d/2];

        % Mean Cross-section
        params.sat(i+1).MeanCrossSection = sum(vertcat(params.sat(i+1).aero_prop(:).A))/2;

    end
end


