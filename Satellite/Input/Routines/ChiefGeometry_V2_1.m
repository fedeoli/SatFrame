%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                CHIEF SATELLITE CHARACTERISTICS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Satellites' characteristics
CD_c = 2.2*Drag_on;                                             % chief's drag coefficients
M_c = 2000;                                                     % chief's mass [kg]
w_c = 1*1e-3;                                                   % Side 1 of a Cubesat [km]
l_c = w_c;                                                      % Side 2 of a Cubesat [km]
h_c = 3*1e-3;                                                   % Side 3 of a Cubesat [km]
l_SAR = 2*1e-3;                                                 % Length of SAR [km]
l_SP = 4*1e-3;                                                  % Length of solar panels [km]

% Inertia moments
Ix_c = 1/12*M_c*(w_c^2 + l_c^2);                              % Inertia moment of a cubesat around x axis (firing axis)
Iy_c = 1/12*M_c*(l_c^2 + h_c^2);                              % Inertia moment of a cubesat around y axis
Iz_c = 1/12*M_c*(w_c^2 + h_c^2);                              % Inertia moment of a cubesat around z axis
I_c = [Ix_c, 0, 0;
       0, Iy_c, 0;
       0, 0, Iz_c];

% Parameters allocation
params.sat(1).I = I_c;
params.sat(1).CD = CD_c;
params.sat(1).M = M_c;


%%%%%% Define chief satellite's surfaces %%%%%%

% 1st surface properties
params.sat(1).aero_prop(1).n = [1 0 0];
params.sat(1).aero_prop(1).A = w_c*l_c;
params.sat(1).aero_prop(1).arm = [h_c/2; 0; 0];

% 2nd surface properties
params.sat(1).aero_prop(2).n = [-1 0 0];
params.sat(1).aero_prop(2).A = w_c*l_c;
params.sat(1).aero_prop(2).arm = [-h_c/2; 0; 0];

% 3rd surface properties
params.sat(1).aero_prop(3).n = [0 1 0];
params.sat(1).aero_prop(3).A = w_c*h_c;
params.sat(1).aero_prop(3).arm = [0; l_c/2; 0];

% 4th surface properties
params.sat(1).aero_prop(4).n = [0 -1 0];
params.sat(1).aero_prop(4).A = w_c*h_c;
params.sat(1).aero_prop(4).arm = [0; -l_c/2; 0];

% 5th surface properties
params.sat(1).aero_prop(5).n = [0 0 1];
params.sat(1).aero_prop(5).A = l_c*h_c;
params.sat(1).aero_prop(5).arm = [0; 0; w_c/2];

% 6th surface properties
params.sat(1).aero_prop(6).n = [0 0 -1];
params.sat(1).aero_prop(6).A = l_c*h_c;
params.sat(1).aero_prop(6).arm = [0; 0; -w_c/2];

% 7th surface properties - SAR1 (1st surface)
params.sat(1).aero_prop(7).n = [1 0 0];
params.sat(1).aero_prop(7).A = l_c*l_SAR;
params.sat(1).aero_prop(7).arm = [0; w_c/2 + h_c/2; 0];

% 8th surface properties - SAR1 (2nd surface)
params.sat(1).aero_prop(8).n = [-1 0 0];
params.sat(1).aero_prop(8).A = l_c*l_SAR;
params.sat(1).aero_prop(8).arm = [0; w_c/2 + h_c/2; 0];

% 9th surface properties - SAR2 (1st surface)
params.sat(1).aero_prop(9).n = [1 0 0];
params.sat(1).aero_prop(9).A = l_c*l_SAR;
params.sat(1).aero_prop(9).arm = [0; -(w_c/2 + h_c/2); 0];

% 10th surface properties - SAR2 (2nd surface)
params.sat(1).aero_prop(10).n = [-1 0 0];
params.sat(1).aero_prop(10).A = l_c*l_SAR;
params.sat(1).aero_prop(10).arm = [0; -(w_c/2 + h_c/2); 0];

% 11th surface properties - SolarPanel 1 (1st surface)
params.sat(1).aero_prop(11).n = [0 1 0];
params.sat(1).aero_prop(11).A = l_SP*l_c;
params.sat(1).aero_prop(11).arm = [0; 0; l_c/2 + l_SP/2];


% 12th surface properties - SolarPanel 1 (2nd surface)
params.sat(1).aero_prop(12).n = [0 -1 0];
params.sat(1).aero_prop(12).A = l_SP*l_c;
params.sat(1).aero_prop(12).arm = [0; 0; l_c/2 + l_SP/2];


% 13th surface properties - SolarPanel 2 (1st surface)
params.sat(1).aero_prop(13).n = [0 1 0];
params.sat(1).aero_prop(13).A = l_SP*l_c;
params.sat(1).aero_prop(13).arm = [0; 0; -(l_c/2 + l_SP/2)];

% 14th surface properties - SolarPanel 2 (2nd surface)
params.sat(1).aero_prop(14).n = [0 -1 0];
params.sat(1).aero_prop(14).A = l_SP*l_c;
params.sat(1).aero_prop(14).arm = [0; 0; -(l_c/2 + l_SP/2)];

% Mean Cross-section
params.sat(1).A = 0;

for j = 1:size(params.sat(1).aero_prop, 2)
    
    params.sat(1).A = params.sat(1).A + params.sat(1).aero_prop(j).A;
    
end