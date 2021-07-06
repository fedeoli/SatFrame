function u_CA = CollisionAvoidancePotential_V1_1(deputy_rel_LVLH, i)

%   CollisionAvoidancePotential_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Computes the Collision Avoidance control effort needed by the i-th satellite to avoid collision with other satellites part of the formation. This
%   control is computed by means of a repulsive potential approach.
%
%   INPUT
%   deputy_rel_LVLH : Array (6 x N), with N equal to the number of deputy satellites, containing each deputy's position and velocity relative components,
%                    expressed in LVLH reference frame.
%   i : Index of the i-th satellite.
%
%   OUTPUT
%   u_CA : Array (3 x 1) containing the Collision Avoidance control on 3-axes for the i-th satellite.
%
%   VERSION
%   20190502 V1_1:
%   -  First Release

% Number of deputy satellites computation
Ndeputy = size(deputy_rel_LVLH, 2);

% Extraction of i-th deputy relative position
r = deputy_rel_LVLH(1:3,i);

% Parameters of the potential bell's shape
Cr = 1e-6; % heigth of the potential bell
Lr = 0.05; % width of the potential bell

% Gradient of the potential initialization ([0; 0; 0] = no correction due to collision avoidance)
grad_CA = [0; 0; 0];

for j = 1:Ndeputy
    
    if j ~= i
        
        % Extraction of j-th deputy relative position
        rj = deputy_rel_LVLH(1:3,j);
        
        % Norm of i-th to j-th relative position
        xjk = norm(rj - r); 
        
        % Gradient of the potential between i-th and j-th satellites
        grad_CA = grad_CA - (Cr/Lr)*((rj-r)/xjk)*exp( - (xjk/Lr) );
      
    end
    
end

% Storage of the Collision Avoidance control needed for the i-th satellite
u_CA = grad_CA';

end