function [R_Hill2Cam, phi, psi]  = RHill2Cam(rho_Hill)

vect_b1 = rho_Hill;

if size(vect_b1,1) > size(vect_b1,2)
    
    vect_b1 = vect_b1';
    
end

vers_b1 = vect_b1/sqrt(rho_Hill(1)^2 + rho_Hill(2)^2 + rho_Hill(3)^2);
phi = asin(vers_b1(3));
psi = atan2(vers_b1(2), vers_b1(1));
if psi <= 0
    
    psi = psi + 2*pi;
    
end

theta = 0;

% ZXY rotation
angle1 = psi - pi/2;
angle2 = phi + pi/2;
angle3 = theta;
%R_Hill2Cam = angle2dcm(angle1, angle2, angle3, 'ZXY');
R_Hill2Cam = [cos(angle3)*cos(angle1) - sin(angle2)*sin(angle3)*sin(angle1),...
              cos(angle3)*sin(angle1) + sin(angle2)*sin(angle3)*cos(angle1),...
              -sin(angle3)*cos(angle2);
              -cos(angle2)*sin(angle1),...
              cos(angle2)*cos(angle1),...
              sin(angle2);
              sin(angle3)*cos(angle1) + sin(angle2)*cos(angle3)*sin(angle1),...
              sin(angle3)*sin(angle1) - sin(angle2)*cos(angle3)*cos(angle1),...
              cos(angle2)*cos(angle3)];

end