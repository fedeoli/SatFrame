function vect_ECI = LVLH2ECI_V1_1(vect_LVLH, incl, raan, th)

%   LVLH2ECI_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Rotates the components of a vector from LVLH reference frame to ECI reference frame.
%
%   INPUT
%   vect_LVLH: Vector expressed in LVLH reference frame. Can be either (1 x 3) or (3 x 1).
%   i: Inclination [rad]
%   raan: Right Ascension of the Ascending Node [rad]
%   th: Argument of the Latitude = Argument of the Perigee + True Anomaly [rad]
%
%   OUTPUT
%   vect_ECI: Vector (3 x 1) rotated into ECI reference frame.
%
%   VERSION
%   20190301 V1_1:
%   -  First Release

% define for C code
C = zeros(3);

% Computation of the rotation matrix
C(1,1) = cos(raan)*cos(th) - sin(raan)*cos(incl)*sin(th);
C(1,2) = -cos(raan)*sin(th) - sin(raan)*cos(incl)*cos(th);
C(1,3) = sin(raan)*sin(incl);
C(2,1) = sin(raan)*cos(th) + cos(raan)*cos(incl)*sin(th);
C(2,2) = -sin(raan)*sin(th) + cos(raan)*cos(incl)*cos(th);
C(2,3) = -cos(raan)*sin(incl);
C(3,1) = sin(incl)*sin(th);
C(3,2) = sin(incl)*cos(th);
C(3,3) = cos(incl);

if size(vect_LVLH,2) > size(vect_LVLH,1)
    
    vect_LVLH = vect_LVLH';
    
end

vect_ECI = C*vect_LVLH;

end