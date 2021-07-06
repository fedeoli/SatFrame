function vect_LVLH = ECI2LVLH_V1_1(vect_ECI, i, raan, th)

%   ECI2LVLH_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Rotates the components of a vector from ECI reference frame to LVLH reference frame.
%
%   INPUT
%   vect_ECI: Vector expressed in ECI reference frame. Can be either (1 x 3) or (3 x 1).
%   i: Inclination [rad]
%   raan: Right Ascension of the Ascending Node [rad]
%   th: Argument of the Latitude = Argument of the Perigee + True Anomaly [rad]
%
%   OUTPUT
%   vect_LVLH: Vector (3 x 1) rotated into LVLH reference frame.
%
%   VERSION
%   20190301 V1_1:
%   -  First Release

% define C variable, for C coder
C = zeros(3);

% Computation of the rotation matrix
C(1,1) = cos(raan)*cos(th) - sin(raan)*cos(i)*sin(th);
C(2,1) = -cos(raan)*sin(th) - sin(raan)*cos(i)*cos(th);
C(3,1) = sin(raan)*sin(i);
C(1,2) = sin(raan)*cos(th) + cos(raan)*cos(i)*sin(th);
C(2,2) = -sin(raan)*sin(th) + cos(raan)*cos(i)*cos(th);
C(3,2) = -cos(raan)*sin(i);
C(1,3) = sin(i)*sin(th);
C(2,3) = sin(i)*cos(th);
C(3,3) = cos(i);

if size(vect_ECI,2) > size(vect_ECI,1)
    
    vect_ECI = vect_ECI';
    
end

vect_LVLH = C*vect_ECI;


end