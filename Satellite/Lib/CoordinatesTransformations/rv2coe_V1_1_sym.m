function coe = rv2coe_V1_1_sym(vect_r, vect_v, mi)

%   rv2coe_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Transforms satellites inertial coordinates (expressed in ECI reference frame) into classical orbital elements.
%
%   INPUT
%   vect_r: Array containing satellite's inertial position expressed in ECI reference frame. Can be either (1 x 3) or (3 x 1).
%   vect_v: Array containing satellite's inertial velocity expressed in ECI reference frame. Can be either (1 x 3) or (3 x 1).
%   mi: Earth's planetary constant [km^3/s^2]
%
%   OUTPUT
%   coe: Array (1 x 6) containing satellite's classical orbital elements
%
%   VERSION
%   20190301 V1_1:
%   -  First Release

% Computation of orbital angular momentum  vect_h = vect_r x vect_v
vect_h(1) = vect_r(2)*vect_v(3) - vect_r(3)*vect_v(2);
vect_h(2) = vect_r(3)*vect_v(1) - vect_r(1)*vect_v(3);
vect_h(3) = vect_r(1)*vect_v(2) - vect_r(2)*vect_v(1);
h = sqrt(vect_h(1)^2 + vect_h(2)^2 + vect_h(3)^2);

% Module of orbital angular momentum on xy plane
h_xy = sqrt(vect_h(1)^2 + vect_h(2)^2);

% Module of satellite's position vector
r = sqrt(vect_r(1)^2 + vect_r(2)^2 + vect_r(3)^2);

% Square of satellite's velocity vector's module
vel_square = (vect_v(1)^2 + vect_v(2)^2 + vect_v(3)^2);

% Scalar product between position and velocity vectors
rv = vect_r(1)*vect_v(1) + vect_r(2)*vect_v(2) + vect_r(3)*vect_v(3);

% Computation of the eccentricity vector
E = vel_square - mi/r;
vect_e(1) = (E*vect_r(1) - rv*vect_v(1))/mi;
vect_e(2) = (E*vect_r(2) - rv*vect_v(2))/mi;
vect_e(3) = (E*vect_r(3) - rv*vect_v(3))/mi;
e = sqrt(vect_e(1)*vect_e(1)+vect_e(2)*vect_e(2)+ vect_e(3)* vect_e(3));
coe(2) = e;

% Semi-major axis
coe(1) = h^2/(mi*(1 - e^2));

% Inclination
cos_incl = vect_h(3)/h;
sin_incl = h_xy/h;
coe(3) = atan2(sin_incl, cos_incl);

% Argument of the perigee
sin_argp = vect_e(3)*h/e/h_xy;
cos_argp =(vect_e(2)*vect_h(1) - vect_e(1)*vect_h(2))/e/h_xy;
coe(4) = atan2(sin_argp, cos_argp);

% Right Ascension of the Ascending Node (RAAN)    
sin_raan = vect_h(1)/h_xy;
cos_raan = -vect_h(2)/h_xy;
coe(5) = atan2(sin_raan, cos_raan);

% True Anomaly    
cos_f = vect_r(1)/r*(-vect_h(2)/h_xy) + vect_r(2)/r*(vect_h(1)/h_xy);
sin_f = vect_r(3)/r*(h/h_xy);

coe(6) = atan2(sin_f,cos_f) - coe(4);

if coe(6) < 0
    
   coe(6) = 2*pi + coe(6);
   
end

end