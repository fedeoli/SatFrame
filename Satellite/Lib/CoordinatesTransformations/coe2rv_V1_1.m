function rv = coe2rv_V1_1(coe, mi)

%   coe2rv_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Transforms satellite's Classical Orbital Elements into inertial position and velocity (expressed in ECI reference frame).
%
%   INPUT
%   coe: Array (1 x 6) containing satellite's orbital elements.
%   mi: Earth's planetary constant [km^3/s^2]
%
%   OUTPUT
%   rv: Array (6 x 1) containing satellite's inertial position and velocity.
%
%   VERSION
%   20190301 V1_1:
%   -  First Release


% Orbital Elements calculation
a = coe(1);                                                                     % semi-major axis
e = coe(2);                                                                     % eccentricity
incl = coe(3);                                                                  % inclination
argp = coe(4);                                                                  % argument of the perigee
raan = coe(5);                                                                  % right ascension of the ascending node
f = coe(6);                                                                     % true anomaly
th = argp + f;                                                                  % argument of the latitude
p = a*(1-e^2);                                                                  % semilatus rectum
r = p/(e*cos(f)+1);                                                             % module of satellite's position
hr = sqrt(mi*p)/r;                                                              % satellite's orbital angular momentum over radius
vr = hr*e*sin(f)/p;                                                             % radial component of satellite's velocity    

% define for C code
vect_r = zeros(1,3);
vect_v = zeros(1,3);

% Computation of satellite's position
vect_r(1) = r*(cos(th)*cos(raan) - sin(th)*sin(raan)*cos(incl));
vect_r(2) = r*(cos(th)*sin(raan) + sin(th)*cos(raan)*cos(incl));
vect_r(3) = r*(sin(th)*sin(incl));

% Computation of satellite's velocity
vect_v(1) = vr*vect_r(1) - hr*(sin(th)*cos(raan) + cos(th)*sin(raan)*cos(incl));
vect_v(2) = vr*vect_r(2) - hr*(sin(th)*sin(raan) - cos(th)*cos(raan)*cos(incl));
vect_v(3) = vr*vect_r(3) + hr*cos(th)*sin(incl);

% Allocation of the output
rv = [vect_r'; vect_v'];

end