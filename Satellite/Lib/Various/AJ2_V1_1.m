function A = AJ2_V1_1(t, chief_OOE, params)

%   AJ2_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Computes the system's state matrix by taking into account the linearized effects caused by J2 perturbation.
%
%   INPUT
%   t: time instant at which the system state matrix has to be calculated [s]
%   chief_OOE: array containing chief's osculating orbital elements
%   params: structure containing the following fields
%       - params.mi = Earth's planetary constant
%       - params.Re = Earth's mean radius
%       - params.J2 = Earth's J2 coefficient
%
%   OUTPUT
%   A: J2 linearized system's state matrix
%
%   VERSION
%   20190326 V1_1:
%   - First release


% Extraction of needed contants from the "params" structure
mi = params.mi;
Re = params.Re;
J2 = params.J2;

n = sqrt(mi/chief_OOE(1)^3)*sqrt(1+(3/8)*J2/chief_OOE(1)^2*Re^2*(1+3*cos(2*chief_OOE(3))));
th = n*t + chief_OOE(4)+chief_OOE(6);
acca0 = sqrt(mi*chief_OOE(1));

k1 = 3*J2*Re^2/(2*chief_OOE(1));
k2 = .75*mi*J2*Re^2/(chief_OOE(1)*acca0);
k3 = -1.5*J2*mi*Re^2;

r = chief_OOE(1)+k1*(1/3*sin(chief_OOE(3))^2*cos(th)^2+(1/3*sin(chief_OOE(3))^2-1)+(1-(2/3)*sin(chief_OOE(3))^2)*cos(th));
k4 = 1.5*J2*mi*Re^2/r^5;
acca = acca0+k2*sin(chief_OOE(3))^2*(cos(2*th)-1);
accap = k2*sin(chief_OOE(3))^2*(-2*n*sin(2*th));
rp = k1*(-(1/3)*n*sin(chief_OOE(3))^2*sin(2*th)-n*(1-(2/3)*sin(chief_OOE(3))^2)*sin(th));
nt = sqrt(mi/(r^3));

omega(1) = k3*sin(2*chief_OOE(3))*sin(th)/(acca*r^3);
omega(2) = 0;
omega(3) = acca/r^2;

omegap(1) = k3*sin(2*chief_OOE(3))/(acca^2*r^6)*(n*cos(th)*acca*r^3-sin(th)*(accap*r^3+3*acca*r^2*rp));
omegap(2) = 0;
omegap(3) = (accap*r-2*acca*rp)/r^3;

A1 = -(-omega(3)^2-2*nt^2-4*k4*(1-3*sin(chief_OOE(3))^2*sin(th)^2));
B1 = -(-omegap(3)-4*k4*sin(chief_OOE(3))^2*sin(2*th));
C1 = -(omega(1)*omega(3)-4*k4*sin(2*chief_OOE(3))*sin(th));
D1 = -(omegap(3)-4*k4*sin(chief_OOE(3))^2*sin(2*th));
E1 = -(nt^2-omega(1)^2-omega(3)^2-4*k4*(-0.25+sin(chief_OOE(3))^2*(1.75*sin(th)^2-0.5)));
F1 = -(-omegap(1)+k4*sin(2*chief_OOE(3))*cos(th));
G1 = C1;
H1 = -(omegap(1)+k4*sin(2*chief_OOE(3))*cos(th));
I1 = -(nt^2-omega(1)^2-4*k4*(-0.75+sin(chief_OOE(3))^2*(1.25*sin(th)^2+0.5)));

% J2 linearized state matrix
A = [0,   0,   0,    1,             0,            0;
     0,   0,   0,    0,             1,            0;
     0,   0,   0,    0,             0,            1;
     A1,  B1,  C1,   0,             2*omega(3),   0;
     D1,  E1,  F1,   -2*omega(3),   0,            2*omega(1);
     G1,  H1,  I1,   0,             -2*omega(1),  0];

end
