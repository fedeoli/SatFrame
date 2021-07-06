% Earth's constants
mi = 398600.613;                    % Earth's planetary constant [km^3*s^-2]
J2 = J2_on*1.0826e-3;               % Earth's J2 coefficient
Re = 6378.14;                       % Earth's mean equatorial radius [km]
omega_e = 0.729211502e-4;           % Earth's rotational velocity [s^-1]

% "params" structure allocation
params.mi = mi;
params.J2 = J2;
params.Re = Re;
params.omega_e = omega_e;

