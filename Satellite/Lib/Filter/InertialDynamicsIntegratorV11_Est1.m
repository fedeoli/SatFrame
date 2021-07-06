function dx = InertialDynamicsIntegratorV11_Est1(temp, params)
global  N_deputy CSIcompleto

satellites_iner_ECI = temp; %CSIcompleto(1:6*(N_deputy+1));  
added_states = CSIcompleto(6*(N_deputy+1)+1:end);

%   InertialDynamicsIntegrator_V1_1.m
%   Made by Sapienza Gn Lab
%
%   Integrates the inertial dynamics equation of N satellites. It includes Drag and J2 perturbations calculations. Control is applied to each deputy, i.e., not 
%   on the first satellite, which is the chief. This function has to be used inside an already built integration method (e.g. ode45 or a home-made rk4 
%   fixed-step method).
%
%   INPUT
%   satellites_iner_ECI: Array (6*N x 1), where N is the number of satellites. It contains satellites' inertial position and velocity.
%   params: Structure containing the following fields
%       - params.mi: Earth's planetary constant
%       - params.Re: Earth's equatorial mean radius
%       - params.J2: Earth's J2 coefficient
%       - params.omega_e: Earth's rotational velocity
%       - params.CDAF: satellites' balistic coefficients
%       - params.u: control matrix (3 x N). j-th column contains the control vector applied to j-th deputy.
%
%   OUTPUT
%   dx: Array (6*N x 1) containing the derivatives of the state. 


global  fh_c r_c h_c fd_c 

% Extraction of constants from "params" structure
mi = params.mi;             % Earth's planetary constant
Re = params.Re;             % Earth's equatorial mean radius
J2 = params.J2;             % Earth's J2 coefficient
omega_e = params.omega_e;   % Earth's rotational velocity
CDAF = params.CDAF;         % Satellite's balistic coefficients
u_LVLH = params.u;          % Control matrix


% Redefinition of control matrix, including a zero control for the chief satellite
u_LVLH = [zeros(3,1), u_LVLH];

% Arrays initialization
n_sat = N_deputy +1;
dx = zeros(6*n_sat, 1);
fn_ECI = zeros(3, n_sat);
p = zeros(n_sat, 1);
r = zeros(n_sat, 1);
th = zeros(n_sat, 1);
f2r = zeros(n_sat, 1);
f2th = zeros(n_sat, 1);
f2h = zeros(n_sat, 1);
fj2_ECI = zeros(3, n_sat);
pos = zeros(3, n_sat);
vel = zeros(3, n_sat);
v_rel = zeros(3, n_sat);
v_rel_mod = zeros(n_sat, 1);
rho = zeros(n_sat, 1);
D = zeros(n_sat, 1);
fd_ECI = zeros(3, n_sat);
u_ECI = zeros(3, n_sat);

% Loop over the number of satellites
for i = 1:n_sat
    
    % Transformation form inertial position and velocity to classical orbital elements
    coe(6*(i-1)+1:i*6) = rv2coe_V1_1(satellites_iner_ECI(6*(i-1)+1:6*(i-1)+3), satellites_iner_ECI(6*(i-1)+4:6*(i-1)+6), mi);

    % Newton's gravitational force computation
    fn_ECI(:,i) = LVLH2ECI_V1_1([-mi/norm(satellites_iner_ECI( 6*(i-1) + 1 : 6*(i-1) + 3 ))^2,0,0], coe(6*(i-1) + 3), coe(6*(i-1) + 5), coe(6*(i-1) + 4) + coe(6*(i-1) + 6));

    % J2 perturbation computation
    p(i) = coe(6*(i-1)+1)*(1-coe(6*(i-1)+2)^2);                                                                                     % semilatum rectum of the i-th satellite
    r(i) = p(i)/(1 + coe(6*(i-1)+2)*cos(coe(6*(i-1)+6)));                                                                           % radius of the i-th satellite
    th(i) = coe(6*(i-1)+6) + coe(6*(i-1)+4);                                                                                        % argument of the latitude of the i-th satellite
    f2r(i) = -3/2*J2*mi*Re^2/(r(i)^4)*(1-(3*(sin(coe(6*(i-1)+3)))^2*(sin(th(i)))^2));                                               % radial component of J2 perturbation acting on i-th satellite
    f2th(i) = -3/2*J2*mi*Re^2/(r(i)^4)*(sin(coe(6*(i-1)+3)))^2*sin(2*th(i));                                                        % tangential component of J2 perturbation acting on i-th satellite
    f2h(i) = -3*J2*mi*Re^2/(r(i)^4)*sin(coe(6*(i-1)+3))*cos(coe(6*(i-1)+3))*sin(th(i));                                             % out-of-plane component of J2 perturbation acting on i-th satellite
    fj2_ECI(:,i) = LVLH2ECI_V1_1([f2r(i), f2th(i), f2h(i)], coe(6*(i-1)+3), coe(6*(i-1)+5), coe(6*(i-1)+4) + coe(6*(i-1)+6))';      % J2 perturbation acting on i-th satellite (expressed in ECI)

    % Drag perturbation computation
    pos(:,i) = satellites_iner_ECI(6*(i-1) + 1 : 6*(i-1) + 3);
    vel(:,i) = satellites_iner_ECI(6*(i-1) + 4 : 6*(i-1) + 6);
    v_rel(:,i) = [vel(1,i) + pos(2,i)*omega_e; vel(2,i) - pos(1,i)*omega_e; vel(3,i)];                                              % satellite's relative velocity wrt Earth's rotation
    v_rel_mod(i) = norm(v_rel(:,i));                                                                                                % module of the relative velocity
    rho(i) = ExponentialAtmDensity_V1_1(r(i) - Re);                                                                                 % atmospheric density at the altitude of i-th satellite
    D(i) = -(1/2)*CDAF(i)*rho(i)*v_rel_mod(i)^2;                                                                                    % drag acceleration module
    fd_ECI(1,i) = D(i)*v_rel(1,i)/v_rel_mod(i);
    fd_ECI(2,i) = D(i)*v_rel(2,i)/v_rel_mod(i);
    fd_ECI(3,i) = D(i)*v_rel(3,1)/v_rel_mod(i);
    
    % Control Computation
    u_ECI(:,i) = LVLH2ECI_V1_1(u_LVLH(:,i), coe(3), coe(5), coe(4) + coe(6));                                                       % Rotation of control vector from LVLH (chief) to ECI
    
    %%DD
    % State derivatives computation  - Absolute dynamics ECI
    dx(6*(i-1) + 1) = satellites_iner_ECI(6*(i-1) + 4);
    dx(6*(i-1) + 2) = satellites_iner_ECI(6*(i-1) + 5);
    dx(6*(i-1) + 3) = satellites_iner_ECI(6*(i-1) + 6);
    dx(6*(i-1) + 4) = fn_ECI(1,i) + fd_ECI(1,i) + fj2_ECI(1,i) + u_ECI(1,i) + added_states(3*(i-1)+1);
    dx(6*(i-1) + 5) = fn_ECI(2,i) + fd_ECI(2,i) + fj2_ECI(2,i) + u_ECI(2,i) + added_states(3*(i-1)+2);
    dx(6*(i-1) + 6) = fn_ECI(3,i) + fd_ECI(3,i) + fj2_ECI(3,i) + u_ECI(3,i) + added_states(3*(i-1)+3);
end


r_c = p(1)/(1 + coe(2)*cos(coe(6)));
h_c = sqrt(mi*p(1));
fd_c = fd_ECI(:,1);
TotalPerturbation_ECI = [(fj2_ECI(1,1) + fd_c(1)), (fj2_ECI(2,1) + fd_c(2)), (fj2_ECI(3,1) + fd_c(3))];                             % Sum of perturbation expressed in ECI reference frame
TotalPerturbation_LVLH = ECI2LVLH_V1_1(TotalPerturbation_ECI, coe(3), coe(5), (coe(4) + coe(6)));                                   % Sum of perturbations expressed in LVLH reference frame
fh_c = TotalPerturbation_LVLH(3);


end