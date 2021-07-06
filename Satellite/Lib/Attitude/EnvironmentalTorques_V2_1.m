function [tau_GG, tau_D] = EnvironmentalTorques_V2_1(SatellitesAttitude, params)

%   EnvironmentalTorques_V2_1.m
%   Made by Sapienza Gn Lab
%
%   Computes the gravity gradient and atmospheric drag torques acting on every satellite at a given time instant.
%
%   INPUT
%   SatellitesAttitude: array (7*(N+1) x 1) containing the quaternions and angular velocities of each satellite at a ginve time instant. [km] and
%                       [km/s].
%   params: structure containing the following fields:
%       - params.mi = Earth's planetary constant [km^3*s^-2]
%       - params.sat = ((N+1) x 1) structure containing the following fields:
%           - sat(i).I = i-th satellite's inertia matrix [kg*km^2];
%           - sat(i).CD = i-th satellite's drag coefficient;
%           - sat(i).aero_prop = (K x 1) structure containing the following fields (K is the number of surfaces of which i-th satellite is made of):
%               - aero_prop(j).A = j-th surface's area [km^2];
%               - aero_prop(j).n = j-th surface's normal;
%               - aero_prop(j).arm = distance between j-th surface's center of pressure and i-th satellite's center of mass [km].
%       - params.SatellitesCoordinates = array (6*(N+1) x 1) containing each satellite's inertial position and velocity at the given time instant. N
%                                        is the number of deputies.
%       - params.omegae = Earth's spinning rate [rad/s]
%       - params.Re = Earth's mean radius [km]
%
%   OUTPUT
%   tau_GG: matrix (3 x (N+1)) containing the gravity gradient torque acting on every satellite. N is the number of deputies. [N*m]
%   tau_D: matrix (3 x (N+1)) containing the atmospheric drag torque acting on every satellite. N is the number of deputies. [N*m]
%
%   VERSION
%
%   20200110 V2_1:
%   - First release


n_sat = length(SatellitesAttitude)/7;
mie = params.mi;

tau_D = zeros(3, n_sat);
tau_GG = tau_D;

for i = 1:n_sat
    
    Att_sat = SatellitesAttitude(7*(i-1) + 1 : i*7);
    q_ECI2Body = Att_sat(1:4);
    I = params.sat(i).I;
    
    X_sat = params.SatellitesCoordinates(6*(i-1) + 1 : 6*i);
    vect_r = X_sat(1:3);
    vect_v = X_sat(4:6);
    
    % Gravity gradient toqrue calculation
    r = norm(vect_r);
    R_ECI2Body = quat2dcm(q_ECI2Body');
    vers_o_Body = -R_ECI2Body*(vect_r/r);
    Io = I*vers_o_Body;
    cross_oIo = [vers_o_Body(2)*Io(3) - vers_o_Body(3)*Io(2);...
        vers_o_Body(3)*Io(1) - vers_o_Body(1)*Io(3);...
        vers_o_Body(1)*Io(2) - vers_o_Body(2)*Io(1)];
    tau_GG(:,i) = (3*mie/(r^3))*cross_oIo*1e6;
    
    % Aerodynamic drag torque calculation
    v_rel = [vect_v(1) + vect_r(2)*params.omega_e; vect_v(2) - vect_r(1)*params.omega_e; vect_v(3)];            % satellite's relative velocity wrt Earth's rotation
    v_rel_mod = norm(v_rel);                                                                                    % module of the relative velocity
    v_rel_body = R_ECI2Body*v_rel;                                                                              % relative velocity expressed in BRF
    rho = ExponentialAtmDensity_V1_1(norm(vect_r) - params.Re);                                                 % atmospheric density at the altitude of i-th satellite

    CD = params.sat(i).CD;
    surf_num = length(params.sat(i).aero_prop);

    for j = 1:surf_num
        
        Area = params.sat(i).aero_prop(j).A;
        Surf_normal = params.sat(i).aero_prop(j).n;
        arm = params.sat(i).aero_prop(j).arm;
        cross_section = max(Area*dot(v_rel_body,Surf_normal),0);
        
        F_drag = -(1/2)*rho*CD*v_rel_mod*cross_section*v_rel_body;
        T_drag = cross(arm, F_drag);
        tau_D(:,i) = tau_D(:,i) + T_drag*1e6;
        
    end
    
end

end

