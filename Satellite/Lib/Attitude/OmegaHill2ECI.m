function [omega, domega] = OmegaHill2ECI(vect_r, vect_v, mie)

if size(vect_r, 1) ~= 3
    
    vect_r = vect_r';
    
end

if size(vect_v, 1) ~= 3
    
    vect_v = vect_v';
    
end


vect_h = [vect_r(2)*vect_v(3) - vect_r(3)*vect_v(2);
        vect_r(3)*vect_v(1) - vect_r(1)*vect_v(3);
        vect_r(1)*vect_v(2) - vect_r(2)*vect_v(1)];
r = sqrt(vect_r(1)^2 + vect_r(2)^2 + vect_r(3)^2);

h = sqrt(vect_h(1)^2 + vect_h(2)^2 + vect_h(3)^2);

omega = vect_h./(r^2);

%[~, e, ~, ~, ~, ni] = rv2coe_cpp(vect_r, vect_v, mie);
e = 0;
ni = sqrt(mie/r^3);
vr = mie*e*sin(ni)/h;
domega = -2*vr.*vect_h./(r^3);


end