function R = RECI2Hill(vect_r, vect_v)

vect_b1 = vect_r;
vect_b3 = [vect_r(2)*vect_v(3) - vect_r(3)*vect_v(2),...
           vect_r(3)*vect_v(1) - vect_r(1)*vect_v(3),...
           vect_r(1)*vect_v(2) - vect_r(2)*vect_v(1)];
    
if size(vect_b1,1) > size(vect_b1,2)
    
    vect_b1 = vect_b1';
    
end

vers_b1 = vect_b1/sqrt(vect_b1(1)^2 + vect_b1(2)^2 + vect_b1(3)^2);
vers_b3 = vect_b3/sqrt(vect_b3(1)^2 + vect_b3(2)^2 + vect_b3(3)^2);
vers_b2 = [vers_b3(2)*vers_b1(3) - vers_b3(3)*vers_b1(2),...
           vers_b3(3)*vers_b1(1) - vers_b3(1)*vers_b1(3),...
           vers_b3(1)*vers_b1(2) - vers_b3(2)*vers_b1(1)];

R = [vers_b1; vers_b2; vers_b3];

end