function [R_ECI2Body] = store_dcm(x)
        % get quaternion evolution
        q_true = x(1:4)'; 
        % get angular velocity evolution
        omega_true = x(5:7)';
        
        % quaternion normalization
        R_ECI2Body = quat2dcm(q_true/norm(q_true));
        
end