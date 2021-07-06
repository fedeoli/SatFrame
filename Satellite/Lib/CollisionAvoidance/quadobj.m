    function [y,grady] = quadobj(x)
        Q = eye(3);
        y = 1/2*x'*Q*x;
        if nargout > 1
            grady = Q*x;
        end
    end
