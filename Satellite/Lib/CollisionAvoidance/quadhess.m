function hess = quadhess(x,lambda,PHI)
% hess = eye(3) + lambda.eqnonlin*PHI;
hess = eye(3) + lambda.ineqnonlin*PHI;
