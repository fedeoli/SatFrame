function [y, yeq, grady, gradyeq] = quadconstr(x,PHI,D)

yeq = [];

y = -x'*PHI*x + D^2;

gradyeq = [];
grady = -2*PHI*x;

end