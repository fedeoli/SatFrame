%% sigma computation - symbolic

clear all
close all

% sym definition
syms xi_1 xi_2 xi_3
syms xj_1 xj_2 xj_3
syms D1 D2 D3
syms dij dij_RD

% sigma definition
syms GPS RD

% agents definition
N = 2;

% theta definition
syms theta

%% step 1
xi = [xi_1; xi_2; xi_3];
xj = [xj_1; xj_2; xj_3];

assume((xi-xj) ~= 0);
f = (1-theta)*(xj + (xi-xj)/norm(xi-xj)*(norm(xi-xj)-dij_RD)/2) + theta*xj;

state = [xj; xi; dij_RD];

J = sym(zeros(length(f),length(state)));
for i=1:length(state)
    J(:,i) = diff(f,state(i));
end
J = simplify(J);

% initial sigma
S1 = eye(N*3)*GPS;
S1_stack = sym(zeros(length(state)));
S1_stack(1:N*3,1:N*3) = S1;
S1_stack(7,7) = RD;

S2 = J*S1_stack*transpose(J);
S2 = simplify(S2);

% vars = [xj;xi;dij_RD;GPS;RD;N;theta];
vars = symvar(S2(1,1));
T1 = symfun(S2(1,1),vars);
vars = symvar(S2(2,2));
T2 = symfun(S2(2,2),vars);
vars = symvar(S2(3,3));
T3 = symfun(S2(3,3),vars);

save('SigmaFun','T1','T2','T3');

