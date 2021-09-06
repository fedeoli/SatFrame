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
syms N

% centroid definition
syms C1 C2 C3

% theta definition
syms theta

%% step 1
xi = [xi_1; xi_2; xi_3];
xj = [xj_1; xj_2; xj_3];

state = [xj; xi];

Delta = xj - xi;
f = Delta;

S1 = eye(6)*GPS;

J = sym(zeros(length(f),length(state)));
for i=1:length(state)
    J(:,i) = sym(diff(f,state(i)));
end

S2 = J*S1*transpose(J);

%% step 2
D = [D1; D2; D3];
assume(D,'positive');

f = norm(Delta);
f = subs(f,Delta,D);

state = D;

J = sym(zeros(length(f),length(state)));
for i=1:length(state)
    J(:,i) = diff(f,state(i));
end

S3 = J*S2*transpose(J);
S3 = simplify(S3);
S3 = simplifyFraction(S3);

%% step 3
f = xj + (xi-xj)/dij*(dij-dij_RD)/2;

state = [xj; xi; dij; dij_RD];

J = sym(zeros(length(f),length(state)));
for i=1:length(state)
    J(:,i) = diff(f,state(i));
end
J = simplify(J);

S3_stack = sym(zeros(length(state)));
S3_stack(1:6,1:6) = S1;
S3_stack(7,7) = S3;
S3_stack(8,8) = RD;

S4 = J*S3_stack*transpose(J);
S4 = simplify(S4);
S4 = simplifyFraction(S4);

%% step 4
S5 = S4/(N-1);

%% step 5
C = [C1; C2; C3];
f = (1-theta)*C + theta*xj;

state = [C; xj];

J = sym(zeros(length(f),length(state)));
for i=1:length(state)
    J(:,i) = diff(f,state(i));
end
J = simplify(J);

S5_stack = sym(zeros(length(state)));
S5_stack(1:3,1:3) = S5;
S5_stack(4:6,4:6) = S1(1:3,1:3);

S6 = J*S5_stack*transpose(J);
S6 = simplify(S6);
S6 = simplifyFraction(S6);

