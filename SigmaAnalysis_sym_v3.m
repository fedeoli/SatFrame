%% sigma computation - symbolic

clear all
close all

% agents definition
N = 2;

% sym definition
syms xi [3 N-1]
syms xj [3 1]

% distance definition
syms dij_RD [N-1 1]

% sigma definition
syms GPS RD

% theta definition
syms theta

%% step 1

% updated distances
for n=1:N-1
    xi_tmp = xi(:,n);
    xj_tmp = xj(:,1);
    XI(:,n) = xj_tmp + ((xi_tmp-xj_tmp)/norm(xi_tmp-xj_tmp))*((norm(xi_tmp-xj_tmp)-dij_RD(n))/2);
end

% centroid
C = sym(zeros(3,1));
for n=1:N-1
    C = C + XI(:,n);
end
C = C/(N-1);

% final position
f = (1-theta)*C + theta*xj;

xi_col = reshape(xi,(N-1)*3,1);
state = [xj; xi_col; dij_RD];

J = sym(zeros(length(f),length(state)));
for i=1:length(state)
    J(:,i) = diff(f,state(i));
end
J = simplify(J);

% initial sigma
S1 = eye(N*3)*GPS;
S1_stack = sym(zeros(length(state)));
S1_stack(1:N*3,1:N*3) = S1;
S1_stack(N*3+1:N*3+N-1,N*3+1:N*3+N-1) = RD;

S2 = J*S1_stack*transpose(J);
S2 = simplify(S2);

vars = symvar(S2(1,1));
T1 = symfun(S2(1,1),vars);
vars = symvar(S2(2,2));
T2 = symfun(S2(2,2),vars);
vars = symvar(S2(3,3));
T3 = symfun(S2(3,3),vars);

save('SigmaFun','T1','T2','T3');

