%% graph test

% adjacenci matrix
N = 10;
tmp = (randi(2,N) -1) .* (ones(N)-eye(N));
A = cast((tmp + transpose(tmp))/2,'logical');

% define graph
G = graph(A);