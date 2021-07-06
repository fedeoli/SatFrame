function a = module_V2_1(A)

%   module_V2.m
%   Made by Sapienza Gn Lab
%
%   Computes the module of each column of a matrix.
%
%   INPUT
%   A: matrix (M x N)
%
%   OUTPUT
%   a: Array (1 x N) containing the module of each column of A.
%
%   VERSION
%   20191125 V2_1:
%   -  First Release


a = zeros(1,size(A,2));

for i = 1:size(A,2)
    
    a(1,i) = norm(A(:,i));
    
end

end