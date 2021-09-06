function AdjacencyMatrix = setDistMatrix(x,Nagents)
%   setAdjacencyMatrix.m: the adjacency matrix with squared distances
%   x: state vector of all agents positions (positions_1,positions_2,....positions_N-th]) 


    AdjacencyMatrix = zeros(Nagents,Nagents,3);
    for i=1:Nagents
        for j=i+1:Nagents
            AdjacencyMatrix(i,j,:) = x([1:3]+3*(i-1)) - x([1:3]+3*(j-1));
        end
    end
end
