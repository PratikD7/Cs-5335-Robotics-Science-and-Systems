% implementation of Dijkstra algorithm.
% Input == V:adjacency matrix, C:costMatrix matrix,
% the algorithm will calculate the minimal path from all N points to the
% configuration number startID
% the algorithm will calculate the minimal path from the configuration number goalID 
% to all N points 
function [paths,costMatrixs] = dijkstra(V,C,startID,goalID,swbar)
    narginchk(2,5);
    [n,adj_count] = size(V);
    [m,cost_count] = size(C);
    if (nargin < 3)
        startID = (1:n);
    elseif isempty(startID)
        startID = (1:n);
    end
    len1 = length(startID);
    len2 = length(goalID);
    if (nargin < 4)
        goalID = (1:n);
    elseif isempty(goalID)
        goalID = (1:n);
    end
    
    if (nargin < 5)
        swbar = (n > 1000 && max(len1,len2) > 1);
    end
    
    % Error check inputs
    if (max(startID) > n || min(startID) < 1)
        eval(['help ' mfilename]);
        error('Invalid [startID] input. See help notes above.');
    end
    if (max(goalID) > n || min(goalID) < 1)
        eval(['help ' mfilename]);
        error('Invalid [goalID] input. See help notes above.');
    end
    [E,costMatrix] = input_processing(V,C);
       
    % Reverse the algorithm if it will be more efficient
    reverse_flag = false;
    if len1 > len2
        E = E(:,[2 1]);
        costMatrix = costMatrix';
        dummy = startID;
        startID = goalID;
        goalID = dummy;
        reverse_flag = true;
    end
    
    
    % Initialize output variables
    len1 = length(startID);
    len2 = length(goalID);
    costMatrixs = zeros(len1,len2);
    paths = num2cell(NaN(len1,len2));
        
    % Create a waitbar if desired
    if swbar
        hWait = waitbar(0,'Calculating Minimum Paths ... ');
    end
    
    % Find the minimum costMatrixs and paths using Dijkstra's Algorithm
    for p = 1:len1
        
        % Initializations
        init_table = NaN(n,1);
        mincostMatrix = Inf(n,1);
        visited = false(n,1);
        graph_path = num2cell(NaN(n,1));
        I = startID(p);
        mincostMatrix(I) = 0;
        init_table(I) = 0;
        visited(I) = true;
        graph_path(I) = {I};
        
        % Execute Dijkstra's Algorithm for this vertex and then
        % Calculate the costMatrixs to the neighbor nodes and record paths
        while any(~visited(goalID))
            % Update the innt table
            j_table = init_table;
            init_table(I) = NaN;
            indexOfNode = find(E(:,1) == I);
                        
            for ii = 1:length(indexOfNode)
                J = E(indexOfNode(ii),2);
                if ~visited(J)
                    c = costMatrix(I,J);
                    empty = isnan(j_table(J));
                    if empty || (j_table(J) > (j_table(I) + c))
                        init_table(J) = j_table(I) + c;
                        if reverse_flag
                            graph_path{J} = [J graph_path{I}];
                        else
                            graph_path{J} = [graph_path{I} J];
                        end
                    else
                        init_table(J) = j_table(J);
                    end
                end
            end
  
            P = find(~isnan(init_table));
            if isempty(P)
                break
            else
                % Settle the minimum value in the table
                [~,N] = min(init_table(P));
                I = P(N);
                mincostMatrix(I) = init_table(I);
                visited(I) = true;
            end
        end
        
        % Store costMatrixs and paths
        costMatrixs(p,:) = mincostMatrix(goalID);
        paths(p,:) = graph_path(goalID);
        if swbar && ~mod(p,ceil(len1/100))
            waitbar(p/len1,hWait);
        end
    end
    if swbar
        delete(hWait);
    end
    
    
    % Reformat outputs if algorithm was reversed
    if reverse_flag
        costMatrixs = costMatrixs';
        paths = paths';
    end
    
    
    % Pass the graph_path as an array if only one source/destination were given
    if len1 == 1 && len2 == 1
        paths = paths{1};
    end
        
    % -------------------------------------------------------------------
    function [E,C] = input_processing(V,C)
        if (n == adj_count)
            if (m == n)
                VertexSet = V;
                VertexSet = VertexSet - diag(diag(VertexSet));
                if (m == cost_count)
                    E = adjToEdgeList(VertexSet);
                else
                    xy = C;
                    E = adjToEdgeList(VertexSet);
                    Dspar = calculateEclusdian(xy,E);
                    C = sparse(E(:,1),E(:,2),Dspar,n,n);
                end
            else
                eval(['help ' mfilename]);
                error('Invalid inputs');
            end
        else
            if (cost_count == 2)
                E = C;
                Dspar = calculateEclusdian(V,E);
                C = sparse(E(:,1),E(:,2),Dspar,n,n);
            elseif (cost_count == 3)
                E3 = C;
                E = E3(:,1:2);
                C = sparse(E(:,1),E(:,2),E3(:,3),n,n);
            else
                eval(['help ' mfilename]);
                error('Invalid inputs.');
            end
        end
    end
    
end

% adjacency-matrix to edge-list conversion
function E = adjToEdgeList(A)
    [Z,X] = find(A);
    E = [Z X];
end

% Compute Euclidean distance for edges
function Dist = calculateEclusdian(V,E)
    Vset = V(E(:,1),:);
    VSet2 = V(E(:,2),:);
    Dist = sqrt(sum((Vset - VSet2).^2,2));
end