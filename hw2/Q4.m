% Create the sPRM graph for a given set of samples.
% input: samples -> nx4 matrix of samples for which to create r-disk graph
%        prmRadius -> radius to be used to construct r-disk-graph
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle.
% output: samplesFree -> prmNumSamples x 4 matrix of roadmap vertices. These are
%                        the vertices that you sample. But, rows 1 and 2 are
%                        special: row 1 should be set to qStart; row 2 should be
%                        set of qGoal.
%         adjacencyMat -> nxn adjacency matrix. If there is an edge between
%                         vertices i and j, then element i,j should be set
%                         to the Euclidean distance between i and j. OW, it
%                         element i,j should be set to zero.
%                         Note that this matrix should have zeros on the
%                         diagonal and it should be symmetric. (weighted adjacency matrix)
%
function [samplesFree, adjacencyMat] = Q4(rob,samples,prmRadius,sphereCenter,sphereRadius)

    % Your code here
    
    % Initialize samplefree woth q0 and qgoal
    samplesFree(1,:) = samples(1,:);
    samplesFree(2,:) = samples(2,:);
    
    %disp(samples(1,:))
    k=3;
    
    % Calculate the samples in a free space
    for i=3:length(samples)
        if robotCollision(rob, samples(i,:), sphereCenter, sphereRadius) == 0
           samplesFree(k,:) = samples(i,:);
           k = k + 1;
        end
    end
    
    
    % Calculate the adjacency matrix
    [n,m] = size(samplesFree);
    adjacencyMat = zeros(n, n);
    
    for i=1:n
        for j=i:n
            eucDist = norm(samplesFree(i,:) - samplesFree(j,:));
            if eucDist <= prmRadius
                if checkEdge(rob,samplesFree(i,:),samplesFree(j,:),sphereCenter,sphereRadius) == 0
                    adjacencyMat(i,j) = eucDist;
                    adjacencyMat(j,i) = eucDist;
                end
            end
        end
    end
end

