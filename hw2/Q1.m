% Compute pairwise distance between random samples in the unit cube.
% input: numSamples -> number of random samples in unit cube.
% output: nodes -> nx3 matrix of randomly sampled vertices
%         distanceMat -> nxn matrix of pairwise distances between samples.
%                        The i,j elt of distanceMat denotes the distance
%                        between sample i and sample j.
%                        Note that distanceMat should be symmetric with
%                        zeros on the diagonal.
function [nodes, distanceMat] = Q1(numSamples)

    n = numSamples;
    m = 3; % three dimensions
    qMin = [0 0 0];
    qMax = [1 1 1];
    
    nodes = zeros(n,m);
    
    % Your code here
    for i=1:n
        for j=1:m
        % Generate random points using the formula below
        t = rand(1);
        points = (1-t) * qMin(j) + t * qMax(j);
        nodes(i,j) = points;
        end
    end
    
    % Calculate the Distance Matrix
    distanceMat = squareform(pdist(nodes));
    
    %disp(nodes);
    %disp(distanceMat);

end
