% 
% Get adjacency matrix corresponding to the r-disk graph for the given
% radius and numSamples. You should accomplish this by subsampling the
% vertices in distanceMat.
% input: distanceMat -> nxn pairwise distance matrix found in Q1.
%        numSamples -> number of elements to sample from distanceMat
%        radius -> radius with which to calculate r-disk graph
% ouput: adjacency -> binary symmetric matrix where elements correspond to
%                     adjacency matrix for r-disk graph.
function adjacency = Q2(distanceMat, numSamples, radius)

    [m,n] = size(distanceMat);
    
    % Your code here
    for i=1:m
        for j=1:n
            % Pruning the edges with weights greater than radius
            if (distanceMat(i,j) > radius)
                distanceMat(i,j) = -1;
            end
        end
    end
    

    % Randomly subsample the points from the distance matrix
    x = randsample(m,numSamples);
    adjacency = zeros(numSamples, numSamples);

    % Calculate the 
    for i=1:length(x)
       for j=1:length(x)
          if(distanceMat(x(i),x(j))>0)
              adjacency(i,j) = 1;
          end
       end
    end
    %disp(adjacency)
    
end
