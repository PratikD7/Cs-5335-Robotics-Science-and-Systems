% 
% Predict whether r-disk graph on unit cube is connected for a given
% numSamples and radius for a "large" number of samples.
% input: numSamples -> number of vertices in r-disk graph
%        radius -> radius of r-disk graph

function connected = Q3(numSamples, radius)

    dim = 3;
    
    % Your code here
    % Theorem 7
    if (4/3*pi*1.^3)*(radius.^dim) >= (log(numSamples)/numSamples)
        connected = 1;
    else
        connected = 0;
    end
end

%{
Below are the results of one of the runs:
actualConnectivity =
     0     0     0     0     0     0
     0     0     0     0     0     0
     0     0     0     0     1     1
     0     0     1     1     1     1
     0     1     1     1     1     1
     1     1     1     1     1     1
     1     1     1     1     1     1
     1     1     1     1     1     1
     1     1     1     1     1     1


predictedConnectivity =
     0     0     0     0     0     0
     0     0     0     0     0     0
     0     0     1     1     1     1
     0     1     1     1     1     1
     1     1     1     1     1     1
     1     1     1     1     1     1
     1     1     1     1     1     1
     1     1     1     1     1     1
     1     1     1     1     1     1

The predicted connectivity is slightly different than the actual
connectivity. (They only differ in 4 of the positions).
One of the reasons for this difference might be the logic by which both the
connectivities are calculated. Actual connectivity prunes the adjacency
matrix and then randomly subsample. Random Subsampling doesnt occur in 
the Predicted connectivity part because it uses the Theorem of Connectivity
of random r disks graph using the above mentioned inequality.

%}