% You must run startup_rvc FIRST before running this function.
% DO NOT MODIFY THIS FILE!
% input: questionNum -> Integer between 1 and 5 that denotes question
%                       number to run.
function hw2(questionNum)

    close all;
    
    if nargin < 1
        error('Error: please enter a question number as a parameter');
    end

    % Create robot
	rob = createRobot();

    % Configure start/goal configuration; min/max c-space bounds
    qStart = [0 -0.78 0 -0.78];
    qGoal = [0.0007 -2.9677 0.0015 -2.7938];
    qMin = [-pi/2,-pi,0,-pi];
    qMax = [pi/2,0,0,0];
    
    % Set up obstacle
    sphereCenter = [0.5;0.0;0];
    sphereRadius = 0.2;

    % Parameters for PRM and RRG in Q4 and Q5.
    prmNumSamples = 3000;
    prmRadius = 0.5;
        
    % Search space for Q1 -- Q3
    [numSamplesMat radiusMat] = meshgrid(500:500:3000, 0.05:0.03:0.3);
%     [numSamplesMat radiusMat] = meshgrid(1000:500:10000, 0.005:0.02:0.20);
    
    % Plot robot and sphere
    rob.plot(qStart);
    hold on;	
    drawSphere(sphereCenter,sphereRadius);

    % Get adjacency matrix for random nodes in unit cube
    if questionNum == 1

        % Get adjacency matrix
        [nodes, distanceMat] = Q1(6);

        if ~issymmetric(distanceMat)
            error('Error: adjacencyMat not symmetric')
        end
        
        if size(nodes,1) ~= size(distanceMat,1)
            error('<nodes> must have same number of rows as <adjacencyMat>')
        end
        
    % Determine whether r-disk graph is connected for a matrix of
    % numSamples and radiuses.
    elseif questionNum == 2

        % Calculate cell array of adjacency matrices
        [nodes, distanceMat] = Q1(max(max(numSamplesMat)));
        adjacencyCellArray = getAdjacency(distanceMat, numSamplesMat, radiusMat);
        
        % Evaluate connectivity for each element of numSamplesMat and radiusMat.
        actualConnectivity = zeros(size(numSamplesMat));
        for i = 1:size(adjacencyCellArray,1)
            for j = 1:size(adjacencyCellArray,2)
                G = graph(adjacencyCellArray{i,j});
                actualConnectivity(i,j) = size(unique(conncomp(G)),2) == 1;
            end
        end

        % Display comparison between connectivity predictions and actual
        display(actualConnectivity)
    
    % Determine whether r-disk graph *should* be connected for a matrix of
    % numSamples and radiuses.
    elseif questionNum == 3
        
        % Test Q3 function
        display(sprintf('This should be connected: %d', Q3(3000, 0.3)))
        
        % Now, compare predicted adjacencies to actual adjacencies...
        
        % Calculate cell array of adjacency matrices
        [nodes, distanceMat] = Q1(max(max(numSamplesMat)));
        adjacencyCellArray = getAdjacency(distanceMat, numSamplesMat, radiusMat);
        
        % Evaluate connectivity for each element of numSamplesMat and radiusMat.
        actualConnectivity = zeros(size(numSamplesMat));
        predictedConnectivity = zeros(size(numSamplesMat));
        for i = 1:size(adjacencyCellArray,1)
            for j = 1:size(adjacencyCellArray,2)
                G = graph(adjacencyCellArray{i,j});
                actualConnectivity(i,j) = size(unique(conncomp(G)),2) == 1;
                predictedConnectivity(i,j) = Q3(numSamplesMat(i,j),radiusMat(i,j));
            end
        end

        % Display comparison between connectivity predictions and actual
        display(actualConnectivity)
        display(predictedConnectivity)

    elseif questionNum == 4
        
        % Generate n uniform random samples in c-space between qMin and qMax
        samples = rand(prmNumSamples,4) .* repmat(qMax - qMin,[prmNumSamples 1]) + repmat(qMin,[prmNumSamples 1]);
        samples(1,:) = qStart;
        samples(2,:) = qGoal;

        % Prune collision samples and get weighted adjacency matrix for
        % r-disk graph.
        [samplesFree, adjacencyMat] = Q4(rob,samples,prmRadius,sphereCenter,sphereRadius);

        % Create matlab graph object
        G = graph(adjacencyMat);
        
        % Display number of connected components
        numComponents = size(unique(conncomp(G)),2);
        display(sprintf('number of connected components: %d',numComponents))

        % Find shortest path
        [path, pathLength] = shortestpath(G,1,2);
        display(sprintf('path length: %f',pathLength))
        %disp(size(path));
        qMilestones = samplesFree(path,:);
        
        if pathLength == Inf
            error('No path found!')
        end
        
        % Plot robot following path
        qTraj = interpMilestones(qMilestones);
        rob.plot(qTraj);
        
    elseif questionNum == 5
        
        qMilestones = Q5(rob,qStart,qGoal,qMin,qMax,sphereCenter,sphereRadius);
        %disp(qMilestones);
        % Plot robot following path
        qTraj = interpMilestones(qMilestones);
        rob.plot(qTraj);
        
    else
        error('Error: question number out of range.');        
    end
    
    
    
end

function traj = interpMilestones(qMilestones)

    d = 0.05;
%     traj = qMilestones(1,:);
    traj = [];
    for i=2:size(qMilestones,1)
        
        delta = qMilestones(i,:) - qMilestones(i-1,:);
        m = max(floor(norm(delta) / d),1);
        vec = linspace(0,1,m);
        leg = repmat(delta',1,m) .* repmat(vec,size(delta,2),1) + repmat(qMilestones(i-1,:)',1,m);
        traj = [traj;leg'];
        
    end
end

function qPath = getPath(tree)

    m = 10;
    idx = size(tree,1);
    path = tree(end,1:end-1);
    
    while(idx ~= 1)
        
        curr = tree(idx,1:end-1);
        idx = tree(idx,end);
        next = tree(idx,1:end-1);
        path = [path;[linspace(curr(1),next(1),m)' linspace(curr(2),next(2),m)' linspace(curr(3),next(3),m)' linspace(curr(4),next(4),m)']];
        
    end
    qPath = path(end:-1:1,:);
    
end


function rob = createRobot()

    L(1) = Link([0 0 0 1.571]);
    L(2) = Link([0 0 0 -1.571]);
    L(3) = Link([0 0.4318 0 -1.571]);
    L(4) = Link([0 0 0.4318 1.571]);
%     L(5) = Link([0 0.4318 0 1.571]);
    
    rob = SerialLink(L, 'name', 'robot');

end

function drawSphere(position,diameter)

%     diameter = 0.1;
    [X,Y,Z] = sphere;
    X=X*diameter;
    Y=Y*diameter;
    Z=Z*diameter;
    X=X+position(1);
    Y=Y+position(2);
    Z=Z+position(3);
    surf(X,Y,Z);
    %~ shading flat

end


