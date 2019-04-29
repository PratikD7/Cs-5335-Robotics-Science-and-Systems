% PRM - ProbablisticRoadMap : Algorithm for Sampling:
%   Input : Obstacle, sample_count and K
%  Sample_count = Number of iterations of sampling in PRM algorithm

%   Output :roadmap 
function roadmap = PRM (obstacle, sample_count, k)
pts = zeros(sample_count+2,2);
random_config = RandomSpaceConfiguration(obstacle);
random_samples = repmat(random_config(:), 1, sample_count);
edge_set = zeros(k*sample_count, 2);
costs_edge = zeros(sample_count*k, 1);
edge_count = 0;

for i = 2:sample_count
    random_config = RandomSpaceConfiguration(obstacle);
    random_samples(:,i) = random_config(:);
    distances = CalculateDistance(random_config, random_samples(:,1:(i-1)));
    [sorted_dist, index] = sort(distances,'ascend');

% Resampling: Once a node is selected to be expanded:
% 1. Pick a random motion direction in c-space and move in this direction
% until an obstacle is hit.
% 2. When a collision occurs, choose a new random direction and proceed for
% some distance.
% 3. Add the resulting nodes and edge_set to the tree. Re-run tree connection
% step.

    for j = 1: min(k,i-1)
        if (LocalPlanner(random_config, random_samples(:,index(j)), obstacle))
            rob = CreatePuma(random_config);
            f= rob.vertices;
            x= f(1,1);
            y = f(1,2);
            pt = [x,y];
            pts(i,:) = pt;
            edge_count = edge_count + 1;
            edge_set(edge_count,:) = [i,index(j)];
            costs_edge(edge_count) = sorted_dist(j);
        end
    end 
    %fprintf (1, 'sample_count = %d\n', i, edge_count);
    fprintf (1, 'sample_count = %d, edge_count = %d\n', i, edge_count);
end
roadmap.pts = pts;
roadmap.random_samples = random_samples;
roadmap.edge_set = edge_set(1:edge_count, :);
roadmap.costs_edge = costs_edge(1:edge_count);