%% =================================================
% Function Q1(ptCloud)
% --------------------------------------------------
% Localize a sphere in the point cloud. Given a point cloud as input, this
% function should locate the position and radius of a sphere.
%
% input: ptCloud -> a pointCloud object that contains the point cloud (see
%                   Matlab documentation)
% output: center -> 3x1 vector denoting sphere center
%         radius -> scalar radius of sphere
%%==================================================
function [center,radius] = Q1(ptCloud)
    %% Defining Constant Terms
    % Maximum number of iterations to run
    MAX_ITERATIONS  = 5000;       
    % Error tolerance for fitness calculation
    EPSILON   = 0.001;      
    
    %% Initialization of Variables
    pc  = ptCloud.Location;
    inCount = -1;
    
    % Calculate Normal at each point
    normals = pcnormals(ptCloud);
    
    %% Calculate samples of points
    for i=1:MAX_ITERATIONS
        %% Pick random point
        [pts, idx] = datasample(pc, 1);
        revVec = normals(idx,:);
        revVec = revVec.*-1;
        
        %% Generate candidate sphere
        [nCenter, nRadius] = fitSphere(pts, revVec);
        
        %% Evaluate fitness of the candidate sphere
        distance = getDistance(pc, nCenter);
        distance = abs(distance - nRadius) < EPSILON;
        count  = sum(distance);
        
        if(count> inCount)
            center = nCenter;
            radius = nRadius;
            inCount  = count;
            disp(['    inCount= ' num2str(count)]);
        end
    end
end

%% =================================================
% Function fitSphere(pts)
% --------------------------------------------------
% Localize a candidate sphere. Given some points in space as input, this
% function should locate the position of center and radius of the candidate sphere.
%
% input: pts -> 4x3 matrix containing 1 3D points to fit
%        revVec -> surface normal of the pts but reversed
% output: center -> 3x1 vector denoting sphere center
%         radius -> scalar radius of sphere
%%==================================================
function [center, radius] = fitSphere(pts, revVec)
    % Select a random radius value between 5 and 11 cm
    a = 0.05;
    b = 0.11;
    radius = rand*(b-a) + a;
    
    % Normalize the vector
    revVec = revVec/norm(revVec);
    
    % Find center coordinates
    center = pts + radius*revVec;

end

%% =================================================
% Function getDistance(X, p)
% --------------------------------------------------
% Given a NxK matrix and a point calculates eucledian distance of p from every point
% in X.
%
% input:  X -> A NxK matrix
% output: dist -> Nx1 matrix of eucledian distance
%%==================================================
function dist = getDistance(X, p)
    l = size(X, 1);
    dist     = X - repmat(p, l, 1);
    dist     = sqrt(sum(dist.^2,2));
end