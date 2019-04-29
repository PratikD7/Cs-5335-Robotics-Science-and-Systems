%% =================================================
% Function Q2(ptCloud)
% --------------------------------------------------
% Localize a cylinder in the point cloud. Given a point cloud as input, this
% function should locate the position and orientation, and radius of the
% cylinder.
% input: ptCloud -> a pointCloud object that contains the point cloud (see
%                   Matlab documentation)
% output: center -> 3x1 vector denoting cylinder center
%         axis -> 3x1 unit vector pointing along cylinder axis
%         radius -> scalar radius of cylinder
%%==================================================
function [center,axis,radius] = Q2(ptCloud)
    %% Defining Constant Terms
    % Max number of iterations to fit circle in projection
    MAX_ITERATION_CIRCLE  = 1000; 
    % Threshold for inlier calculation
    INLIER_THRESHOLD   = 10000;     
    % Error tolerance for fitness calculation
    EPSILON        = 0.001;    
    
    %% Initialization
    normals  = pcnormals(ptCloud, 500);
    pc  = ptCloud.Location;
    pct = pc';
    inCnt = -1;
    
    %% Calculate samples
    flag = false;
    while(~flag)
        %% Calculate axis
        % Pick 2 random surface normals
        n  = datasample(normals, 2); 
        n1 = n(1,:); 
        n2 = n(2,:);

        % Calculate orthagonal vector to surface normals by cross product
        nat = cross(n1, n2);
        nat = nat / norm(nat);
        na  = nat';
        
        %% Project points to plane orthagonal to axis
        plane  = (eye(3,3) - na*nat) * pct;
        plane  = plane';
        
        % Translate to 2D 
        [pts2D, locx, locy, or] = translate3Dto2D(plane);
        
        %% Try to fit circle to projected points
        for j=1:MAX_ITERATION_CIRCLE
            %% Generate candidate cicle
            % Sample 3 points
            [pts, idx] = datasample(pts2D, 1);
            revVec = normals(idx,1:2);
            revVec = revVec.*-1;
            
            % Fit circle to sample points
            [nc, nr] = fitCircle(pts, revVec);
            
            % Skip iteration if radius too big
            if(isnan(nr) || nr>0.3) 
                continue
            end
            
            % Translate to local points back to 3D system
            nc = translate2Dto3D(nc, locx, locy, or);
            
            %% Evaluate fitness
            dist = getDistance(plane, nc);
            dist = abs(dist - nr);
            dist = dist < EPSILON;
            cnt  = sum(dist);
            
            if(cnt> inCnt)
                inCnt  = cnt;
                disp(['inCount= ' num2str(cnt)]);
            end
            
            %% Is threshold passed?
            if(cnt > INLIER_THRESHOLD)
                center = nc';
                radius = nr;
                axis   = na;
                flag  = true;
                
                axis(3) = abs(axis(3));
                
                %% Calculate height of cylinder
                % Project points to plane parallel to axis
                temp  = (eye(3,3) - n1'*n1) * pct;

                % Translate to 2D local point system
                [temp, ~, ~, ~] = translate3Dto2D(temp');
                temp   = max(temp) - min(temp);
                height = temp(2);
                
                %% Reposition center
                center = center + (axis * height/2);
            end
        end
    end
    axis
    center
    radius
    inCnt
end


%% =================================================
% Function fitCircle(pts)
% --------------------------------------------------
% Given 3 points calculates a circle that passes through those points
%
% input: pts -> 3 x 2 matrix 
%
% output: center  -> Is the center of the circle 1x2
%         radius  -> Is the radius of the circle (scalar)
%
% cite: http://www.qc.edu.hk/math/Advanced%20Level/circle%20given%203%20points.htm
%%==================================================
function [center, radius] = fitCircle(pts, revVec)
    % Select a random radius value between 1 and 11 cm
    a = 0.01;
    b = 0.11;
    radius = rand*(b-a) + a;
    
    % Normalize the vector
    revVec = revVec/norm(revVec);
    
    % Find center coordinates
    center = pts + radius*revVec;
end



%% =================================================
% Function translate3Dto2D(pts)
% --------------------------------------------------
% Translates a planar co-ordinates from a 3D system to a local 2D system
%
% input: pts -> N x 3 matrix 
%
% output: locx  -> Is the Local translation in x(scalar)
%         locy  -> Is the Local translation in y(scalar)
%         pts2D -> Is the translated N x 2 matrix
%         origin -> 1x3 is the origin of local system 
% 
% cite: http://stackoverflow.com/questions/26369618/getting-local-2d-coordinates-of-vertices-of-a-planar-polygon-in-3d-space
%%==================================================
function [pts2D, locx, locy, origin] = translate3Dto2D(pts)
    p0 = pts(1,:); p1 = pts(2,:); p2 = pts(3,:);
    
    loc0 = p0;                       % local origin
    locx = p1 - loc0;                % local X axis
    locz = cross(locx, p2 - loc0);   % vector orthogonal to polygon plane
    locy = cross(locz, locx);        % local Y axis
    
    locx = locx/norm(locx);
    locy = locy/norm(locy);
    
    pts2D = zeros(size(pts, 1), 2);
    for i=1:size(pts, 1)
        p = pts(i,:) - loc0;
        pts2D(i,:) = [dot(p, locx)    % local X coordinate
                      dot(p, locy)];  % local Y coordinate
    end
    origin = loc0;
end


%% =================================================
% Function translate2Dto3D(pts, locx, locy, origin)
% --------------------------------------------------
% Translates a planar co-ordinates from a local 2D system to a 3D system
%
% input: pts    -> N x 3 matrix 
%        locx   -> Is the Local translation in x(scalar)
%        locy   -> Is the Local translation in y(scalar)
%        origin -> 1x3 is the origin of local system 
%
% output: pts2D -> Is the translated N x 2 matrix
%
% cite: http://stackoverflow.com/questions/26369618/getting-local-2d-coordinates-of-vertices-of-a-planar-polygon-in-3d-space
%%==================================================
function pts3D = translate2Dto3D(pts, locx, locy, origin)
    pts3D = zeros(size(pts, 1), 3);
    for i=1:size(pts, 1)
        Lx = pts(i,1); Ly = pts(i,2);
        pts3D(i,:) = origin + Lx*locx + Ly*locy;
    end
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