% Find a path from qStart to qGoal using RRT.
% 
% input: qStart -> 1x4 vector describing start configuration
%        qGoal -> 1x4 vector describing goal configuration
%        qMin -> 1x4 vector of minimum joint angle values
%        qMax -> 1x4 vector of maximum joint angle values
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle.
% output: qMilestones -> mx4 matrix of vertices along path from start to
%                        goal.
function qMilestones = Q5(rob,qStart,qGoal,qMin,qMax,sphereCenter,sphereRadius)

    % Maximum allowable iterations
    iterations = 10000;
    
    % Distance limit
    delta = 0.50;
    
    % Tree
    %node = zeros(iterations/10, 4);
    %parent = zeros(iterations/10, 4);
    node(1,:) = qStart;
    parent(1,:) = [-10, -10, -10, -10];
    
    index = 1;
    %disp(node(1,:));
    %disp(parent(1, :));
    
    flag = 0;
    
    % Your code here
    for i=1:iterations
       if (flag==1)
          break; 
       end
       %disp(i);
       if (mod(i,100)==0)
           % For every 100th iteration samplePoint is the Goal Point
          samplePoint = qGoal;
       else
           % Random point in the space boundary
           samplePoint = rand(1,4) .* repmat(qMax - qMin,[1 1]) + repmat(qMin,[1 1]);
       end
       
       % Check if SamplePoint is in Free space or not
       if ~(robotCollision(rob, samplePoint, sphereCenter, sphereRadius))
           % Find the closest point in the tree wrt samplepoint
           closestDist = Inf;
           closestPoint = qStart;
           
           for j=1:index
              dist = norm(node(j,:) - samplePoint);
              if (dist <= delta) && dist < closestDist
                  closestDist = dist;
                  closestPoint = node(j,:);
              end
           end
           if (closestDist==Inf)
              continue 
           else
               % Check if a collision free path exists between the two
               % nodes
               if ~(checkEdge(rob, closestPoint, samplePoint, sphereCenter, sphereRadius))
                   node(index+1, :) = samplePoint;
                   parent(index+1, :) = closestPoint;
                   index = index + 1;
                   if (samplePoint == qGoal)
                       flag=1;
                   end
               end
           end 
       end
    end
    
   
    % Calculate the path
    
    %qMilestones
    currentPoint = qGoal;
    endPoint = qStart;
    q(1,:) = qGoal;
    index=2;
    % Find the path from starting node to the end node
    q = findPath(currentPoint, endPoint, parent, node, index, q);
    
    % Reverse the path so as to get from qStart to qGoal
    for i=1:length(q)
       qMilestones(i,:) = q(length(q)-i+1,:); 
    end
    %disp(qMilestones);
end

% Find path using recursion
function q = findPath(currentPoint, endPoint, parent, node, index, q)
    ind = find(node==currentPoint);
    %disp(ind(1));
    par = parent(ind(1),:);
    if (par==endPoint)
        q(index, :) = par;
        %disp(q);
        return ;
    else
        q(index,:) = par;
        %disp(q);
        q = findPath(par, endPoint, parent, node, index+1, q);
        return;
    end
end


