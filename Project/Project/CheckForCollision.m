% Check if there is a collision between robot and obstacle
% input: rob = PUMA 560 robot links
%      : obstacle 
% output : True if collision detected, False otherwise
function out = CheckForCollision (rob,obstacle)
face_size1 = size (rob.faces, 1);
face_size2 = size (obstacle.faces, 1);
for i = 1:face_size1
    obj1 = rob.vertices(rob.faces(i,:), :);
    for j = 1:face_size2
        obj2 = obstacle.vertices(obstacle.faces(j,:), :);
        if (intersection_check(obj1,obj2))
            out = true;
            return;
        end
    end
end
out = false;
end

% intersection_check : returns true if the faces overlap and false otherwise
function intersection_flag = intersection_check(obj1, obj2)
    intersection_flag = true;
    for k = 1:2 

        coeff = nchoosek(1:size(obj1, 1),2); 
        for i = 1:size(coeff,1)
            objinter = obj1(setdiff((1:size(obj1,1)), coeff(i,:)),:);
            side_one = checkLineIntersection(obj1(coeff(i,1),:), obj1(coeff(i,2), :), objinter);
            side_two = zeros(size(obj2,1),1);
            for j = 1:size(obj2, 1)
                objintr2 = obj1(coeff(i,2),:);
                side_two(j) = checkLineIntersection(obj1(coeff(i,1),:),objintr2, obj2(j,:));
            end
            if length(unique(side_two))==1 && sum(side_two ~= side_one) == size(obj2, 1)
                intersection_flag = false;
                return;
            end
        end
        
        obj = obj1;
        obj1 = obj2;
        obj2 = obj;
    end      
end

