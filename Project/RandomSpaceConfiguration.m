% Create a randomly spaced obstacle in the world space
% input : obstacle 
% output : configuration space of the obstacle if it doesnt collide with
% the robot configuration
function conf = RandomSpaceConfiguration(obstacle)
while true
    conf = rand(6,1)*360;
    rob = CreatePuma(conf);
    if (~CheckForCollision(rob, obstacle))
        return
    end
end
end