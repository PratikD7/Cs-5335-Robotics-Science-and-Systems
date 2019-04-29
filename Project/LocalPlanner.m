% Local planning checks whether there is a local path between two milestones
% input : 2 configuratioons and obstacles
% output: True if there is no obstacles from one config to another, False
% otherwise
function output = LocalPlanner (config1, config2, obstacle)

angle_diff = config2 - config1;
flag = angle_diff < -180;
angle_diff(flag) = angle_diff(flag) + 360;
flag = angle_diff > 180;
angle_diff(flag) = angle_diff(flag) - 360;

sample_count = ceil(sum(abs(angle_diff)) / 10);
for i = 1:sample_count
    config = mod(config1+ (i/sample_count)*angle_diff, 360);
    rob = CreatePuma(config);
    if (CheckForCollision(rob,obstacle))
        output = false;
        return
    end
end
output = true;
end