% Compute the distance between two configurations
%Formula for comaprison of distance between two configurations
%p(theta1, theta2) = min {|theta1 - theta2|, 2*pi - |theta1 - theta2|}
% input : 2 configuration of the robot
% output : Distance between tbe two configurations in degrees
function output = CalculateDistance (config1, config2)
    angle_dif = abs(bsxfun(@minus, config2, config1));
    output = sum(min(angle_dif,360-angle_dif));
end