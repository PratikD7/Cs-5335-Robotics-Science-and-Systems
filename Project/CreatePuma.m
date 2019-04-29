% Create a full representation of the PUMA robot
% input : A 6-linked robot starting configuration
% output : A struct object of PUMA 560 robot links

function robo = CreatePuma(config)
    link = createRobotLinks(0,5,-0.5,0.5);
    robo = link;
    for i = 1:5
        robo = appendRobotLinks(link, rotateStruct(robo, config(i),[5 0]));
    end
    robo = rotateStruct(robo,config(end),[0 0]);
end

% Create robot vertices and faces
% input : 4 points in space
% output : Robot's vertices
%        : Robot's faces
function robo = createRobotLinks(p1,p2,p3,p4)
    robo.vertices = [p1 p3; p2 p3; p2 p4; p1 p4];
 	robo.faces = [1 2 3; 1 3 4];
end

% Append two robot links
% input : 2 structures of robot links
% output : A combination of the two link structure
function robo = appendRobotLinks (struct1, struct2)
    sizeOfStruct = size(struct1.vertices,1);
	robo.vertices = [struct1.vertices; struct2.vertices];
	robo.faces = [struct1.faces; struct2.faces + sizeOfStruct];
end

% Rotate the link structure according to the configuration
% input : rob = A robot link
%       : theta = Configuration of the link in degrees
%       : dist = Distance of the link
% output: A modified robot link structure according to the configuration
function output = rotateStruct(rob, theta, dist)
    output.faces = rob.faces;
	cosine = cosd(theta); sine = sind(theta);
    mat = [cosine sine; -sine cosine];
	output.vertices = bsxfun(@plus, rob.vertices*mat,dist);
end