% TODO: You write this function!
% input: f1 -> a 9-joint robot encoded as a SerialLink class
%        q1Init -> 1x9 vector denoting current joint configuration
%        circle -> 3xn matrix of Cartesian positions that describe the
%                  circle
%        velocity -> scalar denoting desired velocity of end effector
% output: The end effector should trace a circle in the work
%                     space; circle can be of any size in space
%                     (orientation is to be ignored)
function jTrajFull = Q4(f1,qInit,circle,velocity)
 
    % Find intial coordinates of the end effector on the Circle
    %disp(initPos)
    %disp(circle)
    %disp(circle(:,1))
     
    q = Q22_answer(f1,qInit, circle(:,1));
    jTrajFull = [q];
     
    A = f1.fkine(qInit); 
    initPos = A(1:end-1, end:end);
    %disp(initPos)
   
    A = f1.fkine(q);
   currentPos = A(1:end-1, end:end);
   %disp(currentPos)
    
    % Find trajectory of E.E along the circle
    for i = 2:size(circle')
       trajj = Q3_answer(f1,q, circle(:,i), 0.05, velocity);
       q = Q22_answer(f1,q, circle(:,i));
       jTrajFull = [jTrajFull; trajj];
       
       A = f1.fkine(q);
       currentPos = A(1:end-1, end:end);
       %disp(currentPos)
       %disp(jTrajFull)
    end
    %disp(jTrajFull)
end


function q = Q22_answer(f,qInit,posGoal)

    %A = f.fkine(qInit);
    %tempvar = A(1:end-1, end:end);
    %disp(tempvar);
    
    % Constants
    
	% Define rotation matrix:
    R = [1 0 0; 0 -1 0 ; 0 0 1];
    
    % Constants
    alpha = 0.5;
    
    % Find position goals
    q = qInit;
    T = [R posGoal; 0 0 0 1];
    
	% Algorithm:
    for i=1:50
        pos = f.fkine(q);
        
		% Jacobian
		J = f.jacob0(q);
		% Calculating jacobian inverse:
		Jinv = pinv(J);
        
        % Converting homogenous transform dX = (posGoal - p)
		deltaX = tr2delta(pos,T);
        
        % Find joint angles
        deltaQ = alpha * Jinv * deltaX;
        q = deltaQ' + q;
    end
    % disp(q)
end