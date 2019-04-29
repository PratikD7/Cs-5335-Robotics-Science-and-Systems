% TODO: You write this function!
% input: f1 -> a 9-joint robot encoded as a SerialLink class
%        q1Init -> 1x9 vector denoting current joint configuration
%        goalPos -> 3x1 vector describing goal position
%        epsilon -> scalar denoting how close end effector must be before
%                   loop terminates
%        velocity -> scalar denoting desired velocity of end effector
% output: traj -> nx9 matrix that denotes arm trajectory. Each row denotes
%                 a joint configuration. The first row is the first
%                 configuration. The last is the last configuration in the
%                 trajectory.

function traj=Q3_answer(f1,q1Init,goalPos,epsilon,velocity)
    
    % Find intial coordinates of the end effector
    A = f1.fkine(q1Init);
    initPos = A(1:end-1, end:end);
    % disp(initPos);

    q = q1Init;
    pos = initPos;
    C = norm(goalPos - initPos);
    %disp(C);
    traj = q;
    %disp(traj)
    while (C > epsilon)
        ratio = velocity/ norm(pos-goalPos);
        %disp(ratio)
        pos = [((1-ratio)*pos(1)+ratio*goalPos(1));((1-ratio)*pos(2)+ratio*goalPos(2));((1-ratio)*pos(3)+ratio*goalPos(3))];
        %disp(pos)
        qnew = Q22_answer(f1, q, pos);
        %disp(qnew);
        
        traj = [traj;qnew];
        %disp(traj)
        
        q = qnew;
        %disp(qnew)
        C = norm(goalPos - pos);
        %disp(C)
    end
    %disp (traj);
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