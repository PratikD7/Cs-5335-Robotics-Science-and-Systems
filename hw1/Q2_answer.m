% TODO: You write this function!
% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%                     effector position to reach <position>
%                     (orientation is to be ignored)
function q = Q2_answer(f,qInit,posGoal)

    %A = f.fkine(qInit);
    %tempvar = A(1:end-1, end:end);
    %disp(tempvar);
    
    % Constants
    
	% Define rotation matrix:
    R = [-1 0 0; 0 1 0 ; 0 0 1];
    
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


