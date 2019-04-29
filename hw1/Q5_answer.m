% TODO: You write this function!
% input: f1 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        f2 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        qInit -> 1x11 vector denoting current joint configuration.
%                 First seven joints are the arm joints. Joints 8,9 are
%                 finger joints for f1. Joints 10,11 are finger joints
%                 for f2.
%        f1Target, f2Target -> 3x1 vectors denoting the target positions
%                              each of the two fingers.
% output: q -> 1x11 vector of joint angles that cause the fingers to
%              reach the desired positions simultaneously.
%              (orientation is to be ignored)
function q = Q5_answer(f1,f2,qInit,f1Target,f2Target)
    % Constants
    alpha = 0.05;
    R    = [-1 0 0; 0 -1 0; 0 0 1];
    FILL  = [0 0; 0 0; 0 0; 0 0; 0 0; 0 0];
    
    % Goal positions
    pGf1 = [R  f1Target; 0 0 0 1];
    pGf2 = [R  f2Target; 0 0 0 1];
    
    q = qInit;
    for i=1:50
        % Calculate current angles
        qf1 = [q(:, 1:7) q(:,  8: 9)];
        qf2 = [q(:, 1:7) q(:, 10:11)];
        
        % Calculate current positions 
        posf1 = f1.fkine(qf1);
        posf2 = f2.fkine(qf2);
        
        % Calculate current delta 
        dXf1 = tr2delta(posf1, pGf1);     
        dXf2 = tr2delta(posf2, pGf2);     
        dX   = [dXf1; dXf2];            
        
        % Calculate Jacobian
        Jf1  = f1.jacob0(qf1);              
        Jf1  = [Jf1(:,1:7) Jf1(:,8:9) FILL]; 
        Jf2  = f2.jacob0(qf2);              
        Jf2  = [Jf2(:,1:7) FILL Jf2(:,8:9)]; 
        J    = [Jf1; Jf2];                  
        
        % Calculate Jacobian inverse
        Ji   = pinv(J);     
        
        % Find deltaQ
        deltaQ = alpha * Ji * dX;
        q  = deltaQ' + q;
    end
end

    
