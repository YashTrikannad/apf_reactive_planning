function [qNext, isDone] = potentialFieldStep_33(qCurr, map, robot)
% POTENTIALFIELDSTEP_GROUPNO Calculates a single step in a potential field
%   planner based on the virtual forces exerted by all of the elements in
%   map. This function will be called over and over until isDone is set.
%   Use persistent variables if you need historical information. CHANGE 
%   GROUPNO TO YOUR GROUP NUMBER.
%
% INPUTS:
%   qCurr - 1x6 vector representing the current configuration of the robot.
%   map   - a map struct containing the boundaries of the map, any
%           obstacles, the start position, and the goal position.
%   robot - a struct of robot parameters
%
% OUTPUTS:
%   qNext  - 1x6 vector representing the next configuration of the robot
%            after it takes a single step along the potential field.
%   isDone - a boolean flag signifying termination of the potential field
%            algorithm. 
%
%               isDone == 1 -> Terminate the planner. We have either
%                              reached the goal or are stuck with no 
%                              way out.
%               isDone == 0 -> Keep going.

%%

qNext = zeros(1,6);
qNext = qCurr;
qNext(1) = qNext(1)+.01;
isDone = 0;

start_joint_coords = calculateFK_sol(qCurr, robot);
end_joint_coords = calculateFK_sol(map.goal, robot);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

epsilon = 0.1;
rho = 10;

if all(abs(qCurr(1,1:5) - map.goal(1,1:5))<epsilon)
    isDone = 1;
    return;
end
    
%% Calculate the attractive force, a 6x1 vector 

%initialize taua
tauaTotal = zeros(6,1); 
taurTotal = zeros(6,1); 
    
%Find all the J matrices using qCurr 
JAll = [];
    
syms q1 q2 q3 q4 q5 q6

for joint = 1:6
    %% Calculate the attractive joint effort for joint i 
    % Find the positions of the current position and the end goal of the
    % joint i. 
    currPosJointI = start_joint_coords(joint,:);
    endPosJointI = end_joint_coords(joint,:);
        
    % instantiate Fa 
    Fa = zeros(1,3); 
        
    % if case over whether the current pos of joint i is at it's goal 
    if( norm(currPosJointI - endPosJointI) ~= 0)
        % if joint i is not at its goal position, then calculate Fa
        Fa = - (currPosJointI - endPosJointI) / norm(currPosJointI - endPosJointI);
    else
        continue;
    end 
        
    % find the jacobian for joint i 
    J = calcJacobian_33(qCurr,joint, robot);

    % substitute values of qCur into symbolic J 
    J = subs(J, q1, qCurr(1));
    J = subs(J, q2, qCurr(2));
    J = subs(J, q3, qCurr(3));
    J = subs(J, q4, qCurr(4));
    J = subs(J, q5, qCurr(5));
    J = subs(J, q6, qCurr(6));
        
    % Convert sym matrix back to numeric
    J = double(J);

    % Only take the first joint columns 
    Jnew = J(:,1:joint); 
        
    % add zeros if Jnew is not a 6x5
    colsOfZerosToAdd = 6 - joint;
    Jnew = [Jnew, zeros(6,colsOfZerosToAdd)];
        
    % calculate the taua for this joint 
    Jv = Jnew(1:3,:);
    taua = Jv'*Fa'; %6x1   =    [6x3][3x1]

    % find the total Taua by adding the taua for this joint 
    tauaTotal = tauaTotal + taua; %final result of 5x6 matrix
    
    %% Calculate the repulsive forces 
    % initialize a Fr vector;
    Fr = zeros(1,3);
         
    %loop over all the obstacles we have
    for obstacle = 1:size(map.obstacles,1)
        %find the center point of the particular obstacles
        posOfObstacle = map.obstacles(obstacle,:);

        %find the radius of the particular obstacle
        distance = distPointToBox(currPosJointI, posOfObstacle);

        %figure out whether the joint falls within the sphere of
        %influence of the obstacle. this is a boolean 
        jointIsWithinSphereOfInfluence = distance < rho;

        %Robot is within the sphere of influence of the obstacle
        nu = 0.1;
        %find the expression Fr
        firstPart = zeros(1,3);
        if(jointIsWithinSphereOfInfluence)
            firstPart = 1 ./ currPosJointI - 1 ./distance;
        end
        Fr = nu * firstPart;
    end

    %find the taur for obstacle j on joint i
    taur = Jv'*Fr';
    %add taur to taurTotal
    taurTotal = taurTotal + taur; 
end 

%set the tau = to the tauatotal + the taurtotal
tau = tauaTotal + taurTotal;

%set the step rate
alpha = 0.02;

%ensure there is not
if (norm(tau)~=0)
    qNext = qCurr' + alpha * (tau / norm(tau));
else
    qNext = qCurr';
end

qNext = qNext';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end