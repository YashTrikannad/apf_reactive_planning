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
no_of_obs = size(map.obstacles,1);

eta = 4*10^(3);
rho_not = 200 ; 
zeta = 100; 
D = 10; 
epsilon = 1; 
alpha = 0.01;

if all(abs(qCurr(1,1:5) - map.goal(1,1:5))<epsilon)
    isDone = 1;
    fprintf("reached target")
    return;
end

tau_1 = zeros(6,1);   

%% Calculate the attractive force, a 6x1 vector 
for i = 1 : 7 % for 6 joints

      Frep= zeros(1,3);
      Fatt = zeros(1,3);
      
      %attractive force
      if norm(end_joint_coords(i,:) - start_joint_coords (i,:)) < D  % for a distance lesser than D would be parabolic curve
            %parabolic wwell
            Fatt = -zeta.*( (start_joint_coords(i, :) - end_joint_coords( i , :))');           


      else  % conic well
            
           Fatt = -D*zeta*((start_joint_coords(i,:) - end_joint_coords(i,:))' / norm(start_joint_coords(i , : ) - end_joint_coords( i , :)));
           
      end
    
    %% Calculate the repulsive forces 
   
    for k = 1:no_of_obs
            [rhoiq , deltarho ]  = distPointToBox(start_joint_coords(i,:), map.obstacles(k,:));        
            if (rhoiq < rho_not )   
                 Frep = Frep + eta*( (1/ rhoiq ) - (1/rho_not)) * ( 1/ (rhoiq)^2) * deltarho ;% passing each force of each joint to calculate torque.
            end
    end

    Frep = Frep' ;
    Jacobian = (calcJacobian_33(qCurr , i, robot ))';
    Jacobian = Jacobian(1:3);
    F_tot = (Fatt + Frep);
    tau_1 = tau_1 + Jacobian*F_tot; 

end
 
tau_1 = tau_1';

qNext = qCurr + (alpha * (tau_1 / norm (tau_1)));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end