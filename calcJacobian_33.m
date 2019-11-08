function J = calcJacobian_33(q, joint, robot)
% CALCJACOBIAN_GROUPNO Calculate the Jacobian of a particular joint of the 
%   robot in a given configuration. CHANGE GROUPNO TO YOUR GROUP NUMBER.
%
% INPUTS:
%   q     - 1 x 6 vector of joint inputs [q1,q2,q3,q4,q5,q6]
%   joint - scalar in [1,7] representing which joint we care about
%   robot - a struct of robot parameters
%
% OUTPUTS:
%   J - 6 x (joint-1) matrix representing the Jacobian
%

%%

J = [];

if nargin < 3
    return
elseif joint <= 1 || joint > 7
    return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Your Code Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

syms q1 q2 q3 q4 q5 q6

L1 = 3*25.4;          %base height (in mm)
L2 = 5.75*25.4;       %shoulder to elbow length (in mm)
L3 = 7.375*25.4;      %elbow to wrist length (in mm)
L4 = 1.75*25.4;       %Wrist1 to Wrist2 (in mm)
L5 = 1.25*25.4;       %wrist2 to base of gripper (in mm)
L6 = 1.125*25.4;      %gripper length (in mm)

%Frame 1 w.r.t Frame 0
A1 = [cos(q1) -sin(q1)*cos(-sym(pi)/2)  sin(q1)*sin(-sym(pi)/2)  0;
      sin(q1)  cos(q1)*cos(-sym(pi)/2) -cos(q1)*sin(-sym(pi)/2)  0;
        0            sin(-sym(pi)/2)            cos(-sym(pi)/2) L1;
        0                     0                  0              1];
          
%Frame 2 w.r.t Frame 1          
A2 = [cos(q2-(sym(pi)/2)) -sin(q2-(sym(pi)/2))  0   L2*cos(q2-(sym(pi)/2));
      sin(q2-(sym(pi)/2))  cos(q2-(sym(pi)/2))  0   L2*sin(q2-(sym(pi)/2));
              0                        0  1                     0;
              0                        0  0                     1];

%Frame 3 w.r.t Frame 2
A3 = [cos(q3+(sym(pi)/2)) -sin(q3+(sym(pi)/2))  0   L3*cos(q3+(sym(pi)/2));
      sin(q3+(sym(pi)/2))  cos(q3+(sym(pi)/2))  0   L3*sin(q3+(sym(pi)/2));
              0                        0  1                     0;
              0                        0  0                     1];

%Frame 4 w.r.t Frame 3
A4 = [cos(q4-(sym(pi)/2)) -sin(q4-(sym(pi)/2))*cos(-sym(pi)/2)   sin(q4-(sym(pi)/2))*sin(-sym(pi)/2)   0;
      sin(q4-(sym(pi)/2))  cos(q4-(sym(pi)/2))*cos(-sym(pi)/2)  -cos(q4-(sym(pi)/2))*sin(-sym(pi)/2)   0;
              0                          sin(-sym(pi)/2)                    cos(-sym(pi)/2)   0;
              0                                   0                             0   1];
%Frame 5 w.r.t Frame 4 
A5 = [cos(q5) -sin(q5)  0        0;
      sin(q5)  cos(q5)  0        0;
              0          0  1  L4 + L5;
              0          0  0        1];
          
%Gripper Frame w.r.t. Frame 5
A6 = [1 0 0  0;
      0 1 0  0;
      0 0 1  L6;
      0 0 0  1];
  
% Find the position of joint i 
posOfJoint = 1; 
if (joint >= 1)
    posOfJoint = posOfJoint * A1;
    if (joint >= 2)
        posOfJoint = posOfJoint * A2;
        if (joint >= 3)
            posOfJoint = posOfJoint * A3;
            if (joint >= 4)
                posOfJoint = posOfJoint * A4;
                if (joint >= 5)
                    posOfJoint = posOfJoint * A5;
                    if (joint >= 6)
                        posOfJoint = posOfJoint * A6;
                    end
                end
            end
        end
    end
end

posOfJoint = posOfJoint*[0;0;0;1];
pos = posOfJoint(1:3,1);

% Find the Jv by taking deriviatives 
Jv = sym(zeros(3,6));

Jv(:,1) = diff(pos,q1); 
Jv(:,2) = diff(pos,q2); 
Jv(:,3) = diff(pos,q3);
Jv(:,4) = diff(pos,q4);
Jv(:,5) = diff(pos,q5);
Jv(:,6) = diff(pos,q6);

%% Angular Velocity
R01 = A1(1:3,1:3);
R12 = A2(1:3,1:3);
R23 = A3(1:3,1:3);
R34 = A4(1:3,1:3);
R45 = A5(1:3,1:3);

R02 = R01*R12;
R03 = R01*R12*R23;
R04 = R01*R12*R23*R34;
R05 = R01*R12*R23*R34*R45;

z_hat = [0 0 1]';
Jw = [1*z_hat 1*R01*z_hat 1*R02*z_hat 1*R03*z_hat 1*R04*z_hat 0*R05*z_hat];

%% Body Velocity 
J = [Jv; Jw];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Your Code Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end