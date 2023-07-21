%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Title: InverseKinematicsFunction.m
% Course: Introduction to Intelligent Robots I (CPET - 371)
% Developer: Josh Skotarczak (CPET)
% Date: 09/17/2022
% Description: Function that returns thetavalues from Inverse Kinematics 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
function [theta11, theta12, theta21, theta22] = ...
InverseKinematicsFunction( Arm1 ,Arm2, px2, py2)
%%%%%% Inverse Kinematics Theta 2 %%%%%%
c2 = (px2*px2 + py2*py2 - Arm1*Arm1 - Arm2*Arm2) / (2*Arm1*Arm2);
%%%%%% If C is Less Than 1 then Within Range Else Not in Range %%%%%%
if c2 <= 1
s2 = sqrt(1 - c2*c2);
% Define 2 Solutions For Theta 2
theta21 = atan2d(s2, c2); % Solution 1
theta22 = atan2d(-s2, c2); % Solution 2
%%%%%% Inverse Kinematics Theta 1 2 Solutions %%%%%%%%
theta11 = atan2d(py2,px2)-asind((Arm2*sind(theta21))/(sqrt(px2^2 +py2^2)));
theta12 = atan2d(py2,px2)-asind((Arm2*sind(theta22))/(sqrt(px2^2 +py2^2)));
else 
   % If Point is Out of Reach Point Set Theta 1 to point directly to the
   % point out of reach
   theta11 = atan2d(py2,px2);
   theta12 = atan2d(py2,px2);
   theta21 = 0;
   theta22 = 0;
   end
end

