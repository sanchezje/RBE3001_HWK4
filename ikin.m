function [jointAngles] = ikin(tipPosn)
%IKIN Calculates the Inverse Kinematics of the Robot
%   Consumes 3x1 vector representing tip position in mm
%   Produces a 3x1 vector representing corrosponding joint angles in degrees to
%   produce that position

%% Define vars

%syms px py pz a1 a2 a3 theta1 theta2 theta3 pxt;

a1 = 135;
a2 = 175;
a3 = 169.28;

px = tipPosn(1);
py = tipPosn(2);
pz = tipPosn(3) - a1;


jointAngles = [0;0;0];


%% Calculate theta1
% in the first axis, computing the angle of rotation is trivial
theta1 = -atan2(py, px);
disp(theta1)

%% Account for rotation
% since the base rotates, the effective px in the xz plane stretches
% the new coord plane will be the x'-z' plane

pxt = sqrt(px^2 + py^2);
disp(pxt)

%% Calculate theta2
% now that the rotation is accounted for, can use the 2-link robot
% equations as defined from slides in class

eqn1 = atan2(pz, pxt);
eqn2 = pxt^2 + pz^2 + a2^2 - a3^2;
eqn3 = 2*a2 * sqrt(pxt^2 + pz^2);

theta2 = eqn1 + acos(eqn2/eqn3);
disp(theta2)
eqn1 = 0;
eqn2 = 0;
eqn3 = 0;
%% Calculate theta3
% now that the rotation is accounted for, can use the 2-link robot
% equations as defined from slides in class

eqn1 = pxt^2 + pz^2;
eqn2 = a2^2 + a3^2;
eqn3 = 2*a2*a3;

theta3 = acos((eqn1 - eqn2)/eqn3);
disp(theta3)
%% Convert angles to degrees

jointAngles = (360/(2*pi)) * [theta1; theta2; theta3];
disp(jointAngles)
%% Check to see if angles are valid
% once we generate these angles, we must determine if they are feasible for
% the robot to achieve them
% -90 < theta1 < 90
% 0 < theta2 < 90
% -225 < theta3 < 45
% should check all of these later

t1min = -90;
t1max = 90;
t2min = 0;
t2max = 90;
t3min = -225;
t3max = 45;


% a bit inefficient code here
% but makes it more useful to determine what went wrong

if(theta1 < t1min) || (theta1 > t1max)
    error("theta1 out of bounds");
end

if(theta2 < t2min) || (theta2 > t2max)
    error("theta2 out of bounds");
end

if(theta3 < t3min) || (theta3 > t3max)
    error("theta3 out of bounds");
end

end

