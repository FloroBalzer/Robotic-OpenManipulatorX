function [cubex, cubey, handz,angle] = cubeposition(xh,yh)
%xh,yh are the position of the cube based on the holes themselves count the
%holes
%returns the arguments that the IK need to position itself above the cube
angle = -90; %hand pointing straight down
cubeheight = 3; %need to measure
offset = 3; %decide how much higher the hand is going to be BEFORE lowering to pick it up
cubex = xh*d;
cubey = yh*d;
handz = cubeheight+offset;

end