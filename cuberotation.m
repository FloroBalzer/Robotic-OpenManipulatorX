%script for after the cube has been picked up and brought to the rotation
%space: the robot should be in the area above the space after having run
%through cubebring
%it should not enter this script if the red face is already up so i will
%ignore that case
function cuberotation(rot) %rot =number of rotations

%NEEDED pick up the cube, go back to "area" position while holding it effector -90, should make a script for
%this
for i=1:rot
    turn = [area(1),area(2),area(3),area(4)+90]; %rotate cube over rotation area
    trajectory = easedtrajectory(area,turn,30);
    %run through trajectory values
    hold = [area(1),area(2),area(3) - offset,area(4)+90];% value of offset from cubeposition, lowers the cube down the the holder
    trajectory2 = easedtrajectory(turn,hold,30);%run through trjectory values
    %let go of the cube - again script needed
    trajectory2 = easedtrajectory(hold,area,30); %return to area, ready for next rotation if needed
end



    

end    
