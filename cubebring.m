function [traj1, traj2, traj3, traj4, traj5,traj6] = cubebring(cube)
%starts in home position, brings a cube to the rotation area
%x should be the cube position array for that cube, already has a -90 angle
%by default, effector pointing downards
area = cubeposition(x,y); %position of rotation area
start = [5, 0, 20, 0.3];
traj1 = linetasktrajectory(start,cube,30);
%run through values of trajectory
hold = [cube(1),cube(2),cube(3) - offset,cube(4)];
traj2 = linetasktrajectory(cube, hold,30);%move to cube level
%run through traj
%close gripper
traj3 = linetasktrajectory(hold,cube,30);% move up
%run though  traj
traj4= linetasktrajectory(cube,area,30);%move to area
%run through traj
drop = [area(1),area(2),area(3) - offset,area(4)];%offset from cubeposition
traj5 = linetasktrajectory(area,drop,30);%move to area
%run through traj
%drop the cube (open grip)
traj6 = easedtrajectory(drop, area, 30);%move back to area
%make sure to keep track of the grippers state during the code so it doesnt
%think it needs to open twice




end