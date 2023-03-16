% Start and end pos, using inputs floro gave
function [trajectory] = linetasktrajectory(start_pos,end_pos,num_points)

startpoint = start_pos;
endpoint = end_pos;

path_vector = (1/num_points)*(endpoint-startpoint); %scaled path vector

% trajectory matrix has the path
trajectory = zeros(num_points, 4);
for i = 1:num_points
    curr_pos = path_vector*i+startpoint;
    [c1, c2, c3, c4] = InverseKinematics(curr_pos(1), curr_pos(2), curr_pos(3), curr_pos(4));
    trajectory(i, :) = [c1, c2, c3, c4];
  
end
end
