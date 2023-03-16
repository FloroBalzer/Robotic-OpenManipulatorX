function [trajectory, no_point] = trajectory_path(point_matrix)

trajectory=[];
no_point = 0;
i=1;
while i <= size(point_matrix,1)
    start_pos = point_matrix(i,1:4);% format [x, y, z, sin(angle)]
    end_pos = point_matrix(i,5:8); % format [x, y, z, sin(angle)]
    no_point_new = point_matrix(i,9);
    if point_matrix(i,10) == 1
        trajectory_new = linetasktrajectory(start_pos, end_pos, no_point_new);
    else
       center_pos = point_matrix(i,11:13); 
       trajectory_new = circletasktrajectory(start_pos, end_pos,center_pos, no_point_new);
    end
    trajectory = [trajectory; trajectory_new];
    no_point = no_point + no_point_new;
    i = i+1;
end
