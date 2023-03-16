function [trajectory] = circletasktrajectory(start_pos,end_pos,center_pos,num_points)
startpoint = start_pos;
endpoint = end_pos;
centerpoint = center_pos;

radius_vector = startpoint(1,1:3)-centerpoint;
radius_vector2 = endpoint(1,1:3)-centerpoint;
phi = atan2d(norm(cross(radius_vector,radius_vector2)), dot(radius_vector,radius_vector2));
tool_angle_change = (1/num_points)*(end_pos(4)-start_pos(4));


% trajectory matrix has the path
trajectory = zeros(num_points, 4);
for i = 1:num_points
    if phi==90
        curr_pos = radius_vector*cosd((i/num_points)*phi)+radius_vector2*sind((i/num_points)*phi)+centerpoint
    else
        curr_pos =((sind((1-(i/num_points))*phi)/(sind(phi)))*radius_vector+((sind((i/num_points)*phi))/sind(phi))*radius_vector2)+centerpoint;
    end
    angle_pos = start_pos(4)+(i/num_points)*tool_angle_change;
    [c1, c2, c3, c4] = InverseKinematics(curr_pos(1), curr_pos(2), curr_pos(3), angle_pos);
    trajectory(i, :) = [c1, c2, c3, c4];
  
end
end