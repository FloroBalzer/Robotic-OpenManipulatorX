% Start and end pos, using inputs floro gave
function [trajectory] = easedtrajectory(start_pos,end_pos,num_points)

% start and end IKs
[s1, s2, s3, s4] = InverseKinematics(start_pos(1), start_pos(2), start_pos(3), start_pos(4));
[t1, t2, t3, t4] = InverseKinematics(end_pos(1), end_pos(2), end_pos(3), end_pos(4));
start = [s1, s2, s3, s4];
endm = [t1, t2, t3, t4] ;


% trajectory matrix has the path, implements cubic eased trajectory
trajectory = zeros(num_points, 4);
for i = 1:num_points
    t = i/num_points;
    eased_t = t^3 * (3 - 2*t);
    trajectory(i, :) = start*(1-eased_t) + endm*eased_t;
end
end
