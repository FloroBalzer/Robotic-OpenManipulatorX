Points = [15, 0, 15;  22,0,12;  24,0,12;  26,0,12;  28,0,12;  30,0,12];

i=1;
while i < size(Points,1)+1 
    px = Points(i,1);
    py = Points(i,2);
    pz = Points(i,3);
    phi = 0;
    
%      disp(size(Points,1));
%      disp(px);
%      disp(py);
%      disp(pz);
    
    [t1, t2, t3, t4] = InverseKinematics(px, py, pz, phi);
%    t1 = 180;
%    t2 = 180;
%    t3 = 180;
%    t4 = 135;
% disp(t2-atand(0.024/0.128)-79+t3+atand(0.024/0.128)-101+t4-180);
 Simulation(t1, t2, t3, t4, Points(1:i,1:end));
 i = i+1;
end

