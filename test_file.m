 T05 = [1.0,   0,  0,  0.2748;
        0,  0,  1.0,   0;
        0, -1.0,  0, 0.2050;
        0,  0,  0,  1];
 [t1, t2, t3, t4] = InverseKinematics(T05);
Simulation(t1, t2, t3, t4);
%Simulation(180, 180, 180, 180);