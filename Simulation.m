function Simulation(t1, t2, t3, t4)
clf
grid on

xlim([-0.40, 0.40]);
ylim([-0.40, 0.40]); 
zlim([0, 0.80]);

view(180, 0);

[T01, T12, T23, T34,T02, T03, T04, T05] = ForwardKinematics(t1, t2-79, t3-101, t4-180);%(t1, t2, t3, t4);


base = [0; 0; 0; 1];
hold on

% % Joint 1
n_origin = transpose(T01*base);
n_origin = n_origin(1:end-1);

X = transpose([T01(1,1:end-1); T01(2,1:end-1); T01(3,1:end-1)]*[0.05; 0; 0]);
Y = transpose([T01(1,1:end-1); T01(2,1:end-1); T01(3,1:end-1)]*[0; 0.05; 0]);
Z = transpose([T01(1,1:end-1); T01(2,1:end-1); T01(3,1:end-1)]*[0; 0; 0.05]);


X_pos = n_origin;
Y_pos = n_origin;
Z_pos = n_origin;

X_val = [X_pos(1,1), n_origin(1,1)]; Y_val = [X_pos(1,2), n_origin(1,2)]; Z_val = [X_pos(1,3), n_origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'g');
X_val = [Y_pos(1,1), n_origin(1,1)]; Y_val = [Y_pos(1,2), n_origin(1,2)]; Z_val = [Y_pos(1,3), n_origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'b');
X_val = [Z_pos(1,1), n_origin(1,1)]; Y_val = [Z_pos(1,2), n_origin(1,2)]; Z_val = [Z_pos(1,3), n_origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'r');

X_val = [0, n_origin(1,1)]; Y_val = [0, n_origin(1,2)]; Z_val = [0, n_origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 3, 'Color', 'k');
n_origin_old = n_origin;

% Joint 2
n_origin = transpose(T02*base);
n_origin = n_origin(1:end-1);

X = transpose([T02(1,1:end-1); T02(2,1:end-1); T02(3,1:end-1)]*[0.05; 0; 0]);
Y = transpose([T02(1,1:end-1); T02(2,1:end-1); T02(3,1:end-1)]*[0; 0.05; 0]);
Z = transpose([T02(1,1:end-1); T02(2,1:end-1); T02(3,1:end-1)]*[0; 0; 0.05]);

X_pos = n_origin + X;
Y_pos = n_origin + Y;
Z_pos = n_origin + Z;

X_val = [X_pos(1,1), n_origin(1,1)]; Y_val = [X_pos(1,2), n_origin(1,2)]; Z_val = [X_pos(1,3), n_origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'g');
X_val = [Y_pos(1,1), n_origin(1,1)]; Y_val = [Y_pos(1,2), n_origin(1,2)]; Z_val = [Y_pos(1,3), n_origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'b');
X_val = [Z_pos(1,1), n_origin(1,1)]; Y_val = [Z_pos(1,2), n_origin(1,2)]; Z_val = [Z_pos(1,3), n_origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'r');

X_val = [n_origin_old(1,1), n_origin(1,1)]; Y_val = [n_origin_old(1,2), n_origin(1,2)]; Z_val = [n_origin_old(1,3), n_origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 3, 'Color', 'k');
n_origin_old = n_origin;

% Joint 3
n_origin = transpose(T03*base);
n_origin = n_origin(1:end-1);

X = transpose([T03(1,1:end-1); T03(2,1:end-1); T03(3,1:end-1)]*[0.05; 0; 0]);
Y = transpose([T03(1,1:end-1); T03(2,1:end-1); T03(3,1:end-1)]*[0; 0.05; 0]);
Z = transpose([T03(1,1:end-1); T03(2,1:end-1); T03(3,1:end-1)]*[0; 0; 0.05]);

X_pos = n_origin + X;
Y_pos = n_origin + Y;
Z_pos = n_origin + Z;

X_val = [X_pos(1,1), n_origin(1,1)]; Y_val = [X_pos(1,2), n_origin(1,2)]; Z_val = [X_pos(1,3), n_origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'g');
X_val = [Y_pos(1,1), n_origin(1,1)]; Y_val = [Y_pos(1,2), n_origin(1,2)]; Z_val = [Y_pos(1,3), n_origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'b');
X_val = [Z_pos(1,1), n_origin(1,1)]; Y_val = [Z_pos(1,2), n_origin(1,2)]; Z_val = [Z_pos(1,3), n_origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'r');

X_val = [n_origin_old(1,1), n_origin(1,1)]; Y_val = [n_origin_old(1,2), n_origin(1,2)]; Z_val = [n_origin_old(1,3), n_origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 3, 'Color', 'k');
n_origin_old = n_origin;

% % Joint 4
n_origin = transpose(T04*base);
n_origin = n_origin(1:end-1);

X = transpose([T04(1,1:end-1); T04(2,1:end-1); T04(3,1:end-1)]*[0.05; 0; 0]);
Y = transpose([T04(1,1:end-1); T04(2,1:end-1); T04(3,1:end-1)]*[0; 0.05; 0]);
Z = transpose([T04(1,1:end-1); T04(2,1:end-1); T04(3,1:end-1)]*[0; 0; 0.05]);

X_pos = n_origin + X;
Y_pos = n_origin + Y;
Z_pos = n_origin + Z;

X_val = [X_pos(1,1), n_origin(1,1)]; Y_val = [X_pos(1,2), n_origin(1,2)]; Z_val = [X_pos(1,3), n_origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'g');
X_val = [Y_pos(1,1), n_origin(1,1)]; Y_val = [Y_pos(1,2), n_origin(1,2)]; Z_val = [Y_pos(1,3), n_origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'b');
X_val = [Z_pos(1,1), n_origin(1,1)]; Y_val = [Z_pos(1,2), n_origin(1,2)]; Z_val = [Z_pos(1,3), n_origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'r');

X_val = [n_origin_old(1,1), n_origin(1,1)]; Y_val = [n_origin_old(1,2), n_origin(1,2)]; Z_val = [n_origin_old(1,3), n_origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 3, 'Color', 'k');
n_origin_old = n_origin;

% % Joint 5
n_origin = transpose(T05*base);
n_origin = n_origin(1:end-1);

X = transpose([T05(1,1:end-1); T05(2,1:end-1); T05(3,1:end-1)]*[0.05; 0; 0]);
Y = transpose([T05(1,1:end-1); T05(2,1:end-1); T05(3,1:end-1)]*[0; 0.05; 0]);
Z = transpose([T05(1,1:end-1); T05(2,1:end-1); T05(3,1:end-1)]*[0; 0; 0.05]);

X_pos = n_origin + X;
Y_pos = n_origin + Y;
Z_pos = n_origin + Z;

X_val = [X_pos(1,1), n_origin(1,1)]; Y_val = [X_pos(1,2), n_origin(1,2)]; Z_val = [X_pos(1,3), n_origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'g');
X_val = [Y_pos(1,1), n_origin(1,1)]; Y_val = [Y_pos(1,2), n_origin(1,2)]; Z_val = [Y_pos(1,3), n_origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'b');
X_val = [Z_pos(1,1), n_origin(1,1)]; Y_val = [Z_pos(1,2), n_origin(1,2)]; Z_val = [Z_pos(1,3), n_origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'r');

X_val = [n_origin_old(1,1), n_origin(1,1)]; Y_val = [n_origin_old(1,2), n_origin(1,2)]; Z_val = [n_origin_old(1,3), n_origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 3, 'Color', 'k');


disp('Endposition');
disp(T05(1:end-1,4));
hold off
end