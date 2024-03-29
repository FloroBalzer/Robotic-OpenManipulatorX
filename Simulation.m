function [robot_body] = Simulation(t1, t2, t3, t4, x, y, z, px, py, pz)

%% Coordinate Frame Nested Function
    function [l1, l2 , l3, l4] = CoordinateFrame(T, n_origin, n_origin_old)
        
        X_pos = n_origin + transpose([T(1,1:end-1); T(2,1:end-1); T(3,1:end-1)]*[5; 0; 0]);
        Y_pos = n_origin + transpose([T(1,1:end-1); T(2,1:end-1); T(3,1:end-1)]*[0; 5; 0]);
        Z_pos = n_origin + transpose([T(1,1:end-1); T(2,1:end-1); T(3,1:end-1)]*[0; 0; 5]);
        
        %plot Coordinate Frame
        X_val = [X_pos(1,1), n_origin(1,1)]; Y_val = [X_pos(1,2), n_origin(1,2)]; Z_val = [X_pos(1,3), n_origin(1,3)];
        l1 = plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'g');
        X_val = [Y_pos(1,1), n_origin(1,1)]; Y_val = [Y_pos(1,2), n_origin(1,2)]; Z_val = [Y_pos(1,3), n_origin(1,3)];
        l2 = plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'b');
        X_val = [Z_pos(1,1), n_origin(1,1)]; Y_val = [Z_pos(1,2), n_origin(1,2)]; Z_val = [Z_pos(1,3), n_origin(1,3)];
        l3 = plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'r');
        
        %plot connecting line
        X_val = [n_origin_old(1,1), n_origin(1,1)]; Y_val = [n_origin_old(1,2), n_origin(1,2)]; Z_val = [n_origin_old(1,3), n_origin(1,3)];
        l4 = plot3(X_val, Y_val, Z_val,'LineWidth', 3, 'Color', 'k');
    end
%% Plot Set up
grid on
xlim([-40, 40]);
ylim([-40, 40]); 
zlim([-10, 80]);
view(40, 20);

%calculate Forward Kinematics
[T01, T12, T23, T34,T45, T02, T03, T04, T05] = ForwardKinematics(t1, t2, t3, t4);%(t1, t2, t3, t4);



hold on
%% Base
base = [0; 0; 0; 1];
origin = [0, 0, 0];
X_pos = [5, 0, 0];
Y_pos = [0, 5, 0];
Z_pos = [0, 0, 5];

X_val = [X_pos(1,1), origin(1,1)]; Y_val = [X_pos(1,2), origin(1,2)]; Z_val = [X_pos(1,3), origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', [0.4660 0.6740 0.1880]);
X_val = [Y_pos(1,1), origin(1,1)]; Y_val = [Y_pos(1,2), origin(1,2)]; Z_val = [Y_pos(1,3), origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', [0 0.4470 0.7410]);
X_val = [Z_pos(1,1), origin(1,1)]; Y_val = [Z_pos(1,2), origin(1,2)]; Z_val = [Z_pos(1,3), origin(1,3)];
plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', [0.6350 0.0780 0.1840]);
%% Joint 1
n_origin = [0, 0, 0];

[l11, l12 , l13, l14] = CoordinateFrame(T01, n_origin, n_origin)

n_origin_old = n_origin;

%% Joint 2
n_origin = transpose(T02*base);
n_origin = n_origin(1:end-1);

[l21, l22 , l23, l24] = CoordinateFrame(T02, n_origin, n_origin_old);

n_origin_old = n_origin;

%% Joint 3
n_origin = transpose(T03*base);
n_origin = n_origin(1:end-1);

[l31, l32 , l33, l34] = CoordinateFrame(T03, n_origin, n_origin_old);

n_origin_old = n_origin;

%% Joint 4
n_origin = transpose(T04*base);
n_origin = n_origin(1:end-1);

[l41, l42 , l43, l44] = CoordinateFrame(T04, n_origin, n_origin_old);

n_origin_old = n_origin;

%% Joint 5
n_origin = transpose(T05*base);
n_origin = n_origin(1:end-1);

[l51, l52 , l53, l54] = CoordinateFrame(T05, n_origin, n_origin_old);

%% Robot Body
robot_body = [l11, l12 , l13, l14;
l21, l22 , l23, l24;
l31, l32 , l33, l34;
l41, l42 , l43, l44;
l51, l52 , l53, l54];

%% Path trajectory
path_point = plot3(px, py, pz, 'o', 'Color', 'b','MarkerSize',6,'MarkerFaceColor','#D9FFFF');
X_val = [px, x]; Y_val = [py, y]; Z_val = [pz, z];
plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'b');


%% Debugging and Rest

% disp('T01');
% disp(T01);
% disp('T12');
% disp(T12);
% disp('T23');
% disp(T23);
% disp('T34');
% disp(T34);
% disp('T45');
% disp(T45);
% disp('T04');
% disp(T04);
% disp('T05');
% disp(T05);

hold off


end