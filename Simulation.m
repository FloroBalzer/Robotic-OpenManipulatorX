function Simulation(t1, t2, t3, t4, Points)
clf
%% Coordinate Frame Nested Function
    function CoordinateFrame(T, n_origin, n_origin_old)
        
        X_pos = n_origin + transpose([T(1,1:end-1); T(2,1:end-1); T(3,1:end-1)]*[5; 0; 0]);
        Y_pos = n_origin + transpose([T(1,1:end-1); T(2,1:end-1); T(3,1:end-1)]*[0; 5; 0]);
        Z_pos = n_origin + transpose([T(1,1:end-1); T(2,1:end-1); T(3,1:end-1)]*[0; 0; 5]);
        
        %plot Coordinate Frame
        X_val = [X_pos(1,1), n_origin(1,1)]; Y_val = [X_pos(1,2), n_origin(1,2)]; Z_val = [X_pos(1,3), n_origin(1,3)];
        plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'g');
        X_val = [Y_pos(1,1), n_origin(1,1)]; Y_val = [Y_pos(1,2), n_origin(1,2)]; Z_val = [Y_pos(1,3), n_origin(1,3)];
        plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'b');
        X_val = [Z_pos(1,1), n_origin(1,1)]; Y_val = [Z_pos(1,2), n_origin(1,2)]; Z_val = [Z_pos(1,3), n_origin(1,3)];
        plot3(X_val, Y_val, Z_val,'LineWidth', 1, 'Color', 'r');
        
        %plot connecting line
        X_val = [n_origin_old(1,1), n_origin(1,1)]; Y_val = [n_origin_old(1,2), n_origin(1,2)]; Z_val = [n_origin_old(1,3), n_origin(1,3)];
        plot3(X_val, Y_val, Z_val,'LineWidth', 3, 'Color', 'k');
    end
%% Plot Set up
grid on
xlim([-40, 40]);
ylim([-40, 40]); 
zlim([0, 80]);
view(135, 45);

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

CoordinateFrame(T01, n_origin, n_origin)

n_origin_old = n_origin;

%% Joint 2
n_origin = transpose(T02*base);
n_origin = n_origin(1:end-1);

CoordinateFrame(T02, n_origin, n_origin_old);

n_origin_old = n_origin;

%% Joint 3
n_origin = transpose(T03*base);
n_origin = n_origin(1:end-1);

CoordinateFrame(T03, n_origin, n_origin_old);

n_origin_old = n_origin;

%% Joint 4
n_origin = transpose(T04*base);
n_origin = n_origin(1:end-1);

CoordinateFrame(T04, n_origin, n_origin_old);

n_origin_old = n_origin;

%% Joint 5
n_origin = transpose(T05*base);
n_origin = n_origin(1:end-1);

CoordinateFrame(T05, n_origin, n_origin_old);

%% Path trajectory
i = 1;
while i < size(Points,1)
%     disp("enter");
    plot3(Points(i,1), Points(i,2), Points(i,3), 'LineWidth', 1, 'Color', 'm'); 
    i = i+1;
end

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
disp('T05');
disp(T05);
hold off
end