function Animate_3D_Pose(varargin)%X,ray_sum,t_sum, W, mode, ray_dis
%ANIMATE_3D_POSE Summary of this function goes here
%   Detailed explanation goes here

    X = varargin{1};
    ray_sum = varargin{2};
    t_sum = varargin{3};
    mode = varargin{4};
    ray_dis = varargin{5};
    t_pause = varargin{6};
    R_sum = varargin{7};
%end
P = size(X,1)/3;
F = size(X,2);
if P == 18
    R = 7;
    limbSeq = [2 3; 2 6; 3 4; 4 5; 6 7; 7 8; 2 9; 9 10; 10 11; 2 12; 12 13; 13 14; 2 1; 1 15; 15 17; 1 16; 16 18];%; 3 17; 6 18];
    view(7.6000, -62.0000);
    camsize = 0.1;
elseif P == 31%||P == 1
    R = 210;
    limbSeq1 = [1 2;2 3;3 4;4 5;5 6];
    limbSeq2 = [1 7;7 8;8 9;9 10;10 11];
    limbSeq3 = [1 12;12 13;13 14;14 15;15 16;16 17];
    limbSeq4 = [14 18;18 19;19 20;20 21;21 22;22 23];
    limbSeq5 = [21 24];
    limbSeq6 = [14 25;25 26;26 27;27 28;28 29;29 30];
    limbSeq7 = [28 31];
    limbSeq = [limbSeq1;limbSeq2;limbSeq3;limbSeq4;limbSeq5;limbSeq6;limbSeq7];
    view(175.3000, -62.8000);
    camsize = 10;
elseif P == 15
    R = 600;
    %limbSeq = [2 1;1 3;1 4;4 5;5 6;3 7;7 8;8 9;1 10;10 11;11 12;3 13;13 14;14 15];
    limbSeq = [1  2; 2  3; 3  4; 4  5; 2  6; 6  7; 7  8; 2 15; 15  12; 12  13; 13  14; 15  9; 9  10; 10  11];
    view(7.2, -70)
    camsize = 10;
else
    R = 2000;
    limbSeq = [1 1];
    camsize = 10;
end
if nargin == 2||nargin == 4
    W_temp_1 = diag(0.5*ones(F-1,1));
    W_temp_1(1,1) = 1;
    W_temp_2 = diag(0.5*ones(F-1,1));
    W_temp_2(end,end) = 1;    
    W = zeros(F,F);
    W(2:F,1:F-1) = W(2:F,1:F-1) + W_temp_2;
    W(1:F-1,2:F) = W(1:F-1,2:F) + W_temp_1;
    sequence = zeros(2,F);
    sequence(1,:) = 1:F;
    sequence(2,:) = 0:0.5:(F-1)*0.5;  
end
for f = 1:F

    points_temp = reshape(X(:,f),[3,P]);
    if f == 1
        hold on
        camera = plotCamera('Location', t_sum{f},'Orientation', R_sum{f}, 'Size', camsize);
        points_plot = plot3(points_temp(1,:) , points_temp(2,:),  points_temp(3,:), '.', 'MarkerSize', 20, 'MarkerEdgeColor', 'r'); 
        axis equal;
        axis off;
        axis([min(min(X(1:3:end,:)))-0.7*R max(max(X(1:3:end,:)))+0.7*R min(min(X(2:3:end,:)))-0.7*R max(max(X(2:3:end,:)))+0.7*R min(min(X(3:3:end,:)))-0.7*R max(max(X(3:3:end,:)))+0.7*R]);
            
        num_lines = size(limbSeq,1);
        lines_x = reshape(points_temp(1,limbSeq), [num_lines,2]);
        lines_y = reshape(points_temp(2,limbSeq), [num_lines,2]);
        lines_z = reshape(points_temp(3,limbSeq), [num_lines,2]);

        lines_plot = cell(num_lines,1);
        lines_plot_GT = cell(num_lines,1);

        for p = 1:num_lines
            lines_plot{p} = line(lines_x(p,:), lines_y(p,:), lines_z(p,:), 'Color', 'r', 'LineWidth', 2);
        end
        if ray_dis == 1
            for p = 1:P
                rays_plot{p} = line([t_sum{f}(1) ray_sum{f}(1,p)*R+t_sum{f}(1)], [t_sum{f}(2) ray_sum{f}(2,p)*R+t_sum{f}(2)], [t_sum{f}(3) ray_sum{f}(3,p)*R+t_sum{f}(3)]);
            end
        end
        pause;
        
    else
        camera.Location = t_sum{f};
        camera.Orientation = R_sum{f};       
        points_plot.XData = points_temp(1,:);
        points_plot.YData = points_temp(2,:);
        points_plot.ZData = points_temp(3,:);

        lines_x = reshape(points_temp(1,limbSeq), [num_lines,2]);
        lines_y = reshape(points_temp(2,limbSeq), [num_lines,2]);
        lines_z = reshape(points_temp(3,limbSeq), [num_lines,2]);

        for p = 1:num_lines
            lines_plot{p}.XData = lines_x(p,:);
            lines_plot{p}.YData = lines_y(p,:);
            lines_plot{p}.ZData = lines_z(p,:);
        end
        if ray_dis == 1
            for p = 1:P
                rays_plot{p}.XData = [t_sum{f}(1) ray_sum{f}(1,p)*R+t_sum{f}(1)];
                rays_plot{p}.YData = [t_sum{f}(2) ray_sum{f}(2,p)*R+t_sum{f}(2)];
                rays_plot{p}.ZData = [t_sum{f}(3) ray_sum{f}(3,p)*R+t_sum{f}(3)];
            end
        end
    end
    %disp(f);
    if mode == 1
        pause;
    elseif mode == 2
        pause(t_pause)
    end
end
end