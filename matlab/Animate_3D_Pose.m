function Animate_3D_Pose(varargin)
%ANIMATE_3D_POSE Summary of this function goes here
%   Detailed explanation goes here
    warning off;
    X = varargin{1};
    t_sum = varargin{2};
    R_sum = varargin{3};
    limbSeq = varargin{4};
    param = varargin{5};
    
    mode = param.mode;
    t_pause = param.t_pause;
    
    
    
    P = size(X,1)/3;
    F = size(X,2);
if P == 18
    R = 20;
    if isempty(limbSeq)
        limbSeq = [2 3; 2 6; 3 4; 4 5; 6 7; 7 8; 2 9; 9 10; 10 11; 2 12; 12 13; 13 14; 2 1; 1 15; 15 17; 1 16; 16 18];
    end
    az = 7.6;
    el = -62;
    view(az, el);
    camsize = 0.1;
    figure(1)
    hold on
    [Xground,Zground] = meshgrid((-2:0.5:2)-1, (-2:0.5:2)+4);
    Yground = Xground*0 +1.3;
    surf(Xground,Yground,Zground,'Facelighting','gouraud','EdgeColor','white','FaceColor', [0.501, 0.501, 0.501]);
    xr = 0.04;
    zr = 0.04;
    nellipse = 20;
elseif P == 31%||P == 1
    R = 210;
    if isempty(limbSeq)
        limbSeq1 = [1 2;2 3;3 4;4 5;5 6];
        limbSeq2 = [1 7;7 8;8 9;9 10;10 11];
        limbSeq3 = [1 12;12 13;13 14;14 15;15 16;16 17];
        limbSeq4 = [14 18;18 19;19 20;20 21;21 22;22 23];
        limbSeq5 = [21 24];
        limbSeq6 = [14 25;25 26;26 27;27 28;28 29;29 30];
        limbSeq7 = [28 31];
        limbSeq = [limbSeq1;limbSeq2;limbSeq3;limbSeq4;limbSeq5;limbSeq6;limbSeq7];
    end
    az = 175;
    el = -62.8;
    view(az, el);
    camsize = 10;
    xr = 1;
    zr = 1;
    nellipse = 20;
elseif P == 15
    R = 400;
    if isempty(limbSeq)
        limbSeq = [2 1;1 3;1 4;4 5;5 6;3 7;7 8;8 9;1 10;10 11;11 12;3 13;13 14;14 15];
    end
    az =0.296;
    el = -75;
    view(az, el)
    camsize = 10;
    hold on
    [Xground,Zground] = meshgrid((-150:25:150), (-150:25:150));
    Yground = Xground*0 -1;
    surf(Xground,Yground,Zground);
    colormap([0.95  0.95  0.95])
    xr = 4;
    zr = 4;
    nellipse = 20;
elseif P == 45
    R = 4;
    if isempty(limbSeq)
        limbSeq = [1 1];
    end
    camsize = 0.2;
    az = 25.3;
    el = -58.6;
    view(az, el)
    colormap([0.95  0.95  0.95])
    xr = 0.04;
    zr = 0.04;
    nellipse = 20;
elseif P == 17
    R = 15;
    if isempty(limbSeq)
        limbSeq  = [[0, 1]; [1, 2]; [2, 3]; [0, 4]; [4, 5]; [5, 6]; [0, 7]; [7, 8]; [8, 9]; [9, 10]; [8, 14]; [14, 15]; [15, 16]; [8, 11]; [11, 12]; [12, 13]] +1;
    end
    camsize = 0.5;
    az = 93;
    el = 6;
    view(az, el)
    
    figure(1)
    hold on
    [Xground,Yground] = meshgrid((min(min(X(1:3:end,:))):(max(max(X(1:3:end,:)))-min(min(X(1:3:end,:))))/10:max(max(X(1:3:end,:)))), (min(min(X(2:3:end,:))):(max(max(X(2:3:end,:)))-min(min(X(2:3:end,:))))/10:max(max(X(2:3:end,:)))));
    Zground = Xground*0+min(min(X(3:3:end,:)));
    I = imread('floor.jpg');
    surf(Xground,Yground,Zground,'Facelighting','gouraud','EdgeColor','white','FaceColor', [0.501, 0.501, 0.501]);
    %colormap([0.95  0.95  0.95])
    xr = 0.04;
    zr = 0.04;
    nellipse = 20;
elseif P == 89
    if isempty(limbSeq)
        limbSeq = [1 1];
    end
    camsize = 0.02;    
    R = 2;
    az = 7.2;
    el = -50;
    view(az, el)
elseif P == 30
    R = 600;
    if isempty(limbSeq)
        limbSeq = [2 1;1 3;1 4;4 5;5 6;3 7;7 8;8 9;1 10;10 11;11 12;3 13;13 14;14 15];%[1  2; 2  3; 3  4; 4  5; 2  6; 6  7; 7  8; 2 15; 15  12; 12  13; 13  14; 15  9; 9  10; 10  11];% headtop, neck,rShoulder, rElbow, rWrist, lShouder, lElbow, lwrist, rhip, rknee, rankle, lhip, lknee, lankle, bodycenter
        limbSeq = [limbSeq;limbSeq+15];
    end
    az = 7.2;
    el = -70;
    view(az, el)
    camsize = 10;
else
    R = 4;
    camsize = 10;
end
%=================color map===============
colors = hsv(length(limbSeq));

for f = 1:F
    tic;
    points_temp = reshape(X(:,f),[3,P]);

    if f == 1
        hold on
        camera = plotCamera('Location', t_sum{f},'Orientation', R_sum{f}, 'Size', camsize);
        points_plot = plot3(points_temp(1,:) , points_temp(2,:),  points_temp(3,:), '.', 'MarkerSize', 10, 'MarkerEdgeColor', 'k'); 

        axis equal;
        axis off;
        axis([min(min(X(1:3:end,:)))-0.7*R max(max(X(1:3:end,:)))+0.7*R min(min(X(2:3:end,:)))-0.7*R max(max(X(2:3:end,:)))+0.7*R min(min(X(3:3:end,:)))-0.7*R max(max(X(3:3:end,:)))+0.7*R]);
              
        num_lines = size(limbSeq,1);
        lines_x = reshape(points_temp(1,limbSeq), [num_lines,2]);
        lines_y = reshape(points_temp(2,limbSeq), [num_lines,2]);
        lines_z = reshape(points_temp(3,limbSeq), [num_lines,2]);

        lines_plot = cell(num_lines,1);
        trajectory = cell(P,1);
        for p = 1:num_lines
            lines_plot{p} = line(lines_x(p,:), lines_y(p,:), lines_z(p,:), 'Color', colors(p,:), 'LineWidth', 2);
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
%         for p = 1:P
%             trajectory{p} = plot3(X(p*3-2,f-1:f),X(p*3-1,f-1:f),X(p*3,f-1:f),'-','Color', 'b', 'MarkerSize', 0.1,'LineWidth', 0.1);
%             trajectory{p}.Color(4) = 1*f/F;
%         end
    end
    ttmp = toc;
    if mode == 1
        pause;
    elseif mode == 0
         if t_pause-ttmp>0
            pause(t_pause-ttmp)
        else
            continue;
        end
    end
end
delete(camera)
end