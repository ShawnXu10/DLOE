function Animate_3D_Pose(varargin)%X,ray_sum,t_sum, W, mode, ray_dis
%ANIMATE_3D_POSE Summary of this function goes here
%   Detailed explanation goes here
    warning off;
    X = varargin{1};
    ray_sum = varargin{2};
    t_sum = varargin{3};
    mode = varargin{4};
    ray_dis = varargin{5};
    t_pause = varargin{6};
    R_sum = varargin{7};
    ellipsoidLimb = varargin{8};
P = size(X,1)/3;
F = size(X,2);
if P == 18
    R = 20;
    limbSeq = [2 3; 2 6; 3 4; 4 5; 6 7; 7 8; 2 9; 9 10; 10 11; 2 12; 12 13; 13 14; 2 1; 1 15; 15 17; 1 16; 16 18];%; 3 17; 6 18];
    az = 7.6;
    el = -62;
    view(az, el);
    camsize = 0.1;
    figure(1)
    hold on
    [Xground,Zground] = meshgrid((-2:0.5:2)-1, (-2:0.5:2)+4);
    Yground = Xground*0 +1.3;
    I = imread('floor.jpg');
    warp(Xground,Yground,Zground,I);

    %surf(Xground,Yground,Zground,'Facelighting','gouraud','EdgeColor','white','FaceColor', [0.501, 0.501, 0.501]);
    %colormap('gray'),
    xr = 0.04;
    zr = 0.04;
    nellipse = 20;
elseif P == 31%||P == 1
    R = 150;
    limbSeq1 = [1 2;2 3;3 4;4 5;5 6];
    limbSeq2 = [1 7;7 8;8 9;9 10;10 11];
    limbSeq3 = [1 12;12 13;13 14;14 15;15 16;16 17];
    limbSeq4 = [14 18;18 19;19 20;20 21;21 22;22 23];
    limbSeq5 = [21 24];
    limbSeq6 = [14 25;25 26;26 27;27 28;28 29;29 30];
    limbSeq7 = [28 31];
    limbSeq = [limbSeq1;limbSeq2;limbSeq3;limbSeq4;limbSeq5;limbSeq6;limbSeq7];
    az = 175;
    el = -62.8;
    view(az, el);
    camsize = 10;
    xr = 1;
    zr = 1;
    nellipse = 20;
elseif P == 15
    R = 400;
    %limbSeq = [2 1;1 3;1 4;4 5;5 6;3 7;7 8;8 9;1 10;10 11;11 12;3 13;13 14;14 15];
    %limbSeq = [1  2; 2  3; 3  4; 4  5; 2  6; 6  7; 7  8; 2 15; 15  12; 12  13; 13  14; 15  9; 9  10; 10  11];
    limbSeq = [2 1;1 3;1 4;4 5;5 6;3 7;7 8;8 9;1 10;10 11;11 12;3 13;13 14;14 15];%[1  2; 2  3; 3  4; 4  5; 2  6; 6  7; 7  8; 2 15; 15  12; 12  13; 13  14; 15  9; 9  10; 10  11];% headtop, neck,rShoulder, rElbow, rWrist, lShouder, lElbow, lwrist, rhip, rknee, rankle, lhip, lknee, lankle, bodycenter
    %limbSeq = [2 9;2 3;2 6;3 4;4 5;6 7;7 8;9 10;10 11;11 12;9 13; 13 14;14 15;2 1];
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
    limbSeq = [1 1];
    camsize = 0.2;
    az = 25.3;
    el = -58.6;
    figure(1)
    hold on
%     ptcld = pcread('ClimbBGCroped.ply');
%     pcshow(ptcld);
%    view(az, el)
    colormap([0.95  0.95  0.95])
    xr = 0.04;
    zr = 0.04;
    nellipse = 20;
elseif P == 17
    R = 15;
    limbSeq  = [[0, 1]; [1, 2]; [2, 3]; [0, 4]; [4, 5]; [5, 6]; [0, 7]; [7, 8]; [8, 9]; [9, 10]; [8, 14]; [14, 15]; [15, 16]; [8, 11]; [11, 12]; [12, 13]] +1;
    camsize = 0.5;
    az = 93;
    el = 6;
    %view(az, el)
    
%     figure(1)
%     hold on
%     [Xground,Yground] = meshgrid((min(min(X(1:3:end,:))):(max(max(X(1:3:end,:)))-min(min(X(1:3:end,:))))/10:max(max(X(1:3:end,:)))), (min(min(X(2:3:end,:))):(max(max(X(2:3:end,:)))-min(min(X(2:3:end,:))))/10:max(max(X(2:3:end,:)))));
%     Zground = Xground*0+min(min(X(3:3:end,:)));
%     I = imread('floor.jpg');
%     warp(Xground,Yground,Zground,I);
    %colormap([0.95  0.95  0.95])
    xr = 0.04;
    zr = 0.04;
    nellipse = 20;
elseif P == 89
    limbSeq = [1 1];
    camsize = 0.02;    
    R = 2;
    az = 7.2;
    el = -50;

    view(az, el)
    
elseif P == 30
    R = 600;
    limbSeq = [2 1;1 3;1 4;4 5;5 6;3 7;7 8;8 9;1 10;10 11;11 12;3 13;13 14;14 15];%[1  2; 2  3; 3  4; 4  5; 2  6; 6  7; 7  8; 2 15; 15  12; 12  13; 13  14; 15  9; 9  10; 10  11];% headtop, neck,rShoulder, rElbow, rWrist, lShouder, lElbow, lwrist, rhip, rknee, rankle, lhip, lknee, lankle, bodycenter
    limbSeq = [limbSeq;limbSeq+15];
    az = 7.2;
    el = -70;
    view(az, el)
    camsize = 10;
else
    R = 4;
    limbSeq = [1 1];
    camsize = 10;
%     az = 25.3;
%     el = -58.6;
%     view(az, el)
end
%=================color map===============
colors = hsv(length(limbSeq));

for f = 1:F
    tic;
    points_temp = reshape(X(:,f),[3,P]);
    if f == 1
        hold on
        camera = plotCamera('Location', t_sum{f},'Orientation', R_sum{f}, 'Size', camsize);
        points_plot = plot3(points_temp(1,:) , points_temp(2,:),  points_temp(3,:), '.', 'MarkerSize', 1, 'MarkerEdgeColor', 'k'); 
        axis equal;
        axis off;
        axis([min(min(X(1:3:end,:)))-0.7*R max(max(X(1:3:end,:)))+0.7*R min(min(X(2:3:end,:)))-0.7*R max(max(X(2:3:end,:)))+0.7*R min(min(X(3:3:end,:)))-0.7*R max(max(X(3:3:end,:)))+0.7*R]);
              
        num_lines = size(limbSeq,1);
        lines_x = reshape(points_temp(1,limbSeq), [num_lines,2]);
        lines_y = reshape(points_temp(2,limbSeq), [num_lines,2]);
        lines_z = reshape(points_temp(3,limbSeq), [num_lines,2]);
        lines_plot = cell(num_lines,1);
        lines_plot_GT = cell(num_lines,1);
        rays_plot = cell(P,1);
        trajectory = cell(P,1);
        for p = 1:num_lines
            if ellipsoidLimb
                xc = sum(lines_x(p,:))/2;
                yc = sum(lines_y(p,:))/2;
                zc = sum(lines_z(p,:))/2;
                yr = sqrt((lines_x(p,1) - lines_x(p,2))^2+(lines_y(p,1) - lines_y(p,2))^2+(lines_z(p,1) - lines_z(p,2))^2)/2;
                [xepllise, yepllise, zepllise] = ellipsoid(xc,yc,zc,xr,yr,zr,nellipse);
                lines_plot{p} = surf(xepllise, yepllise, zepllise,'FaceColor', colors(p,:),'EdgeColor' ,colors(p,:));% 'r');
                direction = cross([0 1 0], [(lines_x(p,1) - lines_x(p,2)) (lines_y(p,1) - lines_y(p,2)) (lines_z(p,1) - lines_z(p,2))]/(2*yr));
                direction = direction/norm(direction);
                angle = atan(sqrt((lines_x(p,1) - lines_x(p,2))^2+(lines_z(p,1) - lines_z(p,2))^2)/(lines_y(p,1) - lines_y(p,2)))*180/pi;
                rotate(lines_plot{p}, direction, angle, [xc,yc,zc]);
            else
                lines_plot{p} = line(lines_x(p,:), lines_y(p,:), lines_z(p,:), 'Color', 'r', 'LineWidth', 2);
            end
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
            if ellipsoidLimb
                xc = sum(lines_x(p,:))/2;
                yc = sum(lines_y(p,:))/2;
                zc = sum(lines_z(p,:))/2;
                yr = sqrt((lines_x(p,1) - lines_x(p,2))^2+(lines_y(p,1) - lines_y(p,2))^2+(lines_z(p,1) - lines_z(p,2))^2)/2;
                [xepllise, yepllise, zepllise] = ellipsoid(xc,yc,zc,xr,yr,zr,nellipse);
                lines_plot{p}.XData = xepllise;
                lines_plot{p}.YData = yepllise;
                lines_plot{p}.ZData = zepllise;
                direction = cross([0 1 0], [(lines_x(p,1) - lines_x(p,2)) (lines_y(p,1) - lines_y(p,2)) (lines_z(p,1) - lines_z(p,2))]/(2*yr));
                direction = direction/norm(direction);
                angle = atan(sqrt((lines_x(p,1) - lines_x(p,2))^2+(lines_z(p,1) - lines_z(p,2))^2)/(lines_y(p,1) - lines_y(p,2)))*180/pi;
                rotate(lines_plot{p}, direction, angle, [xc,yc,zc]);
            else
                lines_plot{p}.XData = lines_x(p,:);
                lines_plot{p}.YData = lines_y(p,:);
                lines_plot{p}.ZData = lines_z(p,:);
            end
        end
        if ray_dis == 1
            for p = 1:P
                rays_plot{p}.XData = [t_sum{f}(1) ray_sum{f}(1,p)*R+t_sum{f}(1)];
                rays_plot{p}.YData = [t_sum{f}(2) ray_sum{f}(2,p)*R+t_sum{f}(2)];
                rays_plot{p}.ZData = [t_sum{f}(3) ray_sum{f}(3,p)*R+t_sum{f}(3)];
            end
        end
        for p = 1:P
            trajectory{p} = plot3(X(p*3-2,f-1:f),X(p*3-1,f-1:f),X(p*3,f-1:f),'-','Color', 'b', 'MarkerSize', 0.1,'LineWidth', 0.05);
            trajectory{p}.Color(4) = 1*f/F;
        end
        %view(az,el);
    end
    
% %     %==================save image===============================
%      filesname = sprintf('ICCV oral presentation/climb/3Dpose4/pose%03d.jpg', f);
%      saveas(gcf, filesname)
% %     %============================================================
    
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
%delete(camera)
end