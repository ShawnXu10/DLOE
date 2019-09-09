function [ X_init, d_init, X_initorder, D_d, D_X ] = X_initial( ray_sum, t_sum, cam_index, order )
%Generate the initial X for biconvex the function
%   Detailed explanation goes here

cam_index = [0 cam_index];
frames = size(ray_sum, 1);
joints = size(ray_sum{1}, 2);
Dist_ray = cell(frames);%pairwise distiance of each ray between each two frames normalized by depth
D = zeros(frames, frames); % sum of pairwise distiance of each ray between each two frames
D_X = cell(frames, frames); % 3D points for every joint consistent to D (D_X(i,j), all the point of frame i to the frame j )
D_d = cell(frames, frames);% depth of each ray in each frame

X_init = nan(frames,3*joints);
d_init = nan(joints, frames);
X_initorder = zeros(3*joints, frames);
d_initorder = zeros(joints, frames);

% ====calcalute sum of parwise distance of rays btween each two frames=====
for n = 1:(length(cam_index)-1) % for each camera
    D((cam_index(n)+1):cam_index(n+1), (cam_index(n)+1):cam_index(n+1)) = inf; % 
    for i = (cam_index(n)+1):cam_index(n+1) % for all the frames in n-th camera
        t1 = t_sum{i};
        for j = cam_index(n+1)+1:frames % for all the frames in the all cameras except n-th camera
            t2 = t_sum{j};
            D_X{i, j} = zeros(3, joints);
            D_d{i, j} = zeros(joints, 1);
            D_X{j, i} = zeros(3, joints);
            D_d{j, i} = zeros(joints, 1);
            Dist_ray{i,j} = zeros(joints, 1);
            Dist_ray{j,i} = zeros(joints, 1);
            n_valid_joints = 0;
            for p = 1:joints % for all the key points
                r1 = ray_sum{i}(:, p);
                r2 = ray_sum{j}(:, p);
                if isnan(r1(1)) || isnan(r2(1)) % set the value to nan if the key point is missing
                    D_X{i, j}(:, p) = nan;
                    D_d{i, j}(p, :) = nan;
                    Dist_ray{i, j}(p, :) = nan;
                    D_X{j,i}(:, p) = nan;
                    D_d{j,i}(p, :) = nan;
                    Dist_ray{j,i}(p, :) = nan;
                else
                    % calculate the shortest distance between two viewing ray and the corresponding 3D points
                    [ d1, d2, c1_2, c2_1 ] = ray_interact( r1, r2, t1', t2' ); 
                    if d1 < 0 || d2 < 0 %if any depth is negtive, break out
                        break;
                    end
                    D_X{i, j}(:, p) = c1_2;
                    D_d{i, j}(p, :) = d1;
                    D_X{j, i}(:, p) = c2_1;
                    D_d{j, i}(p, :) = d2;
                    
                    Dist_ray{i, j}(p, :) = norm(c1_2 - c2_1)/(d1+d2);
                    Dist_ray{j, i}(p, :) = Dist_ray{i, j}(p, :);
                    D(i, j) = D(i, j) + Dist_ray{i, j}(p, :);                   
                    n_valid_joints = n_valid_joints + 1;
                end
            end
            D(i, j) = D(i, j)/n_valid_joints;
            D(j, i) = D(i,j);
        end
    end
end
% =========================================================================

% =====================need to be cleaned up and add some comments=========
if exist('order')
    Dorder = D(order,order);
    D_Xorder = D_X(order,order);
    %[~, index_2] = min(D_2');
    D_dorder = D_d(order,order);
    
    for m = 1:frames
        for p = 1:joints
            if m == 1
                mnext = 1;
                while(1)
                    if isempty(D_Xorder{m,m+mnext})
                        mnext = mnext+1;
                    elseif mnext > 10
                        break;                        
                    elseif isnan(D_Xorder{m,m+mnext}(1,p))
                        mnext = mnext+1;

                    else
                        break;
                    end
                end

                Xorder = D_Xorder{m,m+mnext};
                X_initorder(:, m) = reshape(Xorder,[3*joints,1]);
                d_initorder(:, m) = D_dorder{m,m+mnext};
            elseif m == frames
                mprev = 1;
                while(1)
                    if isempty(D_Xorder{m,m-mprev})
                        mprev = mprev+1;
                    elseif mprev > 10
                        break;                        
                    elseif isnan(D_Xorder{m,m-mprev}(1,p))
                        mprev = mprev+1;

                    else
                        break;
                    end
                end
                Xorder = D_Xorder{m,m-mprev};
                X_initorder(:, m) = reshape(Xorder,[3*joints,1]);
                d_initorder(:, m) = D_dorder{m,m-mprev};            
            else
                mnext = 1;
                while(1)
                    if (m+mnext) == frames+1
                        mnext = [];
                        break;
                    elseif isempty(D_Xorder{m,m+mnext})%if it's from same camera
                        mnext = mnext+1;
                    elseif mnext > 10
                        break;                        
                    elseif isnan(D_Xorder{m,m+mnext}(1,p))%if it's a occlusion
                        mnext = mnext+1;
                    else
                        break;
                    end
                end
                mprev = 1;
                while(1)
                    if (m-mprev) == 1-1
                        mprev = [];
                        break;
                    elseif isempty(D_Xorder{m,m-mprev})
                        mprev = mprev+1;
                    elseif mprev > 10
                        break;                        
                    elseif isnan(D_Xorder{m,m-mprev}(1,p))
                        mprev = mprev+1;
                    else
                        break;
                    end
                end
                if isempty(mprev)&&isempty(mnext)
                    warning('Not enough frames to triangulation');
                elseif isempty(mprev)&&~isempty(mnext)
                        Xorder = D_Xorder{m,m+mnext}(:,p);
                        d_initorder(:, m) = D_dorder{m,m+mnext};                
                elseif ~isempty(mprev)&&isempty(mnext)
                        Xorder = D_Xorder{m,m-mprev}(:,p);
                        d_initorder(:, m) = D_dorder{m,m-mprev};
                elseif ~isempty(mprev)&&~isempty(mnext)
                    [~,indexorder] = min(Dorder(m,[m-mprev m+mnext]));
                    if indexorder == 1
                        Xorder = D_Xorder{m,m-mprev}(:,p);
                        d_initorder(:, m) = D_dorder{m,m-mprev};
                    elseif indexorder == 2
                        Xorder = D_Xorder{m,m+mnext}(:,p);
                        d_initorder(:, m) = D_dorder{m,m+mnext};
                    else
                        warning('Something with X_initial')
                    end
                end
                X_initorder(3*p-2:3*p, m) = Xorder;
                %d_initorder(:, m) = D_dorder{m,m+indexorder-2};            
            end
        end
    end
end

%find minimal cost for each frame
[~, index] = sort(D,2);

%==========for a ray if the corresponding ray in the nearest frame is 
%missing, try to find the next nearest ray in the other frames============%
for m = 1:frames
    for p = 1:joints
        for indexvalid = 1:min(10,frames)
            %indexvalid = 1;
            if isempty(D_X{m,index(m,indexvalid)})
                continue;
            elseif isnan(D_X{m,index(m,indexvalid)}(1,p))
                continue;
            else
                X_init(m,3*p-2:3*p) = D_X{m,index(m,indexvalid)}(:,p);
                d_init(p, m) = D_d{m,index(m,indexvalid)}(p);%D_d{index(m), m};
                break;
            end
        end
    end
end
%==========================================================================

end

