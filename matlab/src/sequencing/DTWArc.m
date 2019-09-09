function [condition, index ] = DTWArc( X_cam1,X_cam2)
%FIND_MATCHING_NODE Summary of this function goes here
%   Detailed explanation goes here

% %================DTW==========================
n_cam1 = size(X_cam1, 2);
n_cam2 = size(X_cam2, 2);
dist = inf(n_cam2,n_cam1+1);
%=================compute pairwise distance matrix=========================
for f = 1:n_cam2
    dist(f,1) = norm(X_cam2(:,f)-X_cam1(:,1));
    C = X_cam2(:,f);
    for fn = 2:n_cam1
        A = X_cam1(:,fn-1);
        B = X_cam1(:,fn);
        proj_tmp = dot(C-A,B-A);
        if proj_tmp >0&& proj_tmp<=norm(B-A)^2
            dist(f,fn) = norm(C - A - (C-A)'*(B-A)/norm(B-A)^2*(B-A));
%             if dist(f,fn) > 5*dist_threshold1
%                 dist(f,fn) = inf;
%             end
        else
            dist(f,fn) = inf;
        end
    end
    dist(f,end) = norm(X_cam2(:,f)-X_cam1(:,end));
end
%==========================================================================

%====================use dp find shortest path=============================
D = zeros(n_cam2,n_cam1+1);
D(:,1) = cumsum(dist(:,1));
D(1,:) = dist(1,:);
D(1,1) = dist(1,1);
for m = 2:n_cam2
    for n = 2:n_cam1+1
        D(m,n) = dist(m,n) + min(D(m-1,1:n));
    end
end
%==========================================================================

%=====================compute sequence of the path=========================
index = [zeros(1, n_cam2) n_cam1+1];
for n = n_cam2:-1:1
    [~, index_tmp] = min(D(n,1:index(n+1)));
    index(n) = index_tmp;
end
index = index(1:end-1);
%==========================================================================


if all(index==1)&&all(index==(n_cam1+1))
    condition = 5;
    warning('Two cameras are not connected')
elseif index(1) == 1
    if index(end) == n_cam1+1
        condition = 4;
    else
        condition = 1;
    end
else
    if index(end) == n_cam1+1
        condition = 3;
    else
        condition = 2;
    end
end
end


