function dist = pdistArc( X, cam_index )
%PDISTARC Summary of this function goes here
%   Detailed explanation goes here
n_cam = numel(cam_index)-1;
dist = pdist2(X',X');
dist_temp = dist;
%============dist between frames in the same camera========================
for n=1:n_cam
    dist_cam = diag(dist_temp(cam_index(n)+1:cam_index(n+1), cam_index(n)+1:cam_index(n+1)),-1);
    %dist_cam_inter = sum(dist_cam)/numel(dist_cam);

    for f = cam_index(n)+1:cam_index(n+1)
        if f< cam_index(n+1)&&f>cam_index(n)+1
            C = X(:,f);
            A = X(:,f-1);
            B = X(:,f+1);

        end
        dist(f+1:cam_index(n+1),f) = cumsum(dist_cam(f-cam_index(n):end));
        dist(f,f+1:cam_index(n+1)) = dist(f+1:cam_index(n+1),f)';
        dist_temp(f+1:cam_index(n+1),f) = dist(f+1:cam_index(n+1),f);%maybe useless
        dist_temp(f,f+1:cam_index(n+1)) = dist(f,f+1:cam_index(n+1));%maybe useless
    end
end
%==========================================================================


%============dist between frames between the different camera==============
for n=1:n_cam
    for nn = n+1:n_cam
        X_cam1 = X(:,cam_index(n)+1:cam_index(n+1));
        X_cam2 = X(:,cam_index(nn)+1:cam_index(nn+1));
        [~, index] = DTWArc( X_cam1, X_cam2);
        
        indexfn = [cam_index(n)+1:cam_index(n+1) cam_index(n+1)+1];
        indexfn = indexfn(index);
        for f = cam_index(n)+1:cam_index(n+1)
            nf = 1;%n-th frame in cam_index(nn)
            for fn = cam_index(nn)+1:cam_index(nn+1)
                if f >= indexfn(nf) %f is in the front of nf
                    dist(f,fn) = norm(X(:,fn)-X(:,indexfn(nf)))+dist(indexfn(nf), f);
                    dist(fn,f) = dist(f,fn);
                else% f is on the back of nf
                    dist(f,fn) = norm(X(:,fn)-X(:,indexfn(nf)-1))+dist(indexfn(nf)-1, f);
                    dist(fn,f) = dist(f,fn);
                end
                nf = nf + 1;
            end
        end
    end
end
%==========================================================================

end

