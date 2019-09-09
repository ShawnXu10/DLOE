function Wconst = getWconst( cam_index )
%GETWCONST Summary of this function goes here
%   Detailed explanation goes here
n_cam = length(cam_index)-1;
%cam_index = [0 cam_index];
F = cam_index(end);
Wconst = zeros(F,F);
Wconst(2:end,1:end-1) = Wconst(2:end,1:end-1) + 0.5*eye(F-1);
Wconst(1:end-1,2:end) = Wconst(1:end-1,2:end) + 0.5*eye(F-1);
for n = 1:n_cam
    Wconst(cam_index(n)+2, cam_index(n)+1) = 1;
    Wconst(cam_index(n+1)-1, cam_index(n+1)) = 1;
end
for n = 1:n_cam-1
    Wconst(cam_index(n+1)+1, cam_index(n+1)) = 0;
    Wconst(cam_index(n+1), cam_index(n+1)+1) = 0;
end