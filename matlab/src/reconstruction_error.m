function [ error, mean_error, std_error ] = reconstruction_error( X_GT, X )
%Calculate reconstruction error: X_GT - X
%   Input: coefficient error, 3d points X
%   Output: error for each point, mean of error, std of each error
% [x_nan1, y_nan1] = find(isnan(X)==1);
% [x_nan2, y_nan2] = find(isnan(X_GT));
% X_GT([x_nan1;x_nan2],[y_nan1;y_nan2]) = 0;
% X([x_nan1;x_nan2],[y_nan1;y_nan2]) = 0;
% error = X - X_GT;
% row = 3*ones(1, size(X, 1)/3);
% error = mat2cell(error, row)';
% error = cell2mat(error);
% error = sqrt(diag(error'*error));
% mean_error = mean(error);
% std_error = std(error);
frames = size(X_GT,2);
joints = size(X_GT,1)/3;
error = [];%zeros(frames*joints,1);
for f = 1:frames
    for p = 1:joints
        if ~isnan(X(3*p-2,f))
            error = [error norm(X_GT(3*p-2:3*p,f) - X(3*p-2:3*p,f))];%error(joints*(f-1)+p) = norm(X_GT(3*p-2:3*p,f) - X(3*p-2:3*p,f));
        end
    end
end
 mean_error = mean(error);
 std_error = std(error);
end