function [f_node, sequence, eigparam ] = SequenceDReduce( varargin )
%SEQUENCE_LAPLACIAN Summary of this function goes here
%   Detailed explanation goes here
%global L dist
X = varargin{1};
param = varargin{2};


n_cam = numel(param.cam_index);
param.cam_index = [0 param.cam_index];

F = size(X,2);
%P = size(X,1);
A = zeros(F,F);
if strcmp(param.dist_Type,'Euc')
    dist = pdist2(X',X');
    A = dist+diag(inf(F,1));
    A(find(A==0)) = min(min(W(find(A>0))));
    A = 1./A; 
elseif strcmp(param.dist_Type,'Arc')        
    dist = pdistArc( X, param.cam_index);
    A = dist+diag(inf(F,1));
    A(find(A==0)) = min(min(A(find(A>0))));
    A = 1./A;
end
eigparam.dist = dist;


if strcmp(param.Seriation, 'SpRank')
    eigparam.adj = A;
    D = diag(sum(A));

    if strcmp(param.Ln,'Simple')
        L = D - A;
    elseif strcmp(param.Ln,'Normalized')
        L = eye(F) - D^0.5*A*D^0.5;
    elseif strcmp(param.Ln,'Generalized')
        L = eye(F) - D^(-1)*A;
    else
        error('We dont have this laplacian yet');
    end
    eigparam.L = L;
    [V, D, ~] = eig(L);
    [D, index_sort] = sort(diag(D));
    V = V(:,index_sort);
    index_D = min(find(D>1e-5));
    eigparam.fedler = V(:,index_D);
    [f_node, sequence ] = sort(eigparam.fedler);
    D = diag(D);
    
    eigparam.V = V;
    eigparam.D = D;
elseif strcmp(param.Seriation,'MDS')
    J = diag(ones(F,1)) - ones(F,1)*ones(1,F)/F;
    B = -0.5*J*(dist.^2)*J;
    [V, D, ~] = eig(B);
    eigparam.EmbedV = V(:,1)*sqrt(abs(D(1)));
    [f_node,sequence] = sort(eigparam.EmbedV);

end

% flip if the globle sequence is reversed
index1 = find(sequence == (param.cam_index(1)+1));
index2 = find(sequence == param.cam_index(2));
if index2 < index1
    sequence = flipud(sequence);
end