function [f_node, sequence, eigparam ] = SequenceDReduce( varargin )
%SEQUENCE_LAPLACIAN Summary of this function goes here
%   Detailed explanation goes herep: param.dist(1,0), param.Ln('Simple', 'Normalized', 'Generalized')
%global L dist

if nargin == 2
    W = varargin{1};
    param = varargin{2};
elseif nargin == 3%X and W combination
    W = varargin{1};
    W_1 = varargin{2};
    param = varargin{3};
end

n_cam = numel(param.cam_index);
param.cam_index = [0 param.cam_index];
if param.dist == 1
    X = W;
    F = size(X,2);
    %P = size(X,1);
    W = zeros(F,F);
    if strcmp(param.dist_Type,'Space')
        if strcmp(param.Seriation,'Laplacian')
            dist = pdist2(X',X');
            if strcmp(param.DMtype, 'inv')
%                 for n=1:n_cam
%                     dist(param.cam_index(n)+1:param.cam_index(n+1), param.cam_index(n)+1:param.cam_index(n+1)) = inf;
%                 end
                W = dist+diag(inf(F,1));
                W(find(W==0)) = min(min(W(find(W>0))));% It's a temp way to fix this
                W = 1./W;
                %W = inv(dist);
            elseif strcmp(param.DMtype, 'neg')
                W = -dist.^2;
            end
        elseif strcmp(param.Seriation,'MDS')
            dist = pdist2(X',X');
        end
    elseif strcmp(param.dist_Type,'Time')        
        %param.Version = 2;
        dist = pdistArc( X, param.cam_index, param);%distance between frames in different cam
        W = dist+diag(inf(F,1));
        W(find(W==0)) = min(min(W(find(W>0))));% It's a temp way to fix this
        W = 1./W;
    elseif strcmp(param.dist_Type,'Isomap')
        dist = inf(F,F);
        [distID, dist_temp] = knnsearch(X',X', 'k', param.knn);
        for f = 1:F
            dist(distID(f,:),f) = dist_temp(f,:)';
        end
        for k = 1:F
            for f = 1:f
                for ft = f+1:F
                    dist(f,ft) = min(dist(f,ft), dist(f,k)+dist(k,ft));
                    dist(ft,f) = dist(f,ft);
                end
            end
        end
        W = dist+diag(inf(F,1));
        W(find(W==0)) = min(min(W(find(W>0))));% It's a temp way to fix this
        W = 1./W;
    end
    eigparam.dist = dist;
    
    if nargin == 3
        A = W + (W_1 +W_1');
    elseif nargin == 2
        A = W;
    end
elseif param.dist == 0
    F = size(W,1);
    if strcmp(param.graph, 'directed')
        A = W';
    elseif strcmp(param.graph, 'undirected')
        A = W+W';
    end
end
if strcmp(param.Seriation, 'Laplacian')
    eigparam.adj = A;
    D = diag(sum(A));
    if strcmp(param.graph, 'directed')
        if strcmp(param.Ln,'Simple')
            L = diag(sum(A')) - A';
        elseif strcmp(param.Ln,'Prob')
            P_W = A;
            [~,~,WW] = eig(P_W);
            Phi = WW(:,1);
            Phi_cap = diag(Phi);
            L = eye(F)-(Phi_cap^0.5*P_W*Phi_cap^-0.5+Phi_cap^-0.5*P_W'*Phi_cap^0.5)/2;
        elseif strcmp(param.Ln,'Comb')
            %P_W = A;
            alpha = 1;
            P_W = ProbMtrxGen( A,alpha);
            [~,~,WW] = eig(P_W);
            Phi = abs(WW(:,1));
            Phi_cap = diag(Phi);
            L = Phi_cap-(Phi_cap*P_W+P_W'*Phi_cap)/2;
        end
    elseif strcmp(param.graph, 'undirected')
        if strcmp(param.Ln,'Simple')
            L = D - A;
        elseif strcmp(param.Ln,'Normalized')
            L = eye(F) - D^0.5*A*D^0.5;
        elseif strcmp(param.Ln,'Generalized')
            L = eye(F) - D^(-1)*A;
        else
            error('We dont have this laplacian yet');
        end
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
    %dist = pdist2(X',X');
    J = diag(ones(F,1)) - ones(F,1)*ones(1,F)/F;
    B = -0.5*J*(dist.^2)*J;
    [V, D, W] = eig(B);
    eigparam.EmbedV = V(:,1)*sqrt(abs(D(1)));
    [f_node,sequence] = sort(eigparam.EmbedV);
elseif strcmp(param.Seriation,'LLE')
    
end