function [ X_output, W_output, D_output ] = Triconvex_opt( X_init, ray_sum, t_sum, cam_index,param, order)
%BICONVEX_OPT Summary of this function goes here
%   Detailed explanation goes here
%   Optimize cost function over W, D, X
% min ||D(I-W)X||^2 + lemma1*sum(D_ii*W_ij*dis(Xi,Xj)^2) + lemma2*dist(X,ray) + lemma3*sum((D_ii*W_ij*(r_i*r_j))^2)
    
    if exist('order')
    else
        order = false;
    end

    frames = size(ray_sum, 1);
    joints = size(ray_sum{1}, 2);
    X{1} = X_init;
    D{1} = eye(frames);
    n_opt = 0;

    %======================ray convergence=================================
    RayConv = eye(frames,frames);
    for f1 = 1:frames
        for f2 = f1+1:frames
            [~,index_valid1] = find(isnan(ray_sum{f1}(1,:))~=1);
            [~,index_valid2] = find(isnan(ray_sum{f2}(1,:))~=1);
            index_valid = intersect(index_valid1,index_valid2);
            if isempty(index_valid)
                RayConv(f1,f2) = 1;
            else
                RayConv(f1,f2) = sum(sum(ray_sum{f1}(:,index_valid).*ray_sum{f2}(:,index_valid)))/length(index_valid);
            end
            RayConv(f2,f1) = RayConv(f1,f2);
        end
    end
    param.RayConv = RayConv;
    %======================================================================
    while(1)
        n_opt = n_opt +1;
        if strcmp(param.f,'X')
            fXX = X{n_opt}';
        elseif strcmp(param.f,'fiedler')
            param_lap.dist = 1;
            param_lap.Ln = 'Simple';
            param_lap.cam_index = cam_index;
            param_lap.t = 1;
            param_lap.dist_criter = 1.5;
            param_lap.dist_Type = param.dist_Type;
            param_lap.Seriation = param.Seriation; %'MDS'; %'Laplacian';
            param_lap.DMtype = 'inv';
            param_lap.graph = 'undirected';
            param_lap.Version = 2;
            if ~isempty(find(isnan(X{n_opt})))
                fXX = zeros(1,frames);
            else
                [~, ~,  eigparam_X] = SequenceDReduce( X{n_opt}', param_lap );
                fXX = eigparam_X.EmbedV';
            end
        end   
        
        %==========step 1, optimize over W with fixed X and Q==============
        
        if n_opt == 1&&(~isempty(find(isnan(fXX))))
            paramtmp = param;
            paramtmp.lemma1 = 0;
            W{n_opt} = W_opt(X{n_opt}, fXX,D{n_opt}, cam_index,order,paramtmp);
        else
            W{n_opt} = W_opt(X{n_opt}, fXX, D{n_opt}, cam_index, order,param);
        end
        
        %LossF( X{n_opt}, X{n_opt}', W{n_opt},D{n_opt}, t_sum, ray_sum, RayConv, param)

        %==========step 2, optimize over Q with fixed X and W==============
        if n_opt == 1&&(~isempty(find(isnan(X{n_opt}))))
            paramtmp = param;
            paramtmp.lemma1 = 0;            
            D{n_opt+1} = D_opt( X{n_opt},W{n_opt}, paramtmp);
            
        else
            D{n_opt+1} = D_opt( X{n_opt},W{n_opt}, param );
        end
        
        %LossF( X{n_opt}, X{n_opt}', W{n_opt},D{n_opt+1}, t_sum, ray_sum,RayConv, param)

        %==========step 3, optimize over X with fixed W and Q==============
        X{n_opt+1} = X_opt( D{n_opt+1}*W{n_opt}, ray_sum, t_sum, param);        
        %LossF( X{n_opt+1}, X{n_opt+1}', W{n_opt},D{n_opt+1}, t_sum, ray_sum, RayConv, param)
        
        %==========step 4, calculate the difference between X==========
        Lossf(n_opt) = norm(X{n_opt+1}-X{n_opt},'fro')/(frames*joints);
        
        if (Lossf(n_opt) < param.thres||n_opt >param.itermax)
            if strcmp(param.f,'fiedler')
                [Lossftmp,index_Lossf] = min(Lossf);
                X_output = X{index_Lossf+1};
                W_output = W{index_Lossf};
                D_output = D{index_Lossf+1};
                %disp(['Final loss:', num2str(Lossftmp)])
            else
                X_output = X{n_opt+1};
                W_output = W{n_opt};
                D_output = D{n_opt+1};
            end
            if param.Convdisp
              disp(['Final loss:', num2str(Lossf)]);
            end
            break;
        end
        if param.Convdisp
            disp(['Final loss:', num2str(Lossf)]);
        end
    end
end