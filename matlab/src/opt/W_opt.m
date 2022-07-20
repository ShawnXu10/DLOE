function W = W_opt( X, fedler_X, WL, cam_index,order,param)
%W_OPT_FLF Summary of this function goes here
%   Detailed explanation goes here
%   Optimize cost function over D
% min ||Ｄ(I-W)X||^2 + lambda1*sum(D_ii*W_ij*dis(Xi,Xj)^2) + lambda3*sum((D_ii*W_ij*(r_i*r_j))^2)
% where sum(diag(D)) = F, W = Wvar＋Wconst
    addpath(genpath('spams-matlab'))
    const1 = 1 -param.Wprior;
    lambda1 = param.lambda1;
    lambda3 = param.lambda3;
    RayConv = param.RayConv;
    


    if exist('order')
    else
        order = false;
    end

    frames = size(X,1);
    cam_index = [0 cam_index];
    W_temp = cell(frames,1);
    

    
    %======================Wconst and Xconst===============================
    if const1~=1
        Wconst = getWconst( cam_index )';
    else
        Wconst = zeros(frames,frames);
    end
    XWconst = Wconst*X;
    XWconst(isnan(XWconst)) = 0;
    X_XWconst = X - XWconst*(1-const1);
    %======================================================================

    paramAS.epsilon = 1e-2;%param for solver
    paramAS.lambda2 = 1e-2;
    indexNanTmp = find(isnan(fedler_X));
    if isempty(indexNanTmp)
        distsq = pdist2(fedler_X',fedler_X').^2;
    else
        distsq = zeros(frames,frames);
    end
    
    for i = 1:numel(cam_index)-1
            zeroRange = cam_index(i)+1:cam_index(i+1); %the entries in W corresponding to the same camera will be assigned as 0
        parfor f = zeroRange
            %===========if we know the eaxtly order======= need to clean up
            if order
                nonzeroRange =  index_order(order, f, zeroRange);
            else
            %======================================================================================
                nonzeroRange = 1:frames;
                nonzeroRange(zeroRange) = [];% the index of non-zeros entries in W
            end
            
            %Ｂ and A　ｃorresponing the first term
            A1 = X_XWconst(f,:)*WL(f,f);
            index_nan = find(isnan(A1));
            A1(index_nan) = [];
            B = X(nonzeroRange,:)*const1*WL(f,f);
            B(:,index_nan) = [];
            B(isnan(B)) = 0;

            %A　ｃorresponing the second term
            A2 = -0.5*(B\(const1*lambda1*WL(f,f)*distsq(nonzeroRange,f)));
            A = A1+A2';

            %%Ｂ and A　ｃorresponing the third term
            if lambda3 ~= 0
                B2 = abs(RayConv(:,f))*WL(f,f);
                B = [B sqrt(param.lambda3)*B2(nonzeroRange)*const1];
                A = [A -sqrt(param.lambda3)*sparse(Wconst(f,:))*B2*(1-const1)];
            end

            W_temp{f} = zeros(1,frames);
            W_temp{f}(nonzeroRange) = mexActiveSet(B', A', paramAS);

        end
    end
    W_temp = cell2mat(W_temp);
    W = const1*W_temp+(1-const1)*Wconst;
end

function nonzeroRange = index_order(order, f, zeroRange)
frames = length(order);
if find(order==f)== 1 %if it's the first frame
    fnext = 1;
    while(1)
        if ismember(order(find(order==f)+fnext),zeroRange)%if the next adjacent frame is in the same camera
            fnext = fnext+1;
        else
            break;
        end
    end
    nonzeroRange = order(find(order==f)+fnext);
elseif find(order==f) == frames %if it's the last frame
    fprev = 1;
    while(1)
        if ismember(order(find(order==f)-fprev),zeroRange)%if the last adjacent frame is in the same camera
            fprev = fprev+1;
        else
            break;
        end
    end
    nonzeroRange = order(find(order==f)-fprev);
else %if it's the frame in the middle
    fnext = 1;
    while(1)
        if (find(order==f)+fnext) == frames+1%if no next adjacent frame is in the different camera
            fnext = [];
            break;
        elseif ismember(order(find(order==f)+fnext),zeroRange)
            fnext = fnext+1;
        else
            break;
        end
    end
    fprev = 1;
    while(1)
        if (find(order==f)-fprev) == 1-1%if no prev adjacent frame is in the different camera
            fprev = [];
            break;
        elseif ismember(order(find(order==f)-fprev),zeroRange)
            fprev = fprev+1;
        else
            break;
        end
    end
    if isempty(fprev)&&isempty(fnext)
        warning('Not enough frames to optimize');
    elseif isempty(fprev)&&~isempty(fnext)
        nonzeroRange = order(find(order==f)+fnext);
    elseif ~isempty(fprev)&&isempty(fnext)
        nonzeroRange = order(find(order==f)-fprev);
    elseif ~isempty(fprev)&&~isempty(fnext)
         nonzeroRange = [order(find(order==f)-fprev) order(find(order==f)+fnext)];
    end                    
end
end