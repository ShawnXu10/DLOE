function [D, t] = D_opt( X,W,param)
%LAPLACIANOW Summary of this function goes here
%   Optimize cost function over D
% min ||D(I-W)X||^2 + lemma1*sum(D_ii*W_ij*dis(Xi,Xj)^2) + lemma3*sum((D_ii*W_ij*(r_i*r_j))^2)
% where sum(diag(D)) = F, D　＝　(Dvar*const2 + (1-cost2))

const2 = param.const2;
lemma1 = param.lemma1;
lemma3 = param.lemma3;
RayConv = param.RayConv;
F = size(X,1);
P = size(X,2);

if const2 == 0
    D = eye(F);
else
    %=================comput H and ft from first term =====================
    X(isnan(X)) = 0; % if there are missing joints
    IWX = (diag(sum(W,2))-W)*X; % compute (I-W)X
    Btmp = IWX*const2;
    H = diag(sum(Btmp.^2,2));

    Atmp = IWX*(1-const2);
    fT = +2*(sum(Btmp.*Atmp,2));
    %======================================================================
    
    if isempty(find(isnan(X),1)) % in case there are missing joints
        distsq = pdist2(X,X).^2;
    else
        distsq = zeros(F,F);
    end

    %======================fT from second term ========================
    fT = fT + lemma1*sum(W*const2.*distsq,2);
    %==================================================================   


    %================H and ft term from third term================
    if  lemma3 ~=0
        RTRW = RayConv.*W;
        Btmp = sqrt(lemma3)*RTRW*const2;
        H = H + diag(sum(Btmp.^2,2));

        Atmp = sqrt(lemma3)*RTRW*(1-const2);
        fT = fT +2*(sum(Btmp.*Atmp,2));            
    end
    %==================================================================

    %=====================matlab built-in solver ==============================
    Aeq = ones(1,length(fT));
    beq = F;
    lb = zeros(1,length(fT));
    ub = F*ones(1,length(fT));

    options = optimoptions(@quadprog, 'Algorithm', 'interior-point-convex','Display','off');
    D = quadprog(2*H,fT,[],[],Aeq,beq,lb,ub,[],options);
    D = diag(D)*const2+(1-const2)*eye(F);
    %==========================================================================
end
end

