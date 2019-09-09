function [D, t] = D_opt( X,W,param)
%LAPLACIANOW Summary of this function goes here
%   Optimize cost function over D
% min ||(D*const2 + (1-cost2))WX||^2 + lemma2*sum(D_ii*W_ij*dis(Xi,Xj)^2) + lemma4*sum((D_ii*W_ij*(r_i*r_j))^2)

lemma4 = param.lemma4;
const2 = param.const2;
lemma1 = param.lemma1;
RayConv = param.RayConv;
F = size(X,2);

if const2 == 0
    D = eye(F);
else
    %=================comput H and ft from first term =====================
    X(isnan(X)) = 0; % if there are missing joints
    XW = X*(diag(sum(W))-W);
    Btmp = XW*F*const2;
    H = diag(sum(Btmp.^2));

    Atmp = XW*(1-const2);
    fT = +2*(sum(Btmp.*Atmp));
    %======================================================================
    
    if isempty(find(isnan(X),1)) % in case there are missing joints
        distsq = pdist2(X',X').^2;
    else
        distsq = zeros(F,F);
    end
    
    if lemma1 == 0 % if delete first term
        tic;
        [~,index_dist] = min(sum(W.*distsq));
        D = zeros(F,1);
        D(index_dist) = 1;
        t= toc;
    else
        %======================fT from second term ========================
        fT = fT + lemma4*sum(W*F*const2.*distsq);
        %==================================================================   
        
        
        %================H and ft term from R(A,theta) term================
        if  param.lemma5 ~=0
            RTRW = RayConv.*W;
            Btmp = sqrt(param.lemma5)*RTRW*F*const2;
            H = H + diag(sum(Btmp.^2));

            Atmp = sqrt(param.lemma5)*RTRW*(1-const2);
            fT = fT +2*(sum(Btmp.*Atmp));            
        end
        %==================================================================

 
%          H = diag(sum(([XL*F*const2;(sqrt(param.lemma5)*RTRW*F*const2)]).^2));
%          fT = -2*(sum((XL*F*const2).*(XL*(1-const2))) + param.lemma5*sum((RTRW*F*const2).*(RTRW*(1-const2))))+lemma4*sum(W*F*const2.*distsq);

        %=====================matlab built-in solver ==============================
        Aeq = ones(1,length(fT));
        beq = 1;
        lb = zeros(1,length(fT));
        ub = ones(1,length(fT));
        
        options = optimoptions(@quadprog, 'Algorithm', 'interior-point-convex','Display','off');
        D = quadprog(2*H,fT,[],[],Aeq,beq,lb,ub,[],options);
        D = diag(D)*const2*F+(1-const2)*eye(F);
        %==========================================================================
    end
end
end

