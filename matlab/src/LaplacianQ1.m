function [WL, t] = LaplacianQ1( X,W,param)
%LAPLACIANOW Summary of this function goes here
%   Detailed explanation goes here
lemma4 = param.lemma4;
const2 = param.const2;
lemma1 = param.lemma1;
RayConv = param.RayConv;

if const2 == 0
    F = size(X,2);
    WL = eye(F);
else
    
    F = size(X,2);
    P3 =size(X,1);
    WL = zeros(F,1);
    paramAS.epsilon = 1e-8;
    paramAS.lambda2 = 1e-8;

    

    L = diag(sum(W))-W;
    fedler_X = X';
    X(isnan(X)) = 0;
    XL = X*L;
    B = zeros(P3*F,F);
    Btmp = XL*F*const2;
    for f=1:F
        B(((f-1)*P3+1):f*P3,f) = Btmp(:,f);
    end

    A = zeros(P3*F,F);
    Atmp = XL*F*(1-const2);
    for f=1:F
        A(((f-1)*P3+1):f*P3,f) = Atmp(:,f);
    end
    A = A*diag(eye(F)/F);
    
    indexNanTmp = find(isnan(fedler_X));
    if isempty(indexNanTmp)
        distsq = pdist2(fedler_X,fedler_X).^2;
    else
        distsq = zeros(F,F);
    end
    if lemma1 == 0 %delete first term
        tic;
        [~,index_dist] = min(sum(W.*distsq));
        WL = zeros(F,1);
        WL(index_dist) = 1;
        t= toc;
    else
        %============================second version ===========================
        A1 = -0.5*(B'\(lemma4*sum(W*F*const2.*distsq)'));
        A = A + A1;
        %======================================================================        
        
        
        %R(A,theta) term
        if  param.lemma5 ~=0
            RTRW = RayConv.*W;
            B1 = zeros(F*F,F);
            B1tmp = RTRW*F*const2;
            for f=1:F
                B1(((f-1)*F+1):f*F,f) = B1tmp(:,f);
            end

            A2 = zeros(F*F,F);
            A2tmp = RTRW*F*(1-const2);
            for f=1:F
                A2(((f-1)*F+1):f*F,f) = A2tmp(:,f);
            end
            A2 = A2*diag(eye(F)/F);
            
            B = [B;sqrt(param.lemma5)*B1];
            A = [A;sqrt(param.lemma5)*A2];
            
        end



        tic;
        WL = mexActiveSet(B, A, paramAS);
        WL = diag(WL)*const2*F+(1-const2)*eye(F);
        t = toc;


%======================gurobi optimizing solver=======================     
%         model.obj = fT';
%         model.Q = sparse(H);
%         model.A = sparse(ones(1,length(fT)));
%         model.rhs = 1;
%         model.sense = '=';
%         model.lb = zeros(1,length(fT));
%         model.ub = ones(1,length(fT));
%         %param{f}.timelimit = 0.1;
%         param.OutputFlag = 0;
%         result = gurobi(model, param);
%         WL= result.x;
%======================================================================
        
%     %================= decompose SVD version=======================
%     fT = lemma4*sum(W*F*const2.*distsq)'/(F*P)+(2*(A1'*B1)/(F*P))';
%     H = B1'*B1/(F*P);
%     B = sqrt(H);
%     index = find(diag(B')<1e-2);
%     %B(index,:) = [];
%     B(index,:) = [];
%     %fT(index) = [];
%     A = -0.5*((B')\fT);
%     %======================================
    end



end
end

