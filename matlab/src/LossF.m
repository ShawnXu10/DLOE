function Loss = LossF( X, Xorfedler, W, WL, C, r,RayConv, param)
%COSTF Summary of this function goes here
%   Detailed explanation goes here
    lemma1 = param.lemma1;
    lemma2 = param.lemma2;
    lemma3 = param.lemma3;
    

    X = X';
    Xorfedler = Xorfedler';
    W = W';

    F = size(X, 2);
    P = size(X,1)/3;
    C = cell2mat(C)';
    r = reshape([r{:}], [3*P, F]);
    Lout = diag(sum(W*WL))-(W*WL);
    Lin = diag(sum((W*WL)'))-(W*WL)';
    Loss = 0;
    % ===================== loss of first term ============================
    term0 = norm(X*Lout, 'fro')^2/(F*P);
    Loss = Loss + term0;
    %====================== loss of second term ===========================
    term4 = lemma1*trace(Xorfedler'*(Lout+Lin)*Xorfedler)/(P*F);%*P_fiedler
    Loss = Loss + term4;
    
    
    term3 = 0;
    for f = 1:F
        C_tmp = C(:,f);
        for i = 1:P
            if ~isnan(r(3*i-2, f))
                r_tmp = r(3*i-2:3*i, f);
                X_tmp = X(3*i-2:3*i, f);
                term3 = term3 + lemma2*norm(((X_tmp - C_tmp)'*r_tmp)*r_tmp + C_tmp - X_tmp, 'fro')^2/(F*P);
            end
        end
    end
    Loss = Loss + term3;
    
    %======================loss of term5 ==================================
    
    term5  = lemma3*norm(RayConv.*(W*WL),'fro')^2/(F*P);
    Loss = Loss + term5;
end