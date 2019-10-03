function X_opt = X_opt( A, ray, t,param)
%   Optimize cost function over X
% min ||D(I-W)X||^2 + lemma1*sum(D_ii*W_ij*dis(Xi,Xj)^2) + lemma2*dist(X,ray)
    lemma1 = param.lemma1;
    lemma2 = param.lemma2;

    F = numel(ray);
    P = size(ray{1},2);
    C = cell2mat(t)';
    r = reshape([ray{:}], [3*P, F]);
    Lout = diag(sum(A,2)) - A;

    N_var = F*P*3;
    H = sparse(N_var, N_var);
    fT = zeros(N_var, 1);
    %Calculate fT from term 4
    for f = 1:F 
        C_tmp = C(:,f); 
        for i = 1:P
            if isnan(r(3*i-2,f))
                continue;
            else
            r_tmp = r(3*i-2:3*i, f);
            ib = 3*(i-1)*F+f;
            fT(ib) = -2*(r_tmp(1)^2-1)*((r_tmp(1)^2-1)*C_tmp(1)+r_tmp(1)*r_tmp(2)*C_tmp(2)+r_tmp(1)*r_tmp(3)*C_tmp(3))...
                -2*r_tmp(1)*r_tmp(2)*(r_tmp(1)*r_tmp(2)*C_tmp(1)+(r_tmp(2)^2-1)*C_tmp(2)+r_tmp(2)*r_tmp(3)*C_tmp(3))...
                -2*r_tmp(1)*r_tmp(3)*(r_tmp(1)*r_tmp(3)*C_tmp(1)+r_tmp(2)*r_tmp(3)*C_tmp(2)+(r_tmp(3)^2-1)*C_tmp(3));
            fT(ib+F) = -2*r_tmp(1)*r_tmp(2)*((r_tmp(1)^2-1)*C_tmp(1)+r_tmp(1)*r_tmp(2)*C_tmp(2)+r_tmp(1)*r_tmp(3)*C_tmp(3))...
                -2*(r_tmp(2)^2-1)*(r_tmp(1)*r_tmp(2)*C_tmp(1)+(r_tmp(2)^2-1)*C_tmp(2)+r_tmp(2)*r_tmp(3)*C_tmp(3))...
                -2*r_tmp(2)*r_tmp(3)*(r_tmp(1)*r_tmp(3)*C_tmp(1)+r_tmp(2)*r_tmp(3)*C_tmp(2)+(r_tmp(3)^2-1)*C_tmp(3));
            fT(ib+2*F) = -2*r_tmp(1)*r_tmp(3)*((r_tmp(1)^2-1)*C_tmp(1)+r_tmp(1)*r_tmp(2)*C_tmp(2)+r_tmp(1)*r_tmp(3)*C_tmp(3))...
                -2*r_tmp(2)*r_tmp(3)*(r_tmp(1)*r_tmp(2)*C_tmp(1)+(r_tmp(2)^2-1)*C_tmp(2)+r_tmp(2)*r_tmp(3)*C_tmp(3))...
                -2*(r_tmp(3)^2-1)*(r_tmp(1)*r_tmp(3)*C_tmp(1)+r_tmp(2)*r_tmp(3)*C_tmp(2)+(r_tmp(3)^2-1)*C_tmp(3));
            end
        end        
    end
    fT = lemma2*fT/(F*P);

    H_temp = (Lout'*Lout)/(F*P);
    for i = 1:3*P
        H(F*(i-1)+1:F*i,F*(i-1)+1:F*i) = H(F*(i-1)+1:F*i,F*(i-1)+1:F*i) + H_temp;
    end

    %Calculate H form term 4
    iindex =  [];
    jindex = [];
    H_tempVec = [];        
    for f = 1:F
        for i = 1:P
            if isnan(r(3*i-2,f))
                continue;
            else
                r_tmp = r(3*i-2:3*i, f);
                ia = 3*(i-1)*F+f;
                W_temp_1 = [(r_tmp(1)^2-1);r_tmp(1)*r_tmp(2);r_tmp(1)*r_tmp(3)];
                W_temp_2 = [r_tmp(1)*r_tmp(2);(r_tmp(2)^2-1);r_tmp(2)*r_tmp(3)];
                W_temp_3 = [r_tmp(1)*r_tmp(3);r_tmp(2)*r_tmp(3);(r_tmp(3)^2-1)];
                H_temp = lemma2*(W_temp_1*W_temp_1' + W_temp_2*W_temp_2' +W_temp_3*W_temp_3')/(F*P);
                iindex = [iindex ia ia+F ia+2*F ia ia+F ia+2*F ia ia+F ia+2*F];
                jindex = [jindex ia ia ia ia+F ia+F ia+F ia+2*F ia+2*F ia+2*F];
                H_tempVec = [H_tempVec; H_temp(:)];
            end
        end
    end 
    H = H + sparse(iindex,jindex,H_tempVec, N_var,N_var);
    
    %Calculate A from term 4
    A_L = A+A';
    H_temp = lemma1*(diag(sum(A_L))-A_L)/(F*P);
    for i = 1:3*P
        H(F*(i-1)+1:F*i,F*(i-1)+1:F*i) = H(F*(i-1)+1:F*i,F*(i-1)+1:F*i) + H_temp;
    end
    X_opt = -(2*sparse(H))\fT;
    X_opt = reshape(X_opt, [F, 3*P]);

end    