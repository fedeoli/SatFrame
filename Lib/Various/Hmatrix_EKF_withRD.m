%%
function H = Hmatrix_EKF_withRD(DynOpt,xtotal,j_select)
    
    for n=1:DynOpt.ObserverTest.Nagents
       xi(n,:) = xtotal(1+(n-1)*6:3+(n-1)*6);
    end
    
    H_string = 'DynOpt.sym.Hsym_pos(';
    for dim=1:3
        tmp = strcat('xi(',num2str(j_select),',',num2str(dim),')');
        H_string = strcat(H_string,tmp,',');
    end
    for n=1:DynOpt.ObserverTest.Nagents
        if n~=j_select
            for dim =1:3
                tmp = strcat('xi(',num2str(n),',',num2str(dim),')');
                H_string = strcat(H_string,tmp,',');
            end
        end
    end
    H_string = strcat(H_string(1:end-1),');');
    H_tmp = eval(H_string);
    H = double(H_tmp);
end