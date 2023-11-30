%回飛問題
function F3 = OF3(h1, h1_old, h2, h2_old, h3, h3_old)
        
    alpha_max = pi/3;
    k = 100; % undefined factor
    pun_coeff_1 = k * exp(-(((h1-h1_old)/alpha_max)^2));
    pun_coeff_2 = k * exp(-(((h2-h2_old)/alpha_max)^2));
    pun_coeff_3 = k * exp(-(((h3-h3_old)/alpha_max)^2));
    
    if abs(h1 - h1_old) > alpha_max
        f_yaw_1 = h1 * pun_coeff_1;
    else
        f_yaw_1 = 0;
    end
    
    if abs(h2 - h2_old) > alpha_max
        f_yaw_2 = h2 * pun_coeff_2;
    else
        f_yaw_2 = 0;
    end

    if abs(h3 - h3_old) > alpha_max
        f_yaw_3 = h3 * pun_coeff_3;
    else
        f_yaw_3 = 0;
    end
    
    F3 = f_yaw_1 + f_yaw_2 + f_yaw_3;

end