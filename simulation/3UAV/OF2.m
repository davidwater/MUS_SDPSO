%避免碰撞
function F2 = OF2(xn1,yn1,xn2,yn2,xn3,yn3,xs1,ys1,xs2,ys2,xs3,ys3,ds)

    d12 = ((xn1 - xn2)^2 + (yn1 - yn2)^2)^(0.5); % distance between UAV 1 and UAV 2
    d13 = ((xn1 - xn3)^2 + (yn1 - yn3)^2)^(0.5); % distance between UAV 1 and UAV 3
    d23 = ((xn2 - xn3)^2 + (yn2 - yn3)^2)^(0.5); % distance between UAV 2 and UAV 3
    d12_max = ((xs1 - xs2)^2 + (ys1 - ys2)^2)^(0.5); % maximum distance between UAV 1 and UAV 2
    d13_max = ((xs1 - xs3)^2 + (ys1 - ys3)^2)^(0.5); % maximum distance between UAV 1 and UAV 3
    d23_max = ((xs2 - xs3)^2 + (ys2 - ys3)^2)^(0.5); % maximum distance between UAV 2 and UAV 3
    k = 500; % undefined factor
    pun_coeff_12 =  k * exp(-((d12/d12_max)^2)); % punishment coefficient for d12
    pun_coeff_13 =  k * exp(-((d13/d13_max)^2)); % punishment coefficient for d13
    pun_coeff_23 =  k * exp(-((d23/d23_max)^2)); % punishment coefficient for d23
    
    if d12 > ds
            f212 = 0;
        else
            f212 = 1/(d12)*pun_coeff_12;
    end 
    
    if d13 > ds
            f213 = 0;
        else
            f213 = 1/(d13)*pun_coeff_13;
    end

    if d23 > ds
        f223 = 0;
    else
        f223 = 1/(d23)*pun_coeff_23;
    end 
    
    F2 = f212 + f213 + f223;

end