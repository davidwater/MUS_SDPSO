%避免碰撞
function F2 = OF2(xn1,yn1,xn2,yn2,xs1,ys1,xs2,ys2,ds)

    d12 = ((xn1 - xn2)^2 + (yn1 - yn2)^2)^(0.5); % distance between UAV 1 and UAV 2
    d12_max = ((xs1-xs2)^2 - (ys1-ys2)^2)^(0.5); % maximum distance between UAV 1 and UAV 2
    k = 200; % undefined factor
    pun_coeff =  k * exp(-((d12/d12_max)^2)); % punishment coefficient

    if d12 > ds
        F2 = 0;
    else
        F2 = (1/d12)*pun_coeff;
    end 
end