%避免碰撞
function F2 = OF2(xn1,yn1,xn2,yn2,xn3,yn3,xn4,yn4,xn5,yn5,xs1,ys1,xs2,ys2,xs3,ys3,xs4,ys4,xs5,ys5,ds)

    d12 = ((xn1 - xn2)^2 + (yn1 - yn2)^2)^(0.5); % distance between UAV 1 and UAV 2
    d13 = ((xn1 - xn3)^2 + (yn1 - yn3)^2)^(0.5); % distance between UAV 1 and UAV 3
    d14 = ((xn1 - xn4)^2 + (yn4 - yn4)^2)^(0.5); % distance between UAV 1 and UAV 4
    d15 = ((xn1 - xn5)^2 + (yn1 - yn5)^2)^(0.5); % distance between UAV 1 and UAV 5
    d23 = ((xn2 - xn3)^2 + (yn2 - yn3)^2)^(0.5); % distance between UAV 2 and UAV 3
    d24 = ((xn2 - xn4)^2 + (yn2 - yn4)^2)^(0.5); % distance between UAV 2 and UAV 4
    d25 = ((xn2 - xn5)^2 + (yn2 - yn5)^2)^(0.5); % distance between UAV 2 and UAV 5
    d34 = ((xn3 - xn4)^2 + (yn3 - yn4)^2)^(0.5); % distance between UAV 3 and UAV 4
    d35 = ((xn3 - xn5)^2 + (yn3 - yn5)^2)^(0.5); % distance between UAV 3 and UAV 5
    d45 = ((xn4 - xn5)^2 + (yn4 - yn5)^2)^(0.5); % distance between UAV 4 and UAV 5

    d12_max = ((xs1-xs2)^2 - (ys1-ys2)^2)^(0.5); % maximum distance between UAV 1 and UAV 2
    d13_max = ((xs1-xs3)^2 - (ys1-ys3)^2)^(0.5); % maximum distance between UAV 1 and UAV 3
    d14_max = ((xs1-xs4)^2 - (ys1-ys4)^2)^(0.5); % maximum distance between UAV 1 and UAV 4
    d15_max = ((xs1-xs5)^2 - (ys1-ys5)^2)^(0.5); % maximum distance between UAV 1 and UAV 5
    d23_max = ((xs2-xs3)^2 - (ys2-ys3)^2)^(0.5); % maximum distance between UAV 2 and UAV 3
    d24_max = ((xs2-xs4)^2 - (ys2-ys4)^2)^(0.5); % maximum distance between UAV 2 and UAV 4
    d25_max = ((xs2-xs5)^2 - (ys2-ys5)^2)^(0.5); % maximum distance between UAV 2 and UAV 5
    d34_max = ((xs3-xs4)^2 - (ys3-ys4)^2)^(0.5); % maximum distance between UAV 3 and UAV 4
    d35_max = ((xs3-xs5)^2 - (ys3-ys5)^2)^(0.5); % maximum distance between UAV 3 and UAV 5
    d45_max = ((xs4-xs5)^2 - (ys4-ys5)^2)^(0.5); % maximum distance between UAV 4 and UAV 5
    k = 10; % undefined factor
    pun_coeff_12 =  k * exp(-((d12/d12_max)^2)); 
    pun_coeff_13 =  k * exp(-((d13/d13_max)^2)); 
    pun_coeff_14 =  k * exp(-((d14/d14_max)^2));
    pun_coeff_15 =  k * exp(-((d15/d15_max)^2));
    pun_coeff_23 =  k * exp(-((d23/d23_max)^2));
    pun_coeff_24 =  k * exp(-((d24/d24_max)^2));
    pun_coeff_25 =  k * exp(-((d25/d25_max)^2));
    pun_coeff_34 =  k * exp(-((d34/d34_max)^2));
    pun_coeff_35 =  k * exp(-((d35/d35_max)^2));
    pun_coeff_45 =  k * exp(-((d45/d45_max)^2));

    if d12 >= ds
        F2_12 = 0;
    else
        F2_12 = (1/(d12))*pun_coeff_12;
    end
    
    if d13 >= ds
        F2_13 = 0;
    else
        F2_13 = (1/(d13))*pun_coeff_13;
    end
    
    if d14 >= ds
        F2_14 = 0;
    else
        F2_14 = (1/(d14))*pun_coeff_14;
    end

    if d15 >= ds
        F2_15 = 0;
    else
        F2_15 = (1/(d15))*pun_coeff_15;
    end

    if d23 >= ds
        F2_23 = 0;
    else
        F2_23 = (1/(d23))*pun_coeff_23;
    end

    if d24 >= ds
        F2_24 = 0;
    else
        F2_24 = (1/(d24))*pun_coeff_24;
    end

    if d25 >= ds
        F2_25 = 0;
    else
        F2_25 = (1/(d25))*pun_coeff_25;
    end

    if d34 >= ds
        F2_34 = 0;
    else
        F2_34 = (1/(d34))*pun_coeff_34;
    end

    if d35 >= ds
        F2_35 = 0;
    else
        F2_35 = (1/(d35))*pun_coeff_35;
    end

    if d45 >= ds
        F2_45 = 0;
    else
        F2_45 = (1/(d45))*pun_coeff_45;
    end
    
    F2 = F2_12 + F2_13 + F2_14 + F2_15 + F2_23 + F2_24 + F2_25 + F2_34 + F2_35 + F2_45;

end