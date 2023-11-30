%最短路徑
function F1 = OF1(dn1, dn2, l1_max, l2_max)
    
    k = 1;
    pun_coeff = k*exp(1/(dn1+dn2));
    
    F1 = ((dn1^2/l1_max) + (dn2^2/l2_max))*pun_coeff;

end