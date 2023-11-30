%最短路徑
function F1 = OF1(dn1, dn2, dn3, dn4, dn5, l1_max, l2_max, l3_max, l4_max, l5_max)
    
    k=800;
    pun_coeff = k*exp(1/(dn1+dn2+dn3+dn4+dn5));
    F1 = pun_coeff*((dn1*dn1/l1_max) + (dn2*dn2/l2_max) + (dn3*dn3/l3_max) + (dn4*dn4/l4_max) + (dn5*dn5/l5_max));

end