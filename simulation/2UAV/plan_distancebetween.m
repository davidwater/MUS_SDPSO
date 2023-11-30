j=7;
rx1=xc1(j+1)-xc1(j);
ry1=yc1(j+1)-yc1(j);
rx2=xc2(j+1)-xc2(j);
ry2=yc2(j+1)-yc2(j);
pnstd=inf;
pnstt=0;

for i=1:200
    p=[xc1(j)+(i*rx1/200) yc1(j)+(i*ry1/200); xc2(j)+(i*rx2/200) yc2(j)+(i*ry2/200)];
    pd=pdist(p);
    if pd<pnstd
        pnstd=pd;
        pnstt=i;
    end
end