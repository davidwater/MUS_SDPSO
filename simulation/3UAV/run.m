clc;
clear;
close all;

times=50;
time=zeros(1,times);
dist1=zeros(1,times);
dist2=zeros(1,times);
dist3=zeros(1,times);
dist_total=zeros(1,times);

for k=1:times
    main; %有些code要(%)
    time(k)=t;
    d1=zeros(1,i);
    d2=zeros(1,i);
    d3=zeros(1,i);
    
    for j=1:i
        
        p1=[xc1(j),yc1(j);xc1(j+1),yc1(j+1)];
        d1(j)=pdist(p1);
        p2=[xc2(j),yc2(j);xc2(j+1),yc2(j+1)];
        d2(j)=pdist(p2);
        p3=[xc3(j),yc3(j);xc3(j+1),yc3(j+1)];
        d3(j)=pdist(p3);
    
    end
    
    p1=[xc1(j+1),yc1(j+1);xg1,yg1];
    d1(j+1)=pdist(p1);
    dist1(k)=sum(d1);
    p2=[xc2(j+1),yc2(j+1);xg2,yg2];
    d2(j+1)=pdist(p2);
    dist2(k)=sum(d2);
    p3=[xc3(j+1),yc3(j+1);xg3,yg3];
    d3(j+1)=pdist(p3);
    dist3(k)=sum(d3);
    dist_total(k)=dist1(k)+dist2(k)+dist3(k);
    
    disp(['times : ' num2str(k)]);

end

time_avg=mean(time);
dist_total_avg=mean(dist_total);
time_best=min(time);
dist_total_best=min(dist_total);
time_std=std(time);
dist_total_std=std(dist_total);
disp('mission completed');