%%2無人機 無障礙物
clc;
clear;
close all;

%%起始點&目標點&速度&安全距離設定
%%crossing
%xs1 = -20; ys1 =   0; xg1 =  20; yg1 =   0;
%xs2 =   0; ys2 = -20; xg2 =   0; yg2 =  20;
%%head-on
xs1 =  -200; ys1 = 10; xg1 =   -150; yg1 =  100;
xs2 =  -150; ys2 =  100; xg2 =   -200; yg2 = 10;
l1_max = ((xs1 - xg1)^2 + (ys1 - yg1)^2)^(0.5); % UAV1 maximum flight distance
l2_max = ((xs2 - xg2)^2 + (ys2 - yg2)^2)^(0.5); % UAV2 maximum flight distance

it = 1000; % numbers of iteration
df = 2; % destination range
ds = 10;  % safety distance
dt = 0.5; % update rate

% cost value initialization
pun_coeff_1 = zeros(1,it);
pun_coeff_2 = zeros(1,it); 
pun_coeff_3 = zeros(1,it);
pun_coeff_4 = zeros(1,it);
F = zeros(3,it); F1 = zeros(1,it); F2 = zeros(1,it); F3 = zeros(1,it); % individual cost value
F_total = zeros(1,it); % total cost value
f_yaw = zeros(2,it); % sub-cost value in 3
w1=2;            % relative weight factor of cost funtion 1
w2=1;            % relative weight factor of cost funtion 2
w3=1;            % relative weight factor of cost funtion 3

%% SDPSO
% 初始目前位置&初速設定
xc1(1) = xs1; yc1(1) = ys1; vx1(1) = 0; vy1(1) = 0; 
xc2(1) = xs2; yc2(1) = ys2; vx2(1) = 0; vy2(1) = 0; 
dn1(1) = ((xc1(1) - xg1)^2 + (yc1(1) - yg1)^2)^(0.5); % distance between UAV 1 and its goal
dn2(1) = ((xc2(1) - xg2)^2 + (yc2(1) - yg2)^2)^(0.5); % distance between UAV 2 and its goal
d12(1) = ((xc1(1) - xc2(1))^2 + (yc1(1) - yc2(1))^2)^(0.5); % distance between UAV 1 and UAV 2
d1 = 0; % UAV 1 distance between current position and last position
d2 = 0; % UAV 2 distance between current position and last position
d_total = 0; % all UAVs total moving distance 

%初始heading設定
h1(1) = heading((xg1-xs1),(yg1-ys1));
h2(1) = heading((xg2-xs2),(yg2-ys2));

%avoid special case i.e. heading angle = 45*n degree
if (h1(1) == pi/4 || h1(1) == 0.75*pi || h1(1) == -pi/4 || h1(1) == -0.75*pi)
    xs1 = xs1 + 0.01;
elseif (h2(1) == pi/4 || h2(1) == 0.75*pi || h2(1) == -pi/4 || h2(1) == -0.75*pi)
    xs2 = xs2 + 0.01;
end

mindist = inf;
mindist_t = 0;

% SDPSO算出下一航點，飛往下一航點，判斷是否抵達，繼續計算下一航點直到抵達目標點
tic
for i = 1:it

    while i == 1
        [vx1(i+1), vy1(i+1), vx2(i+1), vy2(i+1)] = ...
        SDPSO(xc1(i),yc1(i),xs1,ys1,xg1,yg1,l1_max,vx1(i),vy1(i),h1(i),h1(i), ...
            xc2(i),yc2(i),xs2,ys2,xg2,yg2,l2_max,vx2(i),vy2(i),h2(i),h2(i), ...
            ds);
        wvx1(i) = cos(h1(i))*vx1(i+1)-sin(h1(i))*vy1(i+1);
        wvy1(i) = sin(h1(i))*vx1(i+1)+cos(h1(i))*vy1(i+1);
        wvx2(i) = cos(h2(i))*vx2(i+1)-sin(h2(i))*vy2(i+1);
        wvy2(i) = sin(h2(i))*vx2(i+1)+cos(h2(i))*vy2(i+1);
        xc1(i+1) = xc1(i) + wvx1(i)*dt;
        yc1(i+1) = yc1(i) + wvy1(i)*dt;
        xc2(i+1) = xc2(i) + wvx2(i)*dt;
        yc2(i+1) = yc2(i) + wvy2(i)*dt;
        h1(i+1) = heading((xg1-xc1(i)),(yg1-yc1(i)));
        h2(i+1) = heading((xg2-xc2(i)),(yg2-yc2(i)));
        d12(i) = ((xc1(i) - xc2(i))^2 + (yc1(i) - yc2(i))^2)^(0.5);
        dn1(i) = ((xc1(i) - xg1)^2 + (yc1(i) - yg1)^2)^(0.5);
        dn2(i) = ((xc2(i) - xg2)^2 + (yc2(i) - yg2)^2)^(0.5);
        disp(['xc1 = ' num2str(xc1(i+1)) ' yc1 = ' num2str(yc1(i+1)) ' xc2 = ' num2str(xc2(i+1)) ' yc2 = ' num2str(yc2(i+1))]);
        i = i+1;
    end

    [vx1(i+1), vy1(i+1), vx2(i+1), vy2(i+1)] = ...
    SDPSO(xc1(i),yc1(i),xs1,ys1,xg1,yg1,l1_max,vx1(i),vy1(i),h1(i),h1(i-1), ...
        xc2(i),yc2(i),xs2,ys2,xg2,yg2,l2_max,vx2(i),vy2(i),h2(i),h2(i-1), ...
        ds);
    wvx1(i) = cos(h1(i))*vx1(i+1)-sin(h1(i))*vy1(i+1);
    wvy1(i) = sin(h1(i))*vx1(i+1)+cos(h1(i))*vy1(i+1);
    wvx2(i) = cos(h2(i))*vx2(i+1)-sin(h2(i))*vy2(i+1);
    wvy2(i) = sin(h2(i))*vx2(i+1)+cos(h2(i))*vy2(i+1);
    xc1(i+1) = xc1(i) + wvx1(i)*dt;
    yc1(i+1) = yc1(i) + wvy1(i)*dt;
    xc2(i+1) = xc2(i) + wvx2(i)*dt;
    yc2(i+1) = yc2(i) + wvy2(i)*dt;
    h1(i+1) = heading((xg1-xc1(i)),(yg1-yc1(i)));
    h2(i+1) = heading((xg2-xc2(i)),(yg2-yc2(i)));
    d12(i) = ((xc1(i) - xc2(i))^2 + (yc1(i) - yc2(i))^2)^(0.5);
    dn1(i) = ((xc1(i) - xg1)^2 + (yc1(i) - yg1)^2)^(0.5);
    dn2(i) = ((xc2(i) - xg2)^2 + (yc2(i) - yg2)^2)^(0.5);
    d1 = ((xc1(i) - xc1(i-1))^2 + (yc1(i) - yc1(i-1))^2)^(0.5);
    d2 = ((xc2(i) - xc2(i-1))^2 + (yc2(i) - yc2(i-1))^2)^(0.5);
    d_total = d_total + (d1 + d2); % total distance
%     disp(['iteration: ' num2str(i) ' xc1 = ' num2str(xc1(i+1)) ' yc1 = ' num2str(yc1(i+1)) ' xc2 = ' num2str(xc2(i+1)) ' yc2 = ' num2str(yc2(i+1))]);

    %avoid special case i.e. heading angle = 45*n degree
    if (h1(i+1) == pi/4 || h1(i+1) == 0.75*pi || h1(i+1) == -pi/4 || h1(i+1) == -0.75*pi)
        xc1(i+1) = xc1(i+1) + 0.01;
    elseif (h2(i+1) == pi/4 || h2(i+1) == 0.75*pi || h2(i+1) == -pi/4 || h2(i+1) == -0.75*pi)
        xc2(i+1) = xc2(i+1) + 0.01;
    end
    
    %check if UAVs violate the minimum distance
    if d12(i) < mindist
        mindist = d12(i);
        mindist_t = i;
    end

    %%computing cost value
    % F1
    pun_coeff_1(1,i) = exp(1/(dn1(i) + dn2(i)));
    F(1, i) = ((dn1(i)^2/l1_max) + (dn2(i)^2/l2_max)) * pun_coeff_1(1,i);
    F1(i) = F(1,i) ;

    % F2
    pun_coeff_2(1,i) = 200 * exp(-((d12(i)./d12(1)).^2));
    if d12 >= ds
        F(2, i) = 0;
    else
        F(2, i) = (1/d12(i))*pun_coeff_2(1,i);
    end
    F2(i) = F(2,i);


    % F3
    pun_coeff_3(1,i) = 100 * exp(-(((h1(i)-h1(i-1))./(pi/2)).^2));
    pun_coeff_4(1,i) = 100 * exp(-(((h2(i)-h2(i-1))./(pi/2)).^2));
    if abs(h1(i) - h1(i-1)) > (pi/3)
        f_yaw(1,i) = h1(i) * pun_coeff_3(1,i);
    else
        f_yaw(1,i) = 0;
    end
    
    if abs(h2(i) - h2(i-1)) > (pi/2)
        f_yaw(2,i) = h2(i) * pun_coeff_4(1,i);
    else
        f_yaw(2,i) = 0;
    end
    F(3,i) = f_yaw(1,i) + f_yaw(2,i);
    F3(i) = F(3,i);

    F_total(i) = w1*F(1,i) + w2*F(2,i) + w3*F(3,i);
    disp(['Iteration: ' num2str(i) ' Cost value = ' num2str(F_total(i))])

    if ((xg1 - xc1(i+1))^2 + (yg1 - yc1(i+1))^2)^(0.5) <= df
        xc1(i+1) = xg1;
        yc1(i+1) = yg1;
    end
    if ((xg2 - xc2(i+1))^2 + (yg2 - yc2(i+1))^2)^(0.5) <= df
        xc2(i+1) = xg2;
        yc2(i+1) = yg2;
    end
    % check if UAVs arrive goals
    if ((xg1 - xc1(i+1))^2 + (yg1 - yc1(i+1))^2)^(0.5) <= df &&... 
       ((xg2 - xc2(i+1))^2 + (yg2 - yc2(i+1))^2)^(0.5) <= df
        disp('SDPSO mission completed');
        break; 
    end

end
t_SDPSO=toc;
disp(['Cost value = ' num2str(F_total(it))])
disp(['Process time: ' num2str(t_SDPSO)])
disp(['total distance = ' num2str(d_total)])

%%畫出路徑
figure('name', 'SDPSO');
labels = mindist_t;
hold on
xc1 = [xc1 xg1]; yc1 = [yc1 yg1];
xc2 = [xc2 xg2]; yc2 = [yc2 yg2];
plot(xc1,yc1,'LineStyle','-','marker','d','color','#4DBEEE','MarkerSize',3); hold on;
plot(xc2,yc2,'LineStyle','-','marker','d','color','#EDB120','MarkerSize',3); hold on;
% labelpoints(xc1(labels), yc1(labels), ['t = ' num2str(labels)], 'N'); hold on;
% labelpoints(xc2(labels), yc2(labels), ['t = ' num2str(labels)], 'S'); hold on;
plot(xs1,ys1,'marker','o','color','#4DBEEE','MarkerFaceColor','#4DBEEE','MarkerSize',6); hold on;
plot(xs2,ys2,'marker','o','color','#EDB120','MarkerFaceColor','#EDB120','MarkerSize',6); hold on;
plot(xg1,yg1,'marker','^','color','#4DBEEE','MarkerFaceColor','#4DBEEE','MarkerSize',6); hold on;
plot(xg2,yg2,'marker','^','color','#EDB120','MarkerFaceColor','#EDB120','MarkerSize',6); hold on;
% labelpoints(xc1(1), yc1(1), 'UAV 1 start', 'S'); hold on;
% labelpoints(xc2(1), yc2(1), 'UAV 2 start', 'S'); hold on;
% labelpoints(xc1(i), yc1(i), 'UAV 1 target', 'N'); hold on;
% labelpoints(xc2(i), yc2(i), 'UAV 2 target', 'N'); hold on;
legend('UAV1 path','UAV2 path','Location','northwest');
grid on; axis equal; axis ([-220 -120 0 110]);
xlabel('X(m)')
ylabel('Y(m)')
hold on

%%plot cost value
figure('Name','cost value')
hold on
plot(F1,'LineStyle','--','color','#0072BD'); hold on;
plot(F2,'LineStyle','-.','color','#D95319'); hold on;
plot(F3,'LineStyle',':','color','#77AC30'); hold on;
plot(F_total,'LineStyle','-','color','#000000');hold on;
legend('OF1','OF2','OF3','Total OF')
grid on; axis([0 100 0 300])
xlabel('times')
ylabel('cost value')
hold on

