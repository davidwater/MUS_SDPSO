%% 3無人機 無障礙物
clc;
clear;
close all;

%%起始點&目標點&速度&安全距離設定
xs1 = -150; ys1 = -40; xg1 = -250; yg1 = 60;
xs2 = -200; ys2 = -40; xg2 = -200; yg2 = 60;
xs3 = -250; ys3 = -40; xg3 = -150; yg3 = 60;
l1_max = ((xs1 - xg1)^2 + (ys1 - yg1)^2)^(0.5); % UAV1 maximum flight distance
l2_max = ((xs2 - xg2)^2 + (ys2 - yg2)^2)^(0.5); % UAV2 maximum flight distance
l3_max = ((xs3 - xg3)^2 + (ys3 - yg3)^2)^(0.5); % UAV2 maximum flight distance

it = 1000; % numbers of iteration
df = 2; % destination range
ds = 5;  % safety distance
dt = 0.5; % update rate

% cost value initialization
pun_coeff_1 = zeros(1,it);
pun_coeff_2 = zeros(1,it); 
pun_coeff_3 = zeros(1,it);
pun_coeff_4 = zeros(1,it);
pun_coeff_5 = zeros(1,it); 
pun_coeff_6 = zeros(1,it);
pun_coeff_7 = zeros(1,it);
F = zeros(3,it); F1 = zeros(1,it); F2 = zeros(1,it); F3 = zeros(1,it); % individual cost value
F_total = zeros(1,it); % total cost value
f_collision = zeros(3,it); % sub-cost value in 2
f_yaw = zeros(3,it); % sub-cost value in 3
w1=0.33;            % relative weight factor of cost funtion 1
w2=0.33;            % relative weight factor of cost funtion 2
w3=0.33;            % relative weight factor of cost funtion 3

%% PSO
%%初始目前位置&初速設定
xc1(1) = xs1; yc1(1) = ys1; vx1(1) = 0; vy1(1) = 0;
xc2(1) = xs2; yc2(1) = ys2; vx2(1) = 0; vy2(1) = 0;
xc3(1) = xs3; yc3(1) = ys3; vx3(1) = 0; vy3(1) = 0;
dn1(1) = ((xc1(1) - xg1)^2 + (yc1(1) - yg1)^2)^(0.5); % distance between UAV 1 and its goal
dn2(1) = ((xc2(1) - xg2)^2 + (yc2(1) - yg2)^2)^(0.5); % distance between UAV 2 and its goal
dn3(1) = ((xc3(1) - xg3)^2 + (yc3(1) - yg3)^2)^(0.5); % distance between UAV 3 and its goal
d12(1) = ((xc1(1) - xc2(1))^2 + (yc1(1) - yc2(1))^2)^(0.5); % distance between UAV 1 and UAV 2
d13(1) = ((xc1(1) - xc3(1))^2 + (yc1(1) - yc3(1))^2)^(0.5); % distance between UAV 1 and UAV 3
d23(1) = ((xc2(1) - xc3(1))^2 + (yc2(1) - yc3(1))^2)^(0.5); % distance between UAV 2 and UAV 3
d1 = 0; % UAV 1 distance between current position and last position
d2 = 0; % UAV 2 distance between current position and last position
d3 = 0; % UAV 3 distance between current position and last position
d_total = 0; % all UAVs total moving distance 

%%初始heading設定
h1(1) = heading((xg1-xs1),(yg1-ys1));
h2(1) = heading((xg2-xs2),(yg2-ys2));
h3(1) = heading((xg3-xs3),(yg3-ys3));

%avoid special case i.e. heading angle = 45*n degree
if (h1(1) == pi/4 || h1(1) == 0.75*pi || h1(1) == -pi/4 || h1(1) == -0.75*pi)
    xs1 = xs1 + 0.01;
elseif (h2(1) == pi/4 || h2(1) == 0.75*pi || h2(1) == -pi/4 || h2(1) == -0.75*pi)
    xs2 = xs2 + 0.01;
elseif (h3(1) == pi/4 || h3(1) == 0.75*pi || h3(1) == -pi/4 || h3(1) == -0.75*pi)
    xs3 = xs3 + 0.01;
end

mindist_12 = inf; mindist_13 = inf; mindist_23 = inf;
mindist_t_12 = 0; mindist_t_13 = 0; mindist_t_23 = 0;

% PSO算出下一航點，飛往下一航點，判斷是否抵達，繼續計算下一航點直到抵達目標點
tic
for i = 1:it
    
    while i == 1
        [vx1(i+1), vy1(i+1), vx2(i+1), vy2(i+1), vx3(i+1), vy3(i+1)] = ...
        PSO(xc1(i),yc1(i),xs1,ys1,xg1,yg1,l1_max,vx1(i),vy1(i),h1(i),h1(i), ...
            xc2(i),yc2(i),xs2,ys2,xg2,yg2,l2_max,vx2(i),vy2(i),h2(i),h2(i), ...
            xc3(i),yc3(i),xs3,ys3,xg3,yg3,l3_max,vx3(i),vy3(i),h3(i),h3(i), ...
            ds);
        wvx1(i) = cos(h1(i))*vx1(i+1)+sin(h1(i))*vy1(i+1);
        wvy1(i) = sin(h1(i))*vx1(i+1)-cos(h1(i))*vy1(i+1);
        wvx2(i) = cos(h2(i))*vx2(i+1)+sin(h2(i))*vy2(i+1);
        wvy2(i) = sin(h2(i))*vx2(i+1)-cos(h2(i))*vy2(i+1);
        wvx3(i) = cos(h3(i))*vx3(i+1)+sin(h3(i))*vy3(i+1);
        wvy3(i) = sin(h3(i))*vx3(i+1)-cos(h3(i))*vy3(i+1);
        xc1(i+1) = xc1(i) + wvx1(i)*dt;
        yc1(i+1) = yc1(i) + wvy1(i)*dt;
        xc2(i+1) = xc2(i) + wvx2(i)*dt;
        yc2(i+1) = yc2(i) + wvy2(i)*dt;
        xc3(i+1) = xc3(i) + wvx3(i)*dt;
        yc3(i+1) = yc3(i) + wvy3(i)*dt;
        h1(i+1) = heading((xg1-xc1(i)),(yg1-yc1(i)));
        h2(i+1) = heading((xg2-xc2(i)),(yg2-yc2(i)));
        h3(i+1) = heading((xg3-xc3(i)),(yg3-yc3(i)));
        d12(i) = ((xc1(i) - xc2(i))^2 + (yc1(i) - yc2(i))^2)^(0.5);
        d13(i) = ((xc1(i) - xc3(i))^2 + (yc1(i) - yc3(i))^2)^(0.5);
        d23(i) = ((xc2(i) - xc3(i))^2 + (yc2(i) - yc3(i))^2)^(0.5);
        dn1(i) = ((xc1(i) - xg1)^2 + (yc1(i) - yg1)^2)^(0.5);
        dn2(i) = ((xc2(i) - xg2)^2 + (yc2(i) - yg2)^2)^(0.5);
        dn3(i) = ((xc3(i) - xg3)^2 + (yc3(i) - yg3)^2)^(0.5);
        disp(['xc1 = ' num2str(xc1(i+1)) ' yc1 = ' num2str(yc1(i+1)) ' xc2 = ' num2str(xc2(i+1)) ' yc2 = ' num2str(yc2(i+1)) ...
            ' xc3 = ' num2str(xc3(i+1)) ' yc3 = ' num2str(yc3(i+1))]);
        i = i+1;
    end

    [vx1(i+1), vy1(i+1), vx2(i+1), vy2(i+1), vx3(i+1), vy3(i+1)] = ...
        PSO(xc1(i),yc1(i),xs1,ys1,xg1,yg1,l1_max,vx1(i),vy1(i),h1(i),h1(i-1), ...
            xc2(i),yc2(i),xs2,ys2,xg2,yg2,l2_max,vx2(i),vy2(i),h2(i),h2(i-1), ...
            xc3(i),yc3(i),xs3,ys3,xg3,yg3,l3_max,vx3(i),vy3(i),h3(i),h3(i-1), ...
            ds);
    wvx1(i) = cos(h1(i))*vx1(i+1)+sin(h1(i))*vy1(i+1);
    wvy1(i) = sin(h1(i))*vx1(i+1)-cos(h1(i))*vy1(i+1);
    wvx2(i) = cos(h2(i))*vx2(i+1)+sin(h2(i))*vy2(i+1);
    wvy2(i) = sin(h2(i))*vx2(i+1)-cos(h2(i))*vy2(i+1);
    wvx3(i) = cos(h3(i))*vx3(i+1)+sin(h3(i))*vy3(i+1);
    wvy3(i) = sin(h3(i))*vx3(i+1)-cos(h3(i))*vy3(i+1);
    xc1(i+1) = xc1(i) + wvx1(i)*dt;
    yc1(i+1) = yc1(i) + wvy1(i)*dt;
    xc2(i+1) = xc2(i) + wvx2(i)*dt;
    yc2(i+1) = yc2(i) + wvy2(i)*dt;
    xc3(i+1) = xc3(i) + wvx3(i)*dt;
    yc3(i+1) = yc3(i) + wvy3(i)*dt;
    h1(i+1) = heading((xg1-xc1(i)),(yg1-yc1(i)));
    h2(i+1) = heading((xg2-xc2(i)),(yg2-yc2(i)));
    h3(i+1) = heading((xg3-xc3(i)),(yg3-yc3(i)));
    d12(i) = ((xc1(i) - xc2(i))^2 + (yc1(i) - yc2(i))^2)^(0.5);
    d13(i) = ((xc1(i) - xc3(i))^2 + (yc1(i) - yc3(i))^2)^(0.5);
    d23(i) = ((xc2(i) - xc3(i))^2 + (yc2(i) - yc3(i))^2)^(0.5);
    dn1(i) = ((xc1(i) - xg1)^2 + (yc1(i) - yg1)^2)^(0.5);
    dn2(i) = ((xc2(i) - xg2)^2 + (yc2(i) - yg2)^2)^(0.5);
    dn3(i) = ((xc3(i) - xg3)^2 + (yc3(i) - yg3)^2)^(0.5);
    d1 = ((xc1(i) - xc1(i-1))^2 + (yc1(i) - yc1(i-1))^2)^(0.5);
    d2 = ((xc2(i) - xc2(i-1))^2 + (yc2(i) - yc2(i-1))^2)^(0.5);
    d3 = ((xc3(i) - xc3(i-1))^2 + (yc3(i) - yc3(i-1))^2)^(0.5);
    d_total = d_total + (d1 + d2 + d3);
    disp(['iteration: ' num2str(i) ' xc1 = ' num2str(xc1(i+1)) ' yc1 = ' num2str(yc1(i+1)) ' xc2 = ' num2str(xc2(i+1)) ' yc2 = ' num2str(yc2(i+1)) ...
            ' xc3 = ' num2str(xc3(i+1)) ' yc3 = ' num2str(yc3(i+1))]);

    %avoid special case i.e. heading angle = 45*n degree
    if (h1(i+1) == pi/4 || h1(i+1) == 0.75*pi || h1(i+1) == -pi/4 || h1(i+1) == -0.75*pi)
        xc1(i+1) = xc1(i+1) + 0.01;
    end
    if (h2(i+1) == pi/4 || h2(i+1) == 0.75*pi || h2(i+1) == -pi/4 || h2(i+1) == -0.75*pi)
        xc2(i+1) = xc2(i+1) + 0.01;
    end
    if (h3(i+1) == pi/4 || h3(i+1) == 0.75*pi || h3(i+1) == -pi/4 || h3(i+1) == -0.75*pi)
        xc3(i+1) = xc3(i+1) + 0.01;
    end

    %check if UAVs violate the minimum distance
    if d12(i) < mindist_12
        mindist_12 = d12(i);
        mindist_t_12 = i;
    end
    if d13(i) < mindist_13
        mindist_13 = d13(i);
        mindist_t_13 = i;
    end
    if d23(i) < mindist_23
        mindist_23 = d23(i);
        mindist_t_23 = i;
    end
    
    %%computing cost value
    % F1
    pun_coeff_1(1,i) = (dn1(i) + dn2(i) + dn3(i)) / (l1_max + l2_max + l3_max);
    F(1, i) = (dn1(i) + dn2(i) + dn3(i)) * pun_coeff_1(1,i);
    F1(i) = F(1,i);

    % F2
    pun_coeff_2(1,i) = 200 * exp(-((d12(i)./d12(1)).^2));
    pun_coeff_3(1,i) = 200 * exp(-((d13(i)./d13(1)).^2));
    pun_coeff_4(1,i) = 200 * exp(-((d23(i)./d23(1)).^2));
    if d12 >= ds
        f_collision(1,i) = 0;
    else
        f_collision(1,i) = (1/d12(i))*pun_coeff_2(1,i);
    end

    if d13 >= ds
        f_collision(2,i) = 0;
    else
        f_collision(2,i) = (1/d13(i))*pun_coeff_3(1,i);
    end

    if d23 >= ds
        f_collision(3,i) = 0;
    else
        f_collision(3,i) = (1/d23(i))*pun_coeff_4(1,i);
    end

    F(2,i) = f_collision(1,i) + f_collision(2,i) + f_collision(3,i);
    F2(i) = F(2,i);

    % F3
    pun_coeff_5(1,i) = 100 * exp(-(((h1(i)-h1(i-1))./(pi/2)).^2));
    pun_coeff_6(1,i) = 100 * exp(-(((h2(i)-h2(i-1))./(pi/2)).^2));
    pun_coeff_7(1,i) = 100 * exp(-(((h3(i)-h3(i-1))./(pi/2)).^2));
    if abs(h1(i) - h1(i-1)) > (pi/2)
        f_yaw(1,i) = h1(i) * pun_coeff_5(1,i);
    else
        f_yaw(1,i) = 0;
    end
    
    if abs(h2(i) - h2(i-1)) > (pi/2)
        f_yaw(2,i) = h2(i) * pun_coeff_6(1,i);
    else
        f_yaw(2,i) = 0;
    end

    if abs(h3(i) - h3(i-1)) > (pi/2)
        f_yaw(3,i) = h3(i) * pun_coeff_7(1,i);
    else
        f_yaw(3,i) = 0;
    end

    F(3,i) = f_yaw(1,i) + f_yaw(2,i) + f_yaw(3,i);
    F3(i) = F(3,i);

    F_total(i) = w1*F(1,i) + w2*F(2,i) + w3*F(3,i);
    
    % check if UAVs arrive goals
    if ((xg1 - xc1(i+1))^2 + (yg1 - yc1(i+1))^2)^(0.5) <= df &&... 
       ((xg2 - xc2(i+1))^2 + (yg2 - yc2(i+1))^2)^(0.5) <= df &&... 
       ((xg3 - xc3(i+1))^2 + (yg3 - yc3(i+1))^2)^(0.5) <= df
        disp('mission completed');
        break; 
    end

end
t_PSO=toc;
disp(['Cost value: ' num2str(F_total(i))])
disp(['Process time: ' num2str(t_PSO)])
disp(['Total distance: ' num2str(d_total)])

%%畫出路徑
figure('Name','3 UAVs PSO')
label_12 = mindist_t_12;
label_13 = mindist_t_13;
lable_23 = mindist_t_23;
xc1 = [xc1 xg1]; yc1 = [yc1 yg1];
xc2 = [xc2 xg2]; yc2 = [yc2 yg2];
xc3 = [xc3 xg3]; yc3 = [yc3 yg3];
plot(xc1,yc1,'LineStyle','-','marker','d','color','#4DBEEE','MarkerSize',3); hold on;
plot(xc2,yc2,'LineStyle','-','marker','d','color','#EDB120','MarkerSize',3); hold on;
plot(xc3,yc3,'LineStyle','-','marker','d','color','#7E2F8E','MarkerSize',3); hold on;
% labelpoints(xc1(label_12), yc1(label_12), ['t = ' num2str(label_12)], 'N'); hold on;
% labelpoints(xc1(label_13), yc1(label_13), ['t = ' num2str(label_13)], 'N'); hold on;
% labelpoints(xc2(label_12), yc2(label_12), ['t = ' num2str(label_12)], 'N'); hold on;
% labelpoints(xc2(mindist_t_23), yc2(mindist_t_23), ['t = ' num2str(mindist_t_23)], 'N'); hold on;
% labelpoints(xc3(label_13), yc3(label_13), ['t = ' num2str(label_13)], 'N'); hold on;
% labelpoints(xc3(mindist_t_23), yc3(mindist_t_23), ['t = ' num2str(mindist_t_23)], 'N'); hold on;
plot(xs1,ys1,'marker','o','color','#4DBEEE','MarkerFaceColor','#4DBEEE','MarkerSize',4); hold on;
plot(xs2,ys2,'marker','o','color','#EDB120','MarkerFaceColor','#EDB120','MarkerSize',4); hold on;
plot(xs3,ys3,'marker','o','color','#7E2F8E','MarkerFaceColor','#7E2F8E','MarkerSize',4); hold on;
plot(xg1,yg1,'marker','^','color','#4DBEEE','MarkerFaceColor','#4DBEEE','MarkerSize',4); hold on;
plot(xg2,yg2,'marker','^','color','#EDB120','MarkerFaceColor','#EDB120','MarkerSize',4); hold on;
plot(xg3,yg3,'marker','^','color','#7E2F8E','MarkerFaceColor','#7E2F8E','MarkerSize',4); hold on;
labelpoints(xc1(1), yc1(1), 'UAV 1 start', 'S'); hold on;
labelpoints(xc2(1), yc2(1), 'UAV 2 start', 'S'); hold on;
labelpoints(xc3(1), yc3(1), 'UAV 3 start', 'S'); hold on;
labelpoints(xc1(i), yc1(i), 'UAV 1 target', 'N'); hold on;
labelpoints(xc2(i), yc2(i), 'UAV 2 target', 'N'); hold on;
labelpoints(xc3(i), yc3(i), 'UAV 3 target', 'N'); hold on;
legend('UAV1 path','UAV2 path','UAV3 path','Location','northwest');
grid on; axis equal; axis ([-260 -140 -30 70]);
xlabel('X(m)')
ylabel('Y(m)')

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