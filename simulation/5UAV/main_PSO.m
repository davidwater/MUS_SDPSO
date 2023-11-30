%% 5無人機 無障礙物
clc;
clear;
close all;

%%起始點&目標點&速度&安全距離設定
xs1 =   -40; ys1 = -40; xg1 =  40; yg1 =  40;
xs2 =  40; ys2 = 40; xg2 =   -40; yg2 =  -40;
xs3 = 0; ys3 =   -40; xg3 =  0; yg3 =  40;
xs4 = -40; ys4 =   40; xg4 =  40; yg4 =  -40;
xs5 = 40; ys5 =   -40; xg5 =  -40; yg5 =  40;
l1_max = ((xs1 - xg1)^2 + (ys1 - yg1)^2)^(0.5); % UAV1 maximum flight distance
l2_max = ((xs2 - xg2)^2 + (ys2 - yg2)^2)^(0.5); % UAV2 maximum flight distance
l3_max = ((xs3 - xg3)^2 + (ys3 - yg3)^2)^(0.5); % UAV3 maximum flight distance
l4_max = ((xs4 - xg4)^2 + (ys4 - yg4)^2)^(0.5); % UAV3 maximum flight distance
l5_max = ((xs5 - xg5)^2 + (ys5 - yg5)^2)^(0.5); % UAV3 maximum flight distance

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
pun_coeff_8 = zeros(1,it);
pun_coeff_9 = zeros(1,it); 
pun_coeff_10 = zeros(1,it);
pun_coeff_11 = zeros(1,it);
pun_coeff_12 = zeros(1,it); 
pun_coeff_13 = zeros(1,it);
pun_coeff_14 = zeros(1,it);
pun_coeff_15 = zeros(1,it);
pun_coeff_16 = zeros(1,it);
F = zeros(3,it); F1 = zeros(1,it); F2 = zeros(1,it); F3 = zeros(1,it); % individual cost value
F_total = zeros(1,it); % total cost value
f_collision = zeros(10,it); % sub-cost value in 2
f_yaw = zeros(5,it); % sub-cost value in 3
w1=0.33;            % relative weight factor of cost funtion 1
w2=0.33;            % relative weight factor of cost funtion 2
w3=0.33;            % relative weight factor of cost funtion 3

%% PSO
%%初始目前位置&初速設定
xc1(1) = xs1; yc1(1) = ys1; vx1(1) = 0; vy1(1) = 0;
xc2(1) = xs2; yc2(1) = ys2; vx2(1) = 0; vy2(1) = 0;
xc3(1) = xs3; yc3(1) = ys3; vx3(1) = 0; vy3(1) = 0;
xc4(1) = xs4; yc4(1) = ys4; vx4(1) = 0; vy4(1) = 0;
xc5(1) = xs5; yc5(1) = ys5; vx5(1) = 0; vy5(1) = 0;

dn1(1) = ((xc1(1) - xg1)^2 + (yc1(1) - yg1)^2)^(0.5); % distance between UAV 1 and its goal
dn2(1) = ((xc2(1) - xg2)^2 + (yc2(1) - yg2)^2)^(0.5); % distance between UAV 2 and its goal
dn3(1) = ((xc3(1) - xg3)^2 + (yc3(1) - yg3)^2)^(0.5); % distance between UAV 3 and its goal
dn4(1) = ((xc4(1) - xg4)^2 + (yc4(1) - yg4)^2)^(0.5); % distance between UAV 4 and its goal
dn5(1) = ((xc5(1) - xg5)^2 + (yc5(1) - yg5)^2)^(0.5); % distance between UAV 5 and its goal

d12(1) = ((xc1(1) - xc2(1))^2 + (yc1(1) - yc2(1))^2)^(0.5); % distance between UAV 1 and UAV 2
d13(1) = ((xc1(1) - xc3(1))^2 + (yc1(1) - yc3(1))^2)^(0.5); % distance between UAV 1 and UAV 3
d14(1) = ((xc1(1) - xc4(1))^2 + (yc1(1) - yc4(1))^2)^(0.5); % distance between UAV 1 and UAV 4
d15(1) = ((xc1(1) - xc5(1))^2 + (yc1(1) - yc5(1))^2)^(0.5); % distance between UAV 1 and UAV 5
d23(1) = ((xc2(1) - xc3(1))^2 + (yc2(1) - yc3(1))^2)^(0.5); % distance between UAV 2 and UAV 3
d24(1) = ((xc2(1) - xc4(1))^2 + (yc2(1) - yc4(1))^2)^(0.5); % distance between UAV 2 and UAV 4
d25(1) = ((xc2(1) - xc5(1))^2 + (yc2(1) - yc5(1))^2)^(0.5); % distance between UAV 2 and UAV 5
d34(1) = ((xc3(1) - xc4(1))^2 + (yc3(1) - yc4(1))^2)^(0.5); % distance between UAV 3 and UAV 4
d35(1) = ((xc3(1) - xc5(1))^2 + (yc3(1) - yc5(1))^2)^(0.5); % distance between UAV 3 and UAV 5
d45(1) = ((xc4(1) - xc5(1))^2 + (yc4(1) - yc5(1))^2)^(0.5); % distance between UAV 4 and UAV 5

d1 = 0; % UAV 1 distance between current position and last position
d2 = 0; % UAV 2 distance between current position and last position
d3 = 0; % UAV 3 distance between current position and last position
d4 = 0; % UAV 4 distance between current position and last position
d5 = 0; % UAV 5 distance between current position and last position
d_total = 0; % all UAVs total moving distance 

%%初始heading設定
h1(1) = heading((xg1-xs1),(yg1-ys1));
h2(1) = heading((xg2-xs2),(yg2-ys2));
h3(1) = heading((xg3-xs3),(yg3-ys3));
h4(1) = heading((xg4-xs4),(yg4-ys4));
h5(1) = heading((xg5-xs5),(yg5-ys5));

%avoid special case i.e. heading angle = 45*n degree
if (h1(1) == pi/4 || h1(1) == 0.75*pi || h1(1) == -pi/4 || h1(1) == -0.75*pi)
    xs1 = xs1 + 0.01;
elseif (h2(1) == pi/4 || h2(1) == 0.75*pi || h2(1) == -pi/4 || h2(1) == -0.75*pi)
    xs2 = xs2 + 0.01;
elseif (h3(1) == pi/4 || h3(1) == 0.75*pi || h3(1) == -pi/4 || h3(1) == -0.75*pi)
    xs3 = xs3 + 0.01;
elseif (h4(1) == pi/4 || h4(1) == 0.75*pi || h4(1) == -pi/4 || h4(1) == -0.75*pi)
    xs4 = xs4 + 0.01;
elseif (h5(1) == pi/4 || h5(1) == 0.75*pi || h5(1) == -pi/4 || h5(1) == -0.75*pi)
    xs5 = xs5 + 0.01;
end

mindist_12 = inf; mindist_13 = inf; mindist_14 = inf; mindist_15 = inf; mindist_23 = inf; mindist_24 = inf; mindist_25 = inf; mindist_34 = inf; mindist_35 = inf; mindist_45 =inf;
mindist_t_12 = 0; mindist_t_13 = 0; mindist_t_14 = 0; mindist_t_15 = 0; mindist_t_23 = 0; mindist_t_24 = 0; mindist_t_25 = 0; mindist_t_34 = 0; mindist_t_35 = 0; mindist_t_45 = 0;

% PSO算出下一航點，飛往下一航點，判斷是否抵達，繼續計算下一航點直到抵達目標點
tic
for i = 1:it
    
    while i == 1
        [vx1(i+1), vy1(i+1), vx2(i+1), vy2(i+1), vx3(i+1), vy3(i+1), vx4(i+1), vy4(i+1), vx5(i+1), vy5(i+1)] = ...
        PSO(xc1(i),yc1(i),xs1,ys1,xg1,yg1,l1_max,vx1(i),vy1(i),h1(i),h1(i), ...
            xc2(i),yc2(i),xs2,ys2,xg2,yg2,l2_max,vx2(i),vy2(i),h2(i),h2(i), ...
            xc3(i),yc3(i),xs3,ys3,xg3,yg3,l3_max,vx3(i),vy3(i),h3(i),h3(i), ...
            xc4(i),yc4(i),xs4,ys4,xg4,yg4,l4_max,vx4(i),vy4(i),h4(i),h4(i), ...
            xc5(i),yc5(i),xs5,ys5,xg5,yg5,l5_max,vx5(i),vy5(i),h5(i),h5(i), ...
            ds);
        wvx1(i) = cos(h1(i))*vx1(i+1)+sin(h1(i))*vy1(i+1);
        wvy1(i) = sin(h1(i))*vx1(i+1)-cos(h1(i))*vy1(i+1);
        wvx2(i) = cos(h2(i))*vx2(i+1)+sin(h2(i))*vy2(i+1);
        wvy2(i) = sin(h2(i))*vx2(i+1)-cos(h2(i))*vy2(i+1);
        wvx3(i) = cos(h3(i))*vx3(i+1)+sin(h3(i))*vy3(i+1);
        wvy3(i) = sin(h3(i))*vx3(i+1)-cos(h3(i))*vy3(i+1);
        wvx4(i) = cos(h4(i))*vx4(i+1)+sin(h4(i))*vy4(i+1);
        wvy4(i) = sin(h4(i))*vx4(i+1)-cos(h4(i))*vy4(i+1);
        wvx5(i) = cos(h5(i))*vx5(i+1)+sin(h5(i))*vy5(i+1);
        wvy5(i) = sin(h5(i))*vx5(i+1)-cos(h5(i))*vy5(i+1);
        xc1(i+1) = xc1(i) + wvx1(i)*dt;
        yc1(i+1) = yc1(i) + wvy1(i)*dt;
        xc2(i+1) = xc2(i) + wvx2(i)*dt;
        yc2(i+1) = yc2(i) + wvy2(i)*dt;
        xc3(i+1) = xc3(i) + wvx3(i)*dt;
        yc3(i+1) = yc3(i) + wvy3(i)*dt;
        xc4(i+1) = xc4(i) + wvx4(i)*dt;
        yc4(i+1) = yc4(i) + wvy4(i)*dt;
        xc5(i+1) = xc5(i) + wvx5(i)*dt;
        yc5(i+1) = yc5(i) + wvy5(i)*dt;
        h1(i+1) = heading((xg1-xc1(i)),(yg1-yc1(i)));
        h2(i+1) = heading((xg2-xc2(i)),(yg2-yc2(i)));
        h3(i+1) = heading((xg3-xc3(i)),(yg3-yc3(i)));
        h4(i+1) = heading((xg4-xc4(i)),(yg4-yc4(i)));
        h5(i+1) = heading((xg5-xc5(i)),(yg5-yc5(i)));
        d12(i) = ((xc1(i) - xc2(i))^2 + (yc1(i) - yc2(i))^2)^(0.5);
        d13(i) = ((xc1(i) - xc3(i))^2 + (yc1(i) - yc3(i))^2)^(0.5);
        d14(i) = ((xc1(i) - xc4(i))^2 + (yc1(i) - yc4(i))^2)^(0.5);
        d15(i) = ((xc1(i) - xc5(i))^2 + (yc1(i) - yc5(i))^2)^(0.5);
        d23(i) = ((xc2(i) - xc3(i))^2 + (yc2(i) - yc3(i))^2)^(0.5);
        d24(i) = ((xc2(i) - xc4(i))^2 + (yc2(i) - yc4(i))^2)^(0.5);
        d25(i) = ((xc2(i) - xc5(i))^2 + (yc2(i) - yc5(i))^2)^(0.5);
        d34(i) = ((xc3(i) - xc4(i))^2 + (yc3(i) - yc4(i))^2)^(0.5);
        d35(i) = ((xc3(i) - xc5(i))^2 + (yc3(i) - yc5(i))^2)^(0.5);
        d45(i) = ((xc4(i) - xc5(i))^2 + (yc4(i) - yc5(i))^2)^(0.5);
        dn1(i) = ((xc1(i) - xg1)^2 + (yc1(i) - yg1)^2)^(0.5);
        dn2(i) = ((xc2(i) - xg2)^2 + (yc2(i) - yg2)^2)^(0.5);
        dn3(i) = ((xc3(i) - xg3)^2 + (yc3(i) - yg3)^2)^(0.5);
        dn4(i) = ((xc4(i) - xg4)^2 + (yc4(i) - yg4)^2)^(0.5); 
        dn5(i) = ((xc5(i) - xg5)^2 + (yc5(i) - yg5)^2)^(0.5); 
        disp([' xc1 = ' num2str(xc1(i+1)) ' yc1 = ' num2str(yc1(i+1)) ' xc2 = ' num2str(xc2(i+1)) ' yc2 = ' num2str(yc2(i+1)) ...
            ' xc3 = ' num2str(xc3(i+1)) ' yc3 = ' num2str(yc3(i+1)) ' xc4 = ' num2str(xc4(i+1)) ' yc4 = ' num2str(yc4(i+1)) ...
            ' xc5 = ' num2str(xc5(i+1)) ' yc5 = ' num2str(yc5(i+1)) ]);
        i = i+1;
    end

    [vx1(i+1), vy1(i+1), vx2(i+1), vy2(i+1), vx3(i+1), vy3(i+1), vx4(i+1), vy4(i+1), vx5(i+1), vy5(i+1)] = ...
        PSO(xc1(i),yc1(i),xs1,ys1,xg1,yg1,l1_max,vx1(i),vy1(i),h1(i),h1(i-1), ...
            xc2(i),yc2(i),xs2,ys2,xg2,yg2,l2_max,vx2(i),vy2(i),h2(i),h2(i-1), ...
            xc3(i),yc3(i),xs3,ys3,xg3,yg3,l3_max,vx3(i),vy3(i),h3(i),h3(i-1), ...
            xc4(i),yc4(i),xs4,ys4,xg4,yg4,l4_max,vx4(i),vy4(i),h4(i),h4(i-1), ...
            xc5(i),yc5(i),xs5,ys5,xg5,yg5,l5_max,vx5(i),vy5(i),h5(i),h5(i-1), ...
            ds);
    wvx1(i) = cos(h1(i))*vx1(i+1)+sin(h1(i))*vy1(i+1);
    wvy1(i) = sin(h1(i))*vx1(i+1)-cos(h1(i))*vy1(i+1);
    wvx2(i) = cos(h2(i))*vx2(i+1)+sin(h2(i))*vy2(i+1);
    wvy2(i) = sin(h2(i))*vx2(i+1)-cos(h2(i))*vy2(i+1);
    wvx3(i) = cos(h3(i))*vx3(i+1)+sin(h3(i))*vy3(i+1);
    wvy3(i) = sin(h3(i))*vx3(i+1)-cos(h3(i))*vy3(i+1);
    wvx4(i) = cos(h4(i))*vx4(i+1)+sin(h4(i))*vy4(i+1);
    wvy4(i) = sin(h4(i))*vx4(i+1)-cos(h4(i))*vy4(i+1);
    wvx5(i) = cos(h5(i))*vx5(i+1)+sin(h5(i))*vy5(i+1);
    wvy5(i) = sin(h5(i))*vx5(i+1)-cos(h5(i))*vy5(i+1);

    if ((xg1 - xc1(i))^2 + (yg1 - yc1(i))^2)^(0.5) <= df
    xc1(i+1) = xg1;
    yc1(i+1) = yg1;
    else
        xc1(i+1) = xc1(i) + wvx1(i)*dt;
        yc1(i+1) = yc1(i) + wvy1(i)*dt;
    end

    if ((xg2 - xc2(i))^2 + (yg2 - yc2(i))^2)^(0.5) <= df
        xc2(i+1) = xg2;
        yc2(i+1) = yg2;
    else
        xc2(i+1) = xc2(i) + wvx2(i)*dt;
        yc2(i+1) = yc2(i) + wvy2(i)*dt;
    end

    if ((xg3 - xc3(i))^2 + (yg3 - yc3(i))^2)^(0.5) <= df
        xc3(i+1) = xg3;
        yc3(i+1) = yg3;
    else
        xc3(i+1) = xc3(i) + wvx3(i)*dt;
        yc3(i+1) = yc3(i) + wvy3(i)*dt;
    end

    if ((xg4 - xc4(i))^2 + (yg4 - yc4(i))^2)^(0.5) <= df
        xc4(i+1) = xg4;
        yc4(i+1) = yg4;
    else
        xc4(i+1) = xc4(i) + wvx4(i)*dt;
        yc4(i+1) = yc4(i) + wvy4(i)*dt;
    end

    if ((xg5 - xc5(i))^2 + (yg5 - yc5(i))^2)^(0.5) <= df
        xc5(i+1) = xg5;
        yc5(i+1) = yg5;
    else
        xc5(i+1) = xc5(i) + wvx5(i)*dt;
        yc5(i+1) = yc5(i) + wvy5(i)*dt;
    end
    h1(i+1) = heading((xg1-xc1(i)),(yg1-yc1(i)));
    h2(i+1) = heading((xg2-xc2(i)),(yg2-yc2(i)));
    h3(i+1) = heading((xg3-xc3(i)),(yg3-yc3(i)));
    h4(i+1) = heading((xg4-xc4(i)),(yg4-yc4(i)));
    h5(i+1) = heading((xg5-xc5(i)),(yg5-yc5(i)));
    d12(i) = ((xc1(i) - xc2(i))^2 + (yc1(i) - yc2(i))^2)^(0.5);
    d13(i) = ((xc1(i) - xc3(i))^2 + (yc1(i) - yc3(i))^2)^(0.5);
    d14(i) = ((xc1(i) - xc4(i))^2 + (yc1(i) - yc4(i))^2)^(0.5);
    d15(i) = ((xc1(i) - xc5(i))^2 + (yc1(i) - yc5(i))^2)^(0.5);
    d23(i) = ((xc2(i) - xc3(i))^2 + (yc2(i) - yc3(i))^2)^(0.5);
    d24(i) = ((xc2(i) - xc4(i))^2 + (yc2(i) - yc4(i))^2)^(0.5);
    d25(i) = ((xc2(i) - xc5(i))^2 + (yc2(i) - yc5(i))^2)^(0.5);
    d34(i) = ((xc3(i) - xc4(i))^2 + (yc3(i) - yc4(i))^2)^(0.5);
    d35(i) = ((xc3(i) - xc5(i))^2 + (yc3(i) - yc5(i))^2)^(0.5);
    d45(i) = ((xc4(i) - xc5(i))^2 + (yc4(i) - yc5(i))^2)^(0.5);
    dn1(i) = ((xc1(i) - xg1)^2 + (yc1(i) - yg1)^2)^(0.5);
    dn2(i) = ((xc2(i) - xg2)^2 + (yc2(i) - yg2)^2)^(0.5);
    dn3(i) = ((xc3(i) - xg3)^2 + (yc3(i) - yg3)^2)^(0.5);
    dn4(i) = ((xc4(i) - xg4)^2 + (yc4(i) - yg4)^2)^(0.5); 
    dn5(i) = ((xc5(i) - xg5)^2 + (yc5(i) - yg5)^2)^(0.5);
    d1 = ((xc1(i) - xc1(i-1))^2 + (yc1(i) - yc1(i-1))^2)^(0.5);
    d2 = ((xc2(i) - xc2(i-1))^2 + (yc2(i) - yc2(i-1))^2)^(0.5);
    d3 = ((xc3(i) - xc3(i-1))^2 + (yc3(i) - yc3(i-1))^2)^(0.5);
    d4 = ((xc4(i) - xc4(i-1))^2 + (yc4(i) - yc4(i-1))^2)^(0.5);
    d5 = ((xc5(i) - xc5(i-1))^2 + (yc5(i) - yc5(i-1))^2)^(0.5);
    d_total = d_total + (d1 + d2 + d3 + d4 + d5);
    disp(['iteration = ' num2str(i) ' xc1 = ' num2str(xc1(i+1)) ' yc1 = ' num2str(yc1(i+1)) ' xc2 = ' num2str(xc2(i+1)) ' yc2 = ' num2str(yc2(i+1)) ...
        ' xc3 = ' num2str(xc3(i+1)) ' yc3 = ' num2str(yc3(i+1)) ' xc4 = ' num2str(xc4(i+1)) ' yc4 = ' num2str(yc4(i+1)) ...
        ' xc5 = ' num2str(xc5(i+1)) ' yc5 = ' num2str(yc5(i+1)) ]);

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
    if (h4(i+1) == pi/4 || h4(i+1) == 0.75*pi || h4(i+1) == -pi/4 || h4(i+1) == -0.75*pi)
        xc4(i+1) = xc4(i+1) + 0.01;
    end
    if (h5(i+1) == pi/4 || h5(i+1) == 0.75*pi || h5(i+1) == -pi/4 || h5(i+1) == -0.75*pi)
        xc5(i+1) = xc5(i+1) + 0.01;
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
    if d14(i) < mindist_14
        mindist_14 = d14(i);
        mindist_t_14 = i;
    end
    if d15(i) < mindist_15
        mindist_15 = d15(i);
        mindist_t_15 = i;
    end
    if d23(i) < mindist_23
        mindist_23 = d23(i);
        mindist_t_23 = i;
    end
    if d24(i) < mindist_24
        mindist_24 = d24(i);
        mindist_t_24 = i;
    end
    if d25(i) < mindist_25
        mindist_25 = d25(i);
        mindist_t_25 = i;
    end
    if d34(i) < mindist_34
        mindist_34 = d34(i);
        mindist_t_34 = i;
    end
    if d35(i) < mindist_35
        mindist_35 = d35(i);
        mindist_t_35 = i;
    end
    if d45(i) < mindist_45
        mindist_45 = d45(i);
        mindist_t_45 = i;
    end
    
    %%computing cost value
    % F1
    pun_coeff_1(1,i) = 1000*exp(1/(dn1(i)+dn2(i)+dn3(i)+dn4(i)+dn5(i)));
    F(1, i) = pun_coeff_1(1,i)*((dn1(i)*dn1(i)/l1_max) + (dn2(i)*dn2(i)/l2_max) + (dn3(i)*dn3(i)/l3_max) + (dn4(i)*dn4(i)/l4_max) + (dn5(i)*dn5(i)/l5_max));
    F1(i) = F(1,i);

    % F2
    pun_coeff_2(1,i) = 200 * exp(-((d12(i)./d12(1)).^2));
    pun_coeff_3(1,i) = 200 * exp(-((d13(i)./d13(1)).^2));
    pun_coeff_4(1,i) = 200 * exp(-((d14(i)./d14(1)).^2));
    pun_coeff_5(1,i) = 200 * exp(-((d15(i)./d15(1)).^2));
    pun_coeff_6(1,i) = 200 * exp(-((d23(i)./d23(1)).^2));
    pun_coeff_7(1,i) = 200 * exp(-((d24(i)./d24(1)).^2));
    pun_coeff_8(1,i) = 200 * exp(-((d25(i)./d25(1)).^2));
    pun_coeff_9(1,i) = 200 * exp(-((d34(i)./d34(1)).^2));
    pun_coeff_10(1,i) = 200 * exp(-((d35(i)./d35(1)).^2));
    pun_coeff_11(1,i) = 200 * exp(-((d45(i)./d45(1)).^2));

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

    if d14 >= ds
        f_collision(3,i) = 0;
    else
        f_collision(3,i) = (1/d14(i))*pun_coeff_4(1,i);
    end

    if d15 >= ds
        f_collision(4,i) = 0;
    else
        f_collision(4,i) = (1/d15(i))*pun_coeff_5(1,i);
    end

    if d23 >= ds
        f_collision(5,i) = 0;
    else
        f_collision(5,i) = (1/d23(i))*pun_coeff_6(1,i);
    end

    if d24 >= ds
        f_collision(6,i) = 0;
    else
        f_collision(6,i) = (1/d24(i))*pun_coeff_7(1,i);
    end

    if d25 >= ds
        f_collision(7,i) = 0;
    else
        f_collision(7,i) = (1/d25(i))*pun_coeff_8(1,i);
    end

    if d34 >= ds
        f_collision(8,i) = 0;
    else
        f_collision(8,i) = (1/d34(i))*pun_coeff_9(1,i);
    end

    if d35 >= ds
        f_collision(9,i) = 0;
    else
        f_collision(9,i) = (1/d35(i))*pun_coeff_10(1,i);
    end

    if d45 >= ds
        f_collision(10,i) = 0;
    else
        f_collision(10,i) = (1/d45(i))*pun_coeff_11(1,i);
    end

    

    F(2,i) = f_collision(1,i) + f_collision(2,i) + f_collision(3,i) + f_collision(4,i) + f_collision(5,i) + f_collision(6,i) + ...
    f_collision(7,i) + f_collision(8,i) + f_collision(9,i) + f_collision(10,i);
    F2(i) = F(2,i);

    % F3
    pun_coeff_12(1,i) = 100 * exp(-(((h1(i)-h1(i-1))./(pi/2)).^2));
    pun_coeff_13(1,i) = 100 * exp(-(((h2(i)-h2(i-1))./(pi/2)).^2));
    pun_coeff_14(1,i) = 100 * exp(-(((h3(i)-h3(i-1))./(pi/2)).^2));
    pun_coeff_15(1,i) = 100 * exp(-(((h4(i)-h4(i-1))./(pi/2)).^2));
    pun_coeff_16(1,i) = 100 * exp(-(((h5(i)-h5(i-1))./(pi/2)).^2));

    if abs(h1(i) - h1(i-1)) > (pi/2)
        f_yaw(1,i) = h1(i) * pun_coeff_12(1,i);
    else
        f_yaw(1,i) = 0;
    end
    
    if abs(h2(i) - h2(i-1)) > (pi/2)
        f_yaw(2,i) = h2(i) * pun_coeff_13(1,i);
    else
        f_yaw(2,i) = 0;
    end

    if abs(h3(i) - h3(i-1)) > (pi/2)
        f_yaw(3,i) = h3(i) * pun_coeff_14(1,i);
    else
        f_yaw(3,i) = 0;
    end

    if abs(h4(i) - h4(i-1)) > (pi/2)
        f_yaw(4,i) = h4(i) * pun_coeff_15(1,i);
    else
        f_yaw(4,i) = 0;
    end

    if abs(h5(i) - h5(i-1)) > (pi/2)
        f_yaw(5,i) = h5(i) * pun_coeff_16(1,i);
    else
        f_yaw(5,i) = 0;
    end

    F(3,i) = f_yaw(1,i) + f_yaw(2,i) + f_yaw(3,i) + f_yaw(4,i) + f_yaw(5,i);
    F3(i) = F(3,i);

    F_total(i) = w1*F(1,i) + w2*F(2,i) + w3*F(3,i);
    
    % check if UAVs arrive goals
    if ((xg1 - xc1(i+1))^2 + (yg1 - yc1(i+1))^2)^(0.5) <= df &&... 
       ((xg2 - xc2(i+1))^2 + (yg2 - yc2(i+1))^2)^(0.5) <= df &&... 
       ((xg3 - xc3(i+1))^2 + (yg3 - yc3(i+1))^2)^(0.5) <= df &&...
       ((xg4 - xc4(i+1))^2 + (yg4 - yc4(i+1))^2)^(0.5) <= df &&...
       ((xg5 - xc5(i+1))^2 + (yg5 - yc5(i+1))^2)^(0.5) <= df

        disp('mission completed');
        break; 
    end

end
t_PSO=toc;

%%畫出路徑
figure('Name','3 UAVs PSO')
xc1 = [xc1 xg1]; yc1 = [yc1 yg1];
xc2 = [xc2 xg2]; yc2 = [yc2 yg2];
xc3 = [xc3 xg3]; yc3 = [yc3 yg3];
xc4 = [xc4 xg4]; yc4 = [yc4 yg4];
xc5 = [xc5 xg5]; yc5 = [yc5 yg5];
plot(xc1,yc1,'LineStyle','-','marker','d','color','#4DBEEE','MarkerSize',3); hold on;
plot(xc2,yc2,'LineStyle','-','marker','d','color','#EDB120','MarkerSize',3); hold on;
plot(xc3,yc3,'LineStyle','-','marker','d','color','#7E2F8E','MarkerSize',3); hold on;
plot(xc4,yc4,'LineStyle','-','marker','d','color','#0072BD','MarkerSize',3); hold on;
plot(xc5,yc5,'LineStyle','-','marker','d','color','#A2142F','MarkerSize',3); hold on;
plot(xs1,ys1,'marker','o','color','#4DBEEE','MarkerFaceColor','#4DBEEE','MarkerSize',6); hold on;
plot(xs2,ys2,'marker','o','color','#EDB120','MarkerFaceColor','#EDB120','MarkerSize',6); hold on;
plot(xs3,ys3,'marker','o','color','#7E2F8E','MarkerFaceColor','#7E2F8E','MarkerSize',6); hold on;
plot(xs4,ys4,'marker','o','color','#EDB120','MarkerFaceColor','#0072BD','MarkerSize',6); hold on;
plot(xs5,ys5,'marker','o','color','#7E2F8E','MarkerFaceColor','#A2142F','MarkerSize',6); hold on;
plot(xg1,yg1,'marker','^','color','#4DBEEE','MarkerFaceColor','#4DBEEE','MarkerSize',6); hold on;
plot(xg2,yg2,'marker','^','color','#EDB120','MarkerFaceColor','#EDB120','MarkerSize',6); hold on;
plot(xg3,yg3,'marker','^','color','#7E2F8E','MarkerFaceColor','#7E2F8E','MarkerSize',6); hold on;
plot(xg4,yg4,'marker','^','color','#EDB120','MarkerFaceColor','#0072BD','MarkerSize',6); hold on;
plot(xg5,yg5,'marker','^','color','#7E2F8E','MarkerFaceColor','#A2142F','MarkerSize',6); hold on;
%legend('UAV1 path','UAV2 path','UAV3 path','UAV4 path','UAV5 path','Location','northwest');
grid on; axis equal; axis ([-50 50 -50 50]);
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