clc
clear all
close all
%robot parameters
l = 0.12;
m = 10.1;
I = 0.13;

layer = 14;%number of circles of the formation pattern

time =100;tg = 0.01;t = 0:tg:time;

%define the disturbance
d1 = zeros(size(t,2),layer*4,layer);%disturbance on x axis
d2 = zeros(size(t,2),layer*4,layer);%disturbance on y axis
d1(:,1,1) = 2*sin(t).*exp(-0.2*t);
d2(:,1,1) = 2*sin(t).*exp(-0.2*t);

%gains and weights
kp0 = 0.7;%0.3 to get middle panel
kv0 = 1;%1 to get middle panel
kp = 0.035;%1.3 to get middle panel

veq = 2;%constant linear speed throughout
omega = pi/20;%constant angular velocity during maneuvering
d = 11;%delay=(d-1)*tg

%creat leader position/angle vectors
px = zeros(size(t,2),1);
py = zeros(size(t,2),1);
pxd = zeros(size(t,2),1);
pyd = zeros(size(t,2),1);

%define virtual leader dynamics. The initial states are
%definded in the time interval [0,d*tg] and equal to desired state at 0s.
vx(1:d,1) = veq*sin(omega*zeros(d,1));
vy(1:d,1) = veq*cos(omega*zeros(d,1));
ax(1:d,1) = zeros(d,1);
ay(1:d,1) = zeros(d,1);

ax(1+d:500+d,1) = veq*omega*cos(omega*t(1:500));
ax(501+d:d+2500,1) = 0;
ax(2501+d:4000+d,1) = veq*omega*cos(pi/4+omega*t(1:1500));
ax(4001+d:d+15000,1) = 0;

ay(1+d:500+d,1) = -veq*omega*sin(omega*t(1:500));
ay(501+d:d+2500,1) = 0;
ay(2501+d:4000+d,1) = -veq*omega*sin(pi/4+omega*t(1:1500));
ay(4001+d:d+15000,1) = 0;

for i = d+1:size(t,2)
    vx(i,1) = vx(i-1,1)+ax(i-1,1)*tg;
    vy(i,1) = vy(i-1,1)+ay(i-1,1)*tg;
end
%% the leader position and desired followers position
for i = d+1:size(t,2)
    px(i,1) = px(i-1,1)+(vx(i-1,1)+vx(i,1))/2*tg;
    py(i,1) = py(i-1,1)+(vy(i-1,1)+vy(i,1))/2*tg;
end
laypxd = zeros(size(t,2),4*layer,layer);
laypyd = zeros(size(t,2),4*layer,layer);
laypx = zeros(size(t,2),4*layer,layer);
laypy = zeros(size(t,2),4*layer,layer);
layvx = zeros(size(t,2),4*layer,layer);
layvy = zeros(size(t,2),4*layer,layer);
for j = 1:layer
    for i=1:4*j
        if i/4<=1
            laypxd(:,i,j) = px(:,1)+j*3*cos((i-1)/2*pi);
            laypyd(:,i,j) = py(:,1)+j*3*sin((i-1)/2*pi);
        else
            laypxd(:,i,j)=px(:,1)+j*3*cos(pi/2/j*fix((i-0.1)/4)+(i-(j-1)*4-1)/2*pi);
            laypyd(:,i,j)=py(:,1)+j*3*sin(pi/2/j*fix((i-0.1)/4)+(i-(j-1)*4-1)/2*pi);
        end
    end
end
%set the initial condition(position/velocity) to be at the desired state.
for j = 1:layer
    for i=1:4*j
        laypx(1:d,i,j)=laypxd(1:d,i,j);
        laypy(1:d,i,j)=laypyd(1:d,i,j);
        layvx(1:d,i,j)=vx(1:d,1);
        layvy(1:d,i,j)=vy(1:d,1); 
    end
end
pxd(:,1) = px(:,1);
pyd(:,1) = py(:,1);
positiondev = zeros(size(t,2),4*layer,layer);
%% define followers dynamics in normal disturbance
neighbour_x = zeros(size(t,2),layer*4,layer);
neighbour_y = zeros(size(t,2),layer*4,layer);
neighbour_xd = zeros(size(t,2),layer*4,layer);
neighbour_yd = zeros(size(t,2),layer*4,layer);
theta = pi/2*ones(size(t,2),layer*4,layer);
d11 = zeros(d+size(t,2),4*layer,layer);
d11(d+1:end,:,:) = d1;
d22 = zeros(d+size(t,2),4*layer,layer);
d22(d+1:end,:,:) = d2;
% each agents is connected to the agent immediate ahead and behind them in same circle and one closest from inner circle.
for k = d+1:size(t,2)
   for j = 1:layer
      for i = 1:4*j
         if j==1 %layer 1
             if i>=2 && i<=3
                neighbour_x(k-1,i,1) = laypx(k-d,i-1,1)+laypx(k-d,i+1,1)+laypx(k-d,i,1) - 3*laypx(k-d,i,1);
                neighbour_y(k-1,i,1) = laypy(k-d,i-1,1)+laypy(k-d,i+1,1)+laypy(k-d,i,1) - 3*laypy(k-d,i,1);
                neighbour_xd(k-1,i,1) = laypxd(k-d,i-1,1)+laypxd(k-d,i+1,1)+laypxd(k-d,i,1) - 3*laypxd(k-d,i,1);
                neighbour_yd(k-1,i,1) = laypyd(k-d,i-1,1)+laypyd(k-d,i+1,1)+laypyd(k-d,i,1) - 3*laypyd(k-d,i,1);
             end
             neighbour_x(k-1,1,1) = laypx(k-d,2,1)+laypx(k-d,4,1)+laypx(k-d,1,1)- 3*laypx(k-d,1,1);
             neighbour_y(k-1,1,1) = laypy(k-d,2,1)+laypy(k-d,4,1)+laypy(k-d,1,1)- 3*laypy(k-d,1,1);
             neighbour_xd(k-1,1,1) = laypxd(k-d,2,1)+laypxd(k-d,4,1)+laypxd(k-d,1,1)- 3*laypxd(k-d,1,1);
             neighbour_yd(k-1,1,1) = laypyd(k-d,2,1)+laypyd(k-d,4,1)+laypyd(k-d,1,1)- 3*laypyd(k-d,1,1);
             neighbour_x(k-1,4,1) = laypx(k-d,1,1)+laypx(k-d,3,1)+laypx(k-d,4,1)- 3*laypx(k-d,4,1);
             neighbour_y(k-1,4,1) = laypy(k-d,1,1)+laypy(k-d,3,1)+laypy(k-d,4,1)- 3*laypy(k-d,4,1);
             neighbour_xd(k-1,4,1) = laypxd(k-d,1,1)+laypxd(k-d,3,1)+laypxd(k-d,4,1)- 3*laypxd(k-d,4,1);
             neighbour_yd(k-1,4,1) = laypyd(k-d,1,1)+laypyd(k-d,3,1)+laypyd(k-d,4,1)- 3*laypyd(k-d,4,1);
         elseif j==2
             if i>=2 && i<=4
                neighbour_x(k-1,i,2) = laypx(k-d,i+3,2)+laypx(k-d,i+4,2)+laypx(k-d,i,1)-3*laypx(k-d,i,2);
                neighbour_y(k-1,i,2) = laypy(k-d,i+3,2)+laypy(k-d,i+4,2)+laypy(k-d,i,1)-3*laypy(k-d,i,2);
                neighbour_xd(k-1,i,2) = laypxd(k-d,i+3,2)+laypxd(k-d,i+4,2)+laypxd(k-d,i,1)-3*laypxd(k-d,i,2);
                neighbour_yd(k-1,i,2) = laypyd(k-d,i+3,2)+laypyd(k-d,i+4,2)+laypyd(k-d,i,1)-3*laypyd(k-d,i,2);
             elseif i>=5 && i<=7
                neighbour_x(k-1,i,2) = laypx(k-d,i-3,2)+laypx(k-d,i-4,2)+laypx(k-d,i-4,1)-3*laypx(k-d,i,2);
                neighbour_y(k-1,i,2) = laypy(k-d,i-3,2)+laypy(k-d,i-4,2)+laypy(k-d,i-4,1)-3*laypy(k-d,i,2);
                neighbour_xd(k-1,i,2) = laypxd(k-d,i-3,2)+laypxd(k-d,i-4,2)+laypxd(k-d,i-4,1)-3*laypxd(k-d,i,2);
                neighbour_yd(k-1,i,2) = laypyd(k-d,i-3,2)+laypyd(k-d,i-4,2)+laypyd(k-d,i-4,1)-3*laypyd(k-d,i,2);
             end
             neighbour_x(k-1,1,2) = laypx(k-d,5,2)+laypx(k-d,8,2)+laypx(k-d,1,1)-3*laypx(k-d,1,2);
             neighbour_y(k-1,1,2) = laypy(k-d,5,2)+laypy(k-d,8,2)+laypy(k-d,1,1)-3*laypy(k-d,1,2);
             neighbour_xd(k-1,1,2) = laypxd(k-d,5,2)+laypxd(k-d,8,2)+laypxd(k-d,1,1)-3*laypxd(k-d,1,2);
             neighbour_yd(k-1,1,2) = laypyd(k-d,5,2)+laypyd(k-d,8,2)+laypyd(k-d,1,1)-3*laypyd(k-d,1,2);
             neighbour_x(k-1,8,2) = laypx(k-d,1,2)+laypx(k-d,4,2)+laypx(k-d,4,1)-3*laypx(k-d,8,2);
             neighbour_y(k-1,8,2) = laypy(k-d,1,2)+laypy(k-d,4,2)+laypy(k-d,4,1)-3*laypy(k-d,8,2);
             neighbour_xd(k-1,8,2) = laypxd(k-d,1,2)+laypxd(k-d,4,2)+laypxd(k-d,4,1)-3*laypxd(k-d,8,2);
             neighbour_yd(k-1,8,2) = laypyd(k-d,1,2)+laypyd(k-d,4,2)+laypyd(k-d,4,1)-3*laypyd(k-d,8,2);
         elseif j==3
             if i>=2 && i<=4
                neighbour_x(k-1,i,3) = laypx(k-d,i+4,3)+laypx(k-d,i+7,3)+laypx(k-d,i,2)-3*laypx(k-d,i,3);
                neighbour_y(k-1,i,3) = laypy(k-d,i+4,3)+laypy(k-d,i+7,3)+laypy(k-d,i,2)-3*laypy(k-d,i,3);
                neighbour_xd(k-1,i,3) = laypxd(k-d,i+4,3)+laypxd(k-d,i+7,3)+laypxd(k-d,i,2)-3*laypxd(k-d,i,3);
                neighbour_yd(k-1,i,3) = laypyd(k-d,i+4,3)+laypyd(k-d,i+7,3)+laypyd(k-d,i,2)-3*laypyd(k-d,i,3);
             elseif i>=5 && i<=8
                neighbour_x(k-1,i,3) = laypx(k-d,i-4,3)+laypx(k-d,i+4,3)+laypx(k-d,i,2)-3*laypx(k-d,i,3);
                neighbour_y(k-1,i,3) = laypy(k-d,i-4,3)+laypy(k-d,i+4,3)+laypy(k-d,i,2)-3*laypy(k-d,i,3);
                neighbour_xd(k-1,i,3) = laypxd(k-d,i-4,3)+laypxd(k-d,i+4,3)+laypxd(k-d,i,2)-3*laypxd(k-d,i,3);
                neighbour_yd(k-1,i,3) = laypyd(k-d,i-4,3)+laypyd(k-d,i+4,3)+laypyd(k-d,i,2)-3*laypyd(k-d,i,3);
             elseif i>=9 && i<=11
                neighbour_x(k-1,i,3) = laypx(k-d,i-4,3)+laypx(k-d,i-7,3)+laypx(k-d,i-4,2)-3*laypx(k-d,i,3);
                neighbour_y(k-1,i,3) = laypy(k-d,i-4,3)+laypy(k-d,i-7,3)+laypy(k-d,i-4,2)-3*laypy(k-d,i,3);
                neighbour_xd(k-1,i,3) = laypxd(k-d,i-4,3)+laypxd(k-d,i-7,3)+laypxd(k-d,i-4,2)-3*laypxd(k-d,i,3);
                neighbour_yd(k-1,i,3) = laypyd(k-d,i-4,3)+laypyd(k-d,i-7,3)+laypyd(k-d,i-4,2)-3*laypyd(k-d,i,3);
             end
                neighbour_x(k-1,1,3) = laypx(k-d,5,3)+laypx(k-d,12,3)+laypx(k-d,1,2)-3*laypx(k-d,1,3);
                neighbour_y(k-1,1,3) = laypy(k-d,5,3)+laypy(k-d,12,3)+laypy(k-d,1,2)-3*laypy(k-d,1,3);
                neighbour_xd(k-1,1,3) = laypxd(k-d,5,3)+laypxd(k-d,12,3)+laypxd(k-d,1,2)-3*laypxd(k-d,1,3);
                neighbour_yd(k-1,1,3) = laypyd(k-d,5,3)+laypyd(k-d,12,3)+laypyd(k-d,1,2)-3*laypyd(k-d,1,3);
                neighbour_x(k-1,12,3) = laypx(k-d,1,3)+laypx(k-d,8,3)+laypx(k-d,8,2)-3*laypx(k-d,12,3);
                neighbour_y(k-1,12,3) = laypy(k-d,1,3)+laypy(k-d,8,3)+laypy(k-d,8,2)-3*laypy(k-d,12,3);
                neighbour_xd(k-1,12,3) = laypxd(k-d,1,3)+laypxd(k-d,8,3)+laypxd(k-d,8,2)-3*laypxd(k-d,12,3);
                neighbour_yd(k-1,12,3) = laypyd(k-d,1,3)+laypyd(k-d,8,3)+laypyd(k-d,8,2)-3*laypyd(k-d,12,3);
         elseif j==4
             if i>=2 && i<=4
                neighbour_x(k-1,i,4) = laypx(k-d,i+4,4)+laypx(k-d,i+11,4)+laypx(k-d,i,3)-3*laypx(k-d,i,4);
                neighbour_y(k-1,i,4) = laypy(k-d,i+4,4)+laypy(k-d,i+11,4)+laypy(k-d,i,3)-3*laypy(k-d,i,4);
                neighbour_xd(k-1,i,4) = laypxd(k-d,i+4,4)+laypxd(k-d,i+11,4)+laypxd(k-d,i,3)-3*laypxd(k-d,i,4);
                neighbour_yd(k-1,i,4) = laypyd(k-d,i+4,4)+laypyd(k-d,i+11,4)+laypyd(k-d,i,3)-3*laypyd(k-d,i,4);
             elseif i>=5 && i<=12
                neighbour_x(k-1,i,4) = laypx(k-d,i-4,4)+laypx(k-d,i+4,4)+laypx(k-d,i,3)-3*laypx(k-d,i,4);
                neighbour_y(k-1,i,4) = laypy(k-d,i-4,4)+laypy(k-d,i+4,4)+laypy(k-d,i,3)-3*laypy(k-d,i,4);
                neighbour_xd(k-1,i,4) = laypxd(k-d,i-4,4)+laypxd(k-d,i+4,4)+laypxd(k-d,i,3)-3*laypxd(k-d,i,4);
                neighbour_yd(k-1,i,4) = laypyd(k-d,i-4,4)+laypyd(k-d,i+4,4)+laypyd(k-d,i,3)-3*laypyd(k-d,i,4);
             elseif i>=13 && i<=15
                neighbour_x(k-1,i,4) = laypx(k-d,i-4,4)+laypx(k-d,i-11,4)+laypx(k-d,i-4,3)-3*laypx(k-d,i,4);
                neighbour_y(k-1,i,4) = laypy(k-d,i-4,4)+laypy(k-d,i-11,4)+laypy(k-d,i-4,3)-3*laypy(k-d,i,4);
                neighbour_xd(k-1,i,4) = laypxd(k-d,i-4,4)+laypxd(k-d,i-11,4)+laypxd(k-d,i-4,3)-3*laypxd(k-d,i,4);
                neighbour_yd(k-1,i,4) = laypyd(k-d,i-4,4)+laypyd(k-d,i-11,4)+laypyd(k-d,i-4,3)-3*laypyd(k-d,i,4);
             end
                neighbour_x(k-1,1,4) = laypx(k-d,5,4)+laypx(k-d,16,4)+laypx(k-d,1,3)-3*laypx(k-d,1,4);
                neighbour_y(k-1,1,4) = laypy(k-d,5,4)+laypy(k-d,16,4)+laypy(k-d,1,3)-3*laypy(k-d,1,4);
                neighbour_xd(k-1,1,4) = laypxd(k-d,5,4)+laypxd(k-d,16,4)+laypxd(k-d,1,3)-3*laypxd(k-d,1,4);
                neighbour_yd(k-1,1,4) = laypyd(k-d,5,4)+laypyd(k-d,16,4)+laypyd(k-d,1,3)-3*laypyd(k-d,1,4);
                neighbour_x(k-1,16,4) = laypx(k-d,1,4)+laypx(k-d,12,4)+laypx(k-d,12,3)-3*laypx(k-d,16,4);
                neighbour_y(k-1,16,4) = laypy(k-d,1,4)+laypy(k-d,12,4)+laypy(k-d,12,3)-3*laypy(k-d,16,4);
                neighbour_xd(k-1,16,4) = laypxd(k-d,1,4)+laypxd(k-d,12,4)+laypxd(k-d,12,3)-3*laypxd(k-d,16,4);
                neighbour_yd(k-1,16,4) = laypyd(k-d,1,4)+laypyd(k-d,12,4)+laypyd(k-d,12,3)-3*laypyd(k-d,16,4);
         elseif j==5
             if i>=2 && i<=4
                neighbour_x(k-1,i,5) = laypx(k-d,i+4,5)+laypx(k-d,i+15,5)+laypx(k-d,i,4)-3*laypx(k-d,i,5);
                neighbour_y(k-1,i,5) = laypy(k-d,i+4,5)+laypy(k-d,i+15,5)+laypy(k-d,i,4)-3*laypy(k-d,i,5);
                neighbour_xd(k-1,i,5) = laypxd(k-d,i+4,5)+laypxd(k-d,i+15,5)+laypxd(k-d,i,4)-3*laypxd(k-d,i,5);
                neighbour_yd(k-1,i,5) = laypyd(k-d,i+4,5)+laypyd(k-d,i+15,5)+laypyd(k-d,i,4)-3*laypyd(k-d,i,5);
             elseif i>=5 && i<=16
                neighbour_x(k-1,i,5) = laypx(k-d,i-4,5)+laypx(k-d,i+4,5)+laypx(k-d,i,4)-3*laypx(k-d,i,5);
                neighbour_y(k-1,i,5) = laypy(k-d,i-4,5)+laypy(k-d,i+4,5)+laypy(k-d,i,4)-3*laypy(k-d,i,5);
                neighbour_xd(k-1,i,5) = laypxd(k-d,i-4,5)+laypxd(k-d,i+4,5)+laypxd(k-d,i,4)-3*laypxd(k-d,i,5);
                neighbour_yd(k-1,i,5) = laypyd(k-d,i-4,5)+laypyd(k-d,i+4,5)+laypyd(k-d,i,4)-3*laypyd(k-d,i,5);
             elseif i>=17 && i<=19
                neighbour_x(k-1,i,5) = laypx(k-d,i-4,5)+laypx(k-d,i-15,5)+laypx(k-d,i-4,4)-3*laypx(k-d,i,5);
                neighbour_y(k-1,i,5) = laypy(k-d,i-4,5)+laypy(k-d,i-15,5)+laypy(k-d,i-4,4)-3*laypy(k-d,i,5);
                neighbour_xd(k-1,i,5) = laypxd(k-d,i-4,5)+laypxd(k-d,i-15,5)+laypxd(k-d,i-4,4)-3*laypxd(k-d,i,5);
                neighbour_yd(k-1,i,5) = laypyd(k-d,i-4,5)+laypyd(k-d,i-15,5)+laypyd(k-d,i-4,4)-3*laypyd(k-d,i,5);
             end
                neighbour_x(k-1,1,5) = laypx(k-d,5,5)+laypx(k-d,20,5)+laypx(k-d,1,4)-3*laypx(k-d,1,5);
                neighbour_y(k-1,1,5) = laypy(k-d,5,5)+laypy(k-d,20,5)+laypy(k-d,1,4)-3*laypy(k-d,1,5);
                neighbour_xd(k-1,1,5) = laypxd(k-d,5,5)+laypxd(k-d,20,5)+laypxd(k-d,1,4)-3*laypxd(k-d,1,5);
                neighbour_yd(k-1,1,5) = laypyd(k-d,5,5)+laypyd(k-d,20,5)+laypyd(k-d,1,4)-3*laypyd(k-d,1,5);
                neighbour_x(k-1,20,5) = laypx(k-d,1,5)+laypx(k-d,16,5)+laypx(k-d,16,4)-3*laypx(k-d,20,5);
                neighbour_y(k-1,20,5) = laypy(k-d,1,5)+laypy(k-d,16,5)+laypy(k-d,16,4)-3*laypy(k-d,20,5);
                neighbour_xd(k-1,20,5) = laypxd(k-d,1,5)+laypxd(k-d,16,5)+laypxd(k-d,16,4)-3*laypxd(k-d,20,5);
                neighbour_yd(k-1,20,5) = laypyd(k-d,1,5)+laypyd(k-d,16,5)+laypyd(k-d,16,4)-3*laypyd(k-d,20,5);
         elseif j==6
             if i>=2 && i<=4
                neighbour_x(k-1,i,6) = laypx(k-d,i+4,6)+laypx(k-d,i+19,6)+laypx(k-d,i,5)-3*laypx(k-d,i,6);
                neighbour_y(k-1,i,6) = laypy(k-d,i+4,6)+laypy(k-d,i+19,6)+laypy(k-d,i,5)-3*laypy(k-d,i,6);
                neighbour_xd(k-1,i,6) = laypxd(k-d,i+4,6)+laypxd(k-d,i+19,6)+laypxd(k-d,i,5)-3*laypxd(k-d,i,6);
                neighbour_yd(k-1,i,6) = laypyd(k-d,i+4,6)+laypyd(k-d,i+19,6)+laypyd(k-d,i,5)-3*laypyd(k-d,i,6);
             elseif i>=5 && i<=20
                neighbour_x(k-1,i,6) = laypx(k-d,i-4,6)+laypx(k-d,i+4,6)+laypx(k-d,i,5)-3*laypx(k-d,i,6);
                neighbour_y(k-1,i,6) = laypy(k-d,i-4,6)+laypy(k-d,i+4,6)+laypy(k-d,i,5)-3*laypy(k-d,i,6);
                neighbour_xd(k-1,i,6) = laypxd(k-d,i-4,6)+laypxd(k-d,i+4,6)+laypxd(k-d,i,5)-3*laypxd(k-d,i,6);
                neighbour_yd(k-1,i,6) = laypyd(k-d,i-4,6)+laypyd(k-d,i+4,6)+laypyd(k-d,i,5)-3*laypyd(k-d,i,6);
             elseif i>=21 && i<=23
                neighbour_x(k-1,i,6) = laypx(k-d,i-4,6)+laypx(k-d,i-19,6)+laypx(k-d,i-4,5)-3*laypx(k-d,i,6);
                neighbour_y(k-1,i,6) = laypy(k-d,i-4,6)+laypy(k-d,i-19,6)+laypy(k-d,i-4,5)-3*laypy(k-d,i,6);
                neighbour_xd(k-1,i,6) = laypxd(k-d,i-4,6)+laypxd(k-d,i-19,6)+laypxd(k-d,i-4,5)-3*laypxd(k-d,i,6);
                neighbour_yd(k-1,i,6) = laypyd(k-d,i-4,6)+laypyd(k-d,i-19,6)+laypyd(k-d,i-4,5)-3*laypyd(k-d,i,6);
             end
                neighbour_x(k-1,1,6) = laypx(k-d,5,6)+laypx(k-d,24,6)+laypx(k-d,1,5)-3*laypx(k-d,1,6);
                neighbour_y(k-1,1,6) = laypy(k-d,5,6)+laypy(k-d,24,6)+laypy(k-d,1,5)-3*laypy(k-d,1,6);
                neighbour_xd(k-1,1,6) = laypxd(k-d,5,6)+laypxd(k-d,24,6)+laypxd(k-d,1,5)-3*laypxd(k-d,1,6);
                neighbour_yd(k-1,1,6) = laypyd(k-d,5,6)+laypyd(k-d,24,6)+laypyd(k-d,1,5)-3*laypyd(k-d,1,6);
                neighbour_x(k-1,24,6) = laypx(k-d,1,6)+laypx(k-d,20,6)+laypx(k-d,20,5)-3*laypx(k-d,24,6);
                neighbour_y(k-1,24,6) = laypy(k-d,1,6)+laypy(k-d,20,6)+laypy(k-d,20,5)-3*laypy(k-d,24,6);
                neighbour_xd(k-1,24,6) = laypxd(k-d,1,6)+laypxd(k-d,20,6)+laypxd(k-d,20,5)-3*laypxd(k-d,24,6);
                neighbour_yd(k-1,24,6) = laypyd(k-d,1,6)+laypyd(k-d,20,6)+laypyd(k-d,20,5)-3*laypyd(k-d,24,6);
         elseif j==7
             if i>=2 && i<=4
                neighbour_x(k-1,i,7) = laypx(k-d,i+4,7)+laypx(k-d,i+23,7)+laypx(k-d,i,6)-3*laypx(k-d,i,7);
                neighbour_y(k-1,i,7) = laypy(k-d,i+4,7)+laypy(k-d,i+23,7)+laypy(k-d,i,6)-3*laypy(k-d,i,7);
                neighbour_xd(k-1,i,7) = laypxd(k-d,i+4,7)+laypxd(k-d,i+23,7)+laypxd(k-d,i,6)-3*laypxd(k-d,i,7);
                neighbour_yd(k-1,i,7) = laypyd(k-d,i+4,7)+laypyd(k-d,i+23,7)+laypyd(k-d,i,6)-3*laypyd(k-d,i,7);
             elseif i>=5 && i<=24
                neighbour_x(k-1,i,7) = laypx(k-d,i-4,7)+laypx(k-d,i+4,7)+laypx(k-d,i,6)-3*laypx(k-d,i,7);
                neighbour_y(k-1,i,7) = laypy(k-d,i-4,7)+laypy(k-d,i+4,7)+laypy(k-d,i,6)-3*laypy(k-d,i,7);
                neighbour_xd(k-1,i,7) = laypxd(k-d,i-4,7)+laypxd(k-d,i+4,7)+laypxd(k-d,i,6)-3*laypxd(k-d,i,7);
                neighbour_yd(k-1,i,7) = laypyd(k-d,i-4,7)+laypyd(k-d,i+4,7)+laypyd(k-d,i,6)-3*laypyd(k-d,i,7);
             elseif i>=25 && i<=27
                neighbour_x(k-1,i,7) = laypx(k-d,i-4,7)+laypx(k-d,i-23,7)+laypx(k-d,i-4,6)-3*laypx(k-d,i,7);
                neighbour_y(k-1,i,7) = laypy(k-d,i-4,7)+laypy(k-d,i-23,7)+laypy(k-d,i-4,6)-3*laypy(k-d,i,7);
                neighbour_xd(k-1,i,7) = laypxd(k-d,i-4,7)+laypxd(k-d,i-23,7)+laypxd(k-d,i-4,6)-3*laypxd(k-d,i,7);
                neighbour_yd(k-1,i,7) = laypyd(k-d,i-4,7)+laypyd(k-d,i-23,7)+laypyd(k-d,i-4,6)-3*laypyd(k-d,i,7);
             end
                neighbour_x(k-1,1,7) = laypx(k-d,5,7)+laypx(k-d,28,7)+laypx(k-d,1,6)-3*laypx(k-d,1,7);
                neighbour_y(k-1,1,7) = laypy(k-d,5,7)+laypy(k-d,28,7)+laypy(k-d,1,6)-3*laypy(k-d,1,7);
                neighbour_xd(k-1,1,7) = laypxd(k-d,5,7)+laypxd(k-d,28,7)+laypxd(k-d,1,6)-3*laypxd(k-d,1,7);
                neighbour_yd(k-1,1,7) = laypyd(k-d,5,7)+laypyd(k-d,28,7)+laypyd(k-d,1,6)-3*laypyd(k-d,1,7);
                neighbour_x(k-1,28,7) = laypx(k-d,1,7)+laypx(k-d,24,7)+laypx(k-d,24,6)-3*laypx(k-d,28,7);
                neighbour_y(k-1,28,7) = laypy(k-d,1,7)+laypy(k-d,24,7)+laypy(k-d,24,6)-3*laypy(k-d,28,7);
                neighbour_xd(k-1,28,7) = laypxd(k-d,1,7)+laypxd(k-d,24,7)+laypxd(k-d,24,6)-3*laypxd(k-d,28,7);
                neighbour_yd(k-1,28,7) = laypyd(k-d,1,7)+laypyd(k-d,24,7)+laypyd(k-d,24,6)-3*laypyd(k-d,28,7);
         elseif j==8
             if i>=2 && i<=4
                neighbour_x(k-1,i,8) = laypx(k-d,i+4,8)+laypx(k-d,i+27,8)+laypx(k-d,i,7)-3*laypx(k-d,i,8);
                neighbour_y(k-1,i,8) = laypy(k-d,i+4,8)+laypy(k-d,i+27,8)+laypy(k-d,i,7)-3*laypy(k-d,i,8);
                neighbour_xd(k-1,i,8) = laypxd(k-d,i+4,8)+laypxd(k-d,i+27,8)+laypxd(k-d,i,7)-3*laypxd(k-d,i,8);
                neighbour_yd(k-1,i,8) = laypyd(k-d,i+4,8)+laypyd(k-d,i+27,8)+laypyd(k-d,i,7)-3*laypyd(k-d,i,8);
             elseif i>=5 && i<=28
                neighbour_x(k-1,i,8) = laypx(k-d,i-4,8)+laypx(k-d,i+4,8)+laypx(k-d,i,7)-3*laypx(k-d,i,8);
                neighbour_y(k-1,i,8) = laypy(k-d,i-4,8)+laypy(k-d,i+4,8)+laypy(k-d,i,7)-3*laypy(k-d,i,8);
                neighbour_xd(k-1,i,8) = laypxd(k-d,i-4,8)+laypxd(k-d,i+4,8)+laypxd(k-d,i,7)-3*laypxd(k-d,i,8);
                neighbour_yd(k-1,i,8) = laypyd(k-d,i-4,8)+laypyd(k-d,i+4,8)+laypyd(k-d,i,7)-3*laypyd(k-d,i,8);
             elseif i>=29 && i<=31
                neighbour_x(k-1,i,8) = laypx(k-d,i-4,8)+laypx(k-d,i-27,8)+laypx(k-d,i-4,7)-3*laypx(k-d,i,8);
                neighbour_y(k-1,i,8) = laypy(k-d,i-4,8)+laypy(k-d,i-27,8)+laypy(k-d,i-4,7)-3*laypy(k-d,i,8);
                neighbour_xd(k-1,i,8) = laypxd(k-d,i-4,8)+laypxd(k-d,i-27,8)+laypxd(k-d,i-4,7)-3*laypxd(k-d,i,8);
                neighbour_yd(k-1,i,8) = laypyd(k-d,i-4,8)+laypyd(k-d,i-27,8)+laypyd(k-d,i-4,7)-3*laypyd(k-d,i,8);
             end
                neighbour_x(k-1,1,8) = laypx(k-d,5,8)+laypx(k-d,32,8)+laypx(k-d,1,7)-3*laypx(k-d,1,8);
                neighbour_y(k-1,1,8) = laypy(k-d,5,8)+laypy(k-d,32,8)+laypy(k-d,1,7)-3*laypy(k-d,1,8);
                neighbour_xd(k-1,1,8) = laypxd(k-d,5,8)+laypxd(k-d,32,8)+laypxd(k-d,1,7)-3*laypxd(k-d,1,8);
                neighbour_yd(k-1,1,8) = laypyd(k-d,5,8)+laypyd(k-d,32,8)+laypyd(k-d,1,7)-3*laypyd(k-d,1,8);
                neighbour_x(k-1,32,8) = laypx(k-d,1,8)+laypx(k-d,28,8)+laypx(k-d,28,7)-3*laypx(k-d,32,8);
                neighbour_y(k-1,32,8) = laypy(k-d,1,8)+laypy(k-d,28,8)+laypy(k-d,28,7)-3*laypy(k-d,32,8);
                neighbour_xd(k-1,32,8) = laypxd(k-d,1,8)+laypxd(k-d,28,8)+laypxd(k-d,28,7)-3*laypxd(k-d,32,8);
                neighbour_yd(k-1,32,8) = laypyd(k-d,1,8)+laypyd(k-d,28,8)+laypyd(k-d,28,7)-3*laypyd(k-d,32,8);        
         elseif j==9
             if i>=2 && i<=4
                neighbour_x(k-1,i,9) = laypx(k-d,i+4,9)+laypx(k-d,i+31,9)+laypx(k-d,i,8)-3*laypx(k-d,i,9);
                neighbour_y(k-1,i,9) = laypy(k-d,i+4,9)+laypy(k-d,i+31,9)+laypy(k-d,i,8)-3*laypy(k-d,i,9);
                neighbour_xd(k-1,i,9) = laypxd(k-d,i+4,9)+laypxd(k-d,i+31,9)+laypxd(k-d,i,8)-3*laypxd(k-d,i,9);
                neighbour_yd(k-1,i,9) = laypyd(k-d,i+4,9)+laypyd(k-d,i+31,9)+laypyd(k-d,i,8)-3*laypyd(k-d,i,9);
             elseif i>=5 && i<=32
                neighbour_x(k-1,i,9) = laypx(k-d,i-4,9)+laypx(k-d,i+4,9)+laypx(k-d,i,8)-3*laypx(k-d,i,9);
                neighbour_y(k-1,i,9) = laypy(k-d,i-4,9)+laypy(k-d,i+4,9)+laypy(k-d,i,8)-3*laypy(k-d,i,9);
                neighbour_xd(k-1,i,9) = laypxd(k-d,i-4,9)+laypxd(k-d,i+4,9)+laypxd(k-d,i,8)-3*laypxd(k-d,i,9);
                neighbour_yd(k-1,i,9) = laypyd(k-d,i-4,9)+laypyd(k-d,i+4,9)+laypyd(k-d,i,8)-3*laypyd(k-d,i,9);
             elseif i>=33 && i<=35
                neighbour_x(k-1,i,9) = laypx(k-d,i-4,9)+laypx(k-d,i-31,9)+laypx(k-d,i-4,8)-3*laypx(k-d,i,9);
                neighbour_y(k-1,i,9) = laypy(k-d,i-4,9)+laypy(k-d,i-31,9)+laypy(k-d,i-4,8)-3*laypy(k-d,i,9);
                neighbour_xd(k-1,i,9) = laypxd(k-d,i-4,9)+laypxd(k-d,i-31,9)+laypxd(k-d,i-4,8)-3*laypxd(k-d,i,9);
                neighbour_yd(k-1,i,9) = laypyd(k-d,i-4,9)+laypyd(k-d,i-31,9)+laypyd(k-d,i-4,8)-3*laypyd(k-d,i,9);
             end
                neighbour_x(k-1,1,9) = laypx(k-d,5,9)+laypx(k-d,36,9)+laypx(k-d,1,8)-3*laypx(k-d,1,9);
                neighbour_y(k-1,1,9) = laypy(k-d,5,9)+laypy(k-d,36,9)+laypy(k-d,1,8)-3*laypy(k-d,1,9);
                neighbour_xd(k-1,1,9) = laypxd(k-d,5,9)+laypxd(k-d,36,9)+laypxd(k-d,1,8)-3*laypxd(k-d,1,9);
                neighbour_yd(k-1,1,9) = laypyd(k-d,5,9)+laypyd(k-d,36,9)+laypyd(k-d,1,8)-3*laypyd(k-d,1,9);
                neighbour_x(k-1,36,9) = laypx(k-d,1,9)+laypx(k-d,32,9)+laypx(k-d,32,8)-3*laypx(k-d,36,9);
                neighbour_y(k-1,36,9) = laypy(k-d,1,9)+laypy(k-d,32,9)+laypy(k-d,32,8)-3*laypy(k-d,36,9);
                neighbour_xd(k-1,36,9) = laypxd(k-d,1,9)+laypxd(k-d,32,9)+laypxd(k-d,32,8)-3*laypxd(k-d,36,9);
                neighbour_yd(k-1,36,9) = laypyd(k-d,1,9)+laypyd(k-d,32,9)+laypyd(k-d,32,8)-3*laypyd(k-d,36,9);        
         elseif j==10
             if i>=2 && i<=4
                neighbour_x(k-1,i,10) = laypx(k-d,i+4,10)+laypx(k-d,i+35,10)+laypx(k-d,i,9)-3*laypx(k-d,i,10);
                neighbour_y(k-1,i,10) = laypy(k-d,i+4,10)+laypy(k-d,i+35,10)+laypy(k-d,i,9)-3*laypy(k-d,i,10);
                neighbour_xd(k-1,i,10) = laypxd(k-d,i+4,10)+laypxd(k-d,i+35,10)+laypxd(k-d,i,9)-3*laypxd(k-d,i,10);
                neighbour_yd(k-1,i,10) = laypyd(k-d,i+4,10)+laypyd(k-d,i+35,10)+laypyd(k-d,i,9)-3*laypyd(k-d,i,10);
             elseif i>=5 && i<=36
                neighbour_x(k-1,i,10) = laypx(k-d,i-4,10)+laypx(k-d,i+4,10)+laypx(k-d,i,9)-3*laypx(k-d,i,10);
                neighbour_y(k-1,i,10) = laypy(k-d,i-4,10)+laypy(k-d,i+4,10)+laypy(k-d,i,9)-3*laypy(k-d,i,10);
                neighbour_xd(k-1,i,10) = laypxd(k-d,i-4,10)+laypxd(k-d,i+4,10)+laypxd(k-d,i,9)-3*laypxd(k-d,i,10);
                neighbour_yd(k-1,i,10) = laypyd(k-d,i-4,10)+laypyd(k-d,i+4,10)+laypyd(k-d,i,9)-3*laypyd(k-d,i,10);
             elseif i>=37 && i<=39
                neighbour_x(k-1,i,10) = laypx(k-d,i-4,10)+laypx(k-d,i-35,10)+laypx(k-d,i-4,9)-3*laypx(k-d,i,10);
                neighbour_y(k-1,i,10) = laypy(k-d,i-4,10)+laypy(k-d,i-35,10)+laypy(k-d,i-4,9)-3*laypy(k-d,i,10);
                neighbour_xd(k-1,i,10) = laypxd(k-d,i-4,10)+laypxd(k-d,i-35,10)+laypxd(k-d,i-4,9)-3*laypxd(k-d,i,10);
                neighbour_yd(k-1,i,10) = laypyd(k-d,i-4,10)+laypyd(k-d,i-35,10)+laypyd(k-d,i-4,9)-3*laypyd(k-d,i,10);
             end
                neighbour_x(k-1,1,10) = laypx(k-d,5,10)+laypx(k-d,40,10)+laypx(k-d,1,9)-3*laypx(k-d,1,10);
                neighbour_y(k-1,1,10) = laypy(k-d,5,10)+laypy(k-d,40,10)+laypy(k-d,1,9)-3*laypy(k-d,1,10);
                neighbour_xd(k-1,1,10) = laypxd(k-d,5,10)+laypxd(k-d,40,10)+laypxd(k-d,1,9)-3*laypxd(k-d,1,10);
                neighbour_yd(k-1,1,10) = laypyd(k-d,5,10)+laypyd(k-d,40,10)+laypyd(k-d,1,9)-3*laypyd(k-d,1,10);
                neighbour_x(k-1,40,10) = laypx(k-d,1,10)+laypx(k-d,36,10)+laypx(k-d,36,9)-3*laypx(k-d,40,10);
                neighbour_y(k-1,40,10) = laypy(k-d,1,10)+laypy(k-d,36,10)+laypy(k-d,36,9)-3*laypy(k-d,40,10);
                neighbour_xd(k-1,40,10) = laypxd(k-d,1,10)+laypxd(k-d,36,10)+laypxd(k-d,36,9)-3*laypxd(k-d,40,10);
                neighbour_yd(k-1,40,10) = laypyd(k-d,1,10)+laypyd(k-d,36,10)+laypyd(k-d,36,9)-3*laypyd(k-d,40,10);
         elseif j==11
             if i>=2 && i<=4
                neighbour_x(k-1,i,11) = laypx(k-d,i+4,11)+laypx(k-d,i+39,11)+laypx(k-d,i,10)-3*laypx(k-d,i,11);
                neighbour_y(k-1,i,11) = laypy(k-d,i+4,11)+laypy(k-d,i+39,11)+laypy(k-d,i,10)-3*laypy(k-d,i,11);
                neighbour_xd(k-1,i,11) = laypxd(k-d,i+4,11)+laypxd(k-d,i+39,11)+laypxd(k-d,i,10)-3*laypxd(k-d,i,11);
                neighbour_yd(k-1,i,11) = laypyd(k-d,i+4,11)+laypyd(k-d,i+39,11)+laypyd(k-d,i,10)-3*laypyd(k-d,i,11);
             elseif i>=5 && i<=40
                neighbour_x(k-1,i,11) = laypx(k-d,i-4,11)+laypx(k-d,i+4,11)+laypx(k-d,i,10)-3*laypx(k-d,i,11);
                neighbour_y(k-1,i,11) = laypy(k-d,i-4,11)+laypy(k-d,i+4,11)+laypy(k-d,i,10)-3*laypy(k-d,i,11);
                neighbour_xd(k-1,i,11) = laypxd(k-d,i-4,11)+laypxd(k-d,i+4,11)+laypxd(k-d,i,10)-3*laypxd(k-d,i,11);
                neighbour_yd(k-1,i,11) = laypyd(k-d,i-4,11)+laypyd(k-d,i+4,11)+laypyd(k-d,i,10)-3*laypyd(k-d,i,11);
             elseif i>=41 && i<=43
                neighbour_x(k-1,i,11) = laypx(k-d,i-4,11)+laypx(k-d,i-39,11)+laypx(k-d,i-4,10)-3*laypx(k-d,i,11);
                neighbour_y(k-1,i,11) = laypy(k-d,i-4,11)+laypy(k-d,i-39,11)+laypy(k-d,i-4,10)-3*laypy(k-d,i,11);
                neighbour_xd(k-1,i,11) = laypxd(k-d,i-4,11)+laypxd(k-d,i-39,11)+laypxd(k-d,i-4,10)-3*laypxd(k-d,i,11);
                neighbour_yd(k-1,i,11) = laypyd(k-d,i-4,11)+laypyd(k-d,i-39,11)+laypyd(k-d,i-4,10)-3*laypyd(k-d,i,11);
             end
                neighbour_x(k-1,1,11) = laypx(k-d,5,11)+laypx(k-d,44,11)+laypx(k-d,1,10)-3*laypx(k-d,1,11);
                neighbour_y(k-1,1,11) = laypy(k-d,5,11)+laypy(k-d,44,11)+laypy(k-d,1,10)-3*laypy(k-d,1,11);
                neighbour_xd(k-1,1,11) = laypxd(k-d,5,11)+laypxd(k-d,44,11)+laypxd(k-d,1,10)-3*laypxd(k-d,1,11);
                neighbour_yd(k-1,1,11) = laypyd(k-d,5,11)+laypyd(k-d,44,11)+laypyd(k-d,1,10)-3*laypyd(k-d,1,11);
                neighbour_x(k-1,44,11) = laypx(k-d,1,11)+laypx(k-d,40,11)+laypx(k-d,40,10)-3*laypx(k-d,44,11);
                neighbour_y(k-1,44,11) = laypy(k-d,1,11)+laypy(k-d,40,11)+laypy(k-d,40,10)-3*laypy(k-d,44,11);
                neighbour_xd(k-1,44,11) = laypxd(k-d,1,11)+laypxd(k-d,40,11)+laypxd(k-d,40,10)-3*laypxd(k-d,44,11);
                neighbour_yd(k-1,44,11) = laypyd(k-d,1,11)+laypyd(k-d,40,11)+laypyd(k-d,40,10)-3*laypyd(k-d,44,11); 
         elseif j==12
             if i>=2 && i<=4
                neighbour_x(k-1,i,12) = laypx(k-d,i+4,12)+laypx(k-d,i+43,12)+laypx(k-d,i,11)-3*laypx(k-d,i,12);
                neighbour_y(k-1,i,12) = laypy(k-d,i+4,12)+laypy(k-d,i+43,12)+laypy(k-d,i,11)-3*laypy(k-d,i,12);
                neighbour_xd(k-1,i,12) = laypxd(k-d,i+4,12)+laypxd(k-d,i+43,12)+laypxd(k-d,i,11)-3*laypxd(k-d,i,12);
                neighbour_yd(k-1,i,12) = laypyd(k-d,i+4,12)+laypyd(k-d,i+43,12)+laypyd(k-d,i,11)-3*laypyd(k-d,i,12);
             elseif i>=5 && i<=44
                neighbour_x(k-1,i,12) = laypx(k-d,i-4,12)+laypx(k-d,i+4,12)+laypx(k-d,i,11)-3*laypx(k-d,i,12);
                neighbour_y(k-1,i,12) = laypy(k-d,i-4,12)+laypy(k-d,i+4,12)+laypy(k-d,i,11)-3*laypy(k-d,i,12);
                neighbour_xd(k-1,i,12) = laypxd(k-d,i-4,12)+laypxd(k-d,i+4,12)+laypxd(k-d,i,11)-3*laypxd(k-d,i,12);
                neighbour_yd(k-1,i,12) = laypyd(k-d,i-4,12)+laypyd(k-d,i+4,12)+laypyd(k-d,i,11)-3*laypyd(k-d,i,12);
             elseif i>=45 && i<=47
                neighbour_x(k-1,i,12) = laypx(k-d,i-4,12)+laypx(k-d,i-43,12)+laypx(k-d,i-4,11)-3*laypx(k-d,i,12);
                neighbour_y(k-1,i,12) = laypy(k-d,i-4,12)+laypy(k-d,i-43,12)+laypy(k-d,i-4,11)-3*laypy(k-d,i,12);
                neighbour_xd(k-1,i,12) = laypxd(k-d,i-4,12)+laypxd(k-d,i-43,12)+laypxd(k-d,i-4,11)-3*laypxd(k-d,i,12);
                neighbour_yd(k-1,i,12) = laypyd(k-d,i-4,12)+laypyd(k-d,i-43,12)+laypyd(k-d,i-4,11)-3*laypyd(k-d,i,12);
             end
                neighbour_x(k-1,1,12) = laypx(k-d,5,12)+laypx(k-d,48,12)+laypx(k-d,1,11)-3*laypx(k-d,1,12);
                neighbour_y(k-1,1,12) = laypy(k-d,5,12)+laypy(k-d,48,12)+laypy(k-d,1,11)-3*laypy(k-d,1,12);
                neighbour_xd(k-1,1,12) = laypxd(k-d,5,12)+laypxd(k-d,48,12)+laypxd(k-d,1,11)-3*laypxd(k-d,1,12);
                neighbour_yd(k-1,1,12) = laypyd(k-d,5,12)+laypyd(k-d,48,12)+laypyd(k-d,1,11)-3*laypyd(k-d,1,12);
                neighbour_x(k-1,48,12) = laypx(k-d,1,12)+laypx(k-d,44,12)+laypx(k-d,44,11)-3*laypx(k-d,48,12);
                neighbour_y(k-1,48,12) = laypy(k-d,1,12)+laypy(k-d,44,12)+laypy(k-d,44,11)-3*laypy(k-d,48,12);
                neighbour_xd(k-1,48,12) = laypxd(k-d,1,12)+laypxd(k-d,44,12)+laypxd(k-d,44,11)-3*laypxd(k-d,48,12);
                neighbour_yd(k-1,48,12) = laypyd(k-d,1,12)+laypyd(k-d,44,12)+laypyd(k-d,44,11)-3*laypyd(k-d,48,12);
         elseif j==13
             if i>=2 && i<=4
                neighbour_x(k-1,i,13) = laypx(k-d,i+4,13)+laypx(k-d,i+47,13)+laypx(k-d,i,12)-3*laypx(k-d,i,13);
                neighbour_y(k-1,i,13) = laypy(k-d,i+4,13)+laypy(k-d,i+47,13)+laypy(k-d,i,12)-3*laypy(k-d,i,13);
                neighbour_xd(k-1,i,13) = laypxd(k-d,i+4,13)+laypxd(k-d,i+47,13)+laypxd(k-d,i,12)-3*laypxd(k-d,i,13);
                neighbour_yd(k-1,i,13) = laypyd(k-d,i+4,13)+laypyd(k-d,i+47,13)+laypyd(k-d,i,12)-3*laypyd(k-d,i,13);
             elseif i>=5 && i<=48
                neighbour_x(k-1,i,13) = laypx(k-d,i-4,13)+laypx(k-d,i+4,13)+laypx(k-d,i,12)-3*laypx(k-d,i,13);
                neighbour_y(k-1,i,13) = laypy(k-d,i-4,13)+laypy(k-d,i+4,13)+laypy(k-d,i,12)-3*laypy(k-d,i,13);
                neighbour_xd(k-1,i,13) = laypxd(k-d,i-4,13)+laypxd(k-d,i+4,13)+laypxd(k-d,i,12)-3*laypxd(k-d,i,13);
                neighbour_yd(k-1,i,13) = laypyd(k-d,i-4,13)+laypyd(k-d,i+4,13)+laypyd(k-d,i,12)-3*laypyd(k-d,i,13);
             elseif i>=49 && i<=51
                neighbour_x(k-1,i,13) = laypx(k-d,i-4,13)+laypx(k-d,i-47,13)+laypx(k-d,i-4,12)-3*laypx(k-d,i,13);
                neighbour_y(k-1,i,13) = laypy(k-d,i-4,13)+laypy(k-d,i-47,13)+laypy(k-d,i-4,12)-3*laypy(k-d,i,13);
                neighbour_xd(k-1,i,13) = laypxd(k-d,i-4,13)+laypxd(k-d,i-47,13)+laypxd(k-d,i-4,12)-3*laypxd(k-d,i,13);
                neighbour_yd(k-1,i,13) = laypyd(k-d,i-4,13)+laypyd(k-d,i-47,13)+laypyd(k-d,i-4,12)-3*laypyd(k-d,i,13);
             end
                neighbour_x(k-1,1,13) = laypx(k-d,5,13)+laypx(k-d,52,13)+laypx(k-d,1,12)-3*laypx(k-d,1,13);
                neighbour_y(k-1,1,13) = laypy(k-d,5,13)+laypy(k-d,52,13)+laypy(k-d,1,12)-3*laypy(k-d,1,13);
                neighbour_xd(k-1,1,13) = laypxd(k-d,5,13)+laypxd(k-d,52,13)+laypxd(k-d,1,12)-3*laypxd(k-d,1,13);
                neighbour_yd(k-1,1,13) = laypyd(k-d,5,13)+laypyd(k-d,52,13)+laypyd(k-d,1,12)-3*laypyd(k-d,1,13);
                neighbour_x(k-1,52,13) = laypx(k-d,1,13)+laypx(k-d,48,13)+laypx(k-d,48,12)-3*laypx(k-d,52,13);
                neighbour_y(k-1,52,13) = laypy(k-d,1,13)+laypy(k-d,48,13)+laypy(k-d,48,12)-3*laypy(k-d,52,13);
                neighbour_xd(k-1,52,13) = laypxd(k-d,1,13)+laypxd(k-d,48,13)+laypxd(k-d,48,12)-3*laypxd(k-d,52,13);
                neighbour_yd(k-1,52,13) = laypyd(k-d,1,13)+laypyd(k-d,48,13)+laypyd(k-d,48,12)-3*laypyd(k-d,52,13);
        elseif j==14
             if i>=2 && i<=4
                neighbour_x(k-1,i,14) = laypx(k-d,i+4,14)+laypx(k-d,i+51,14)+laypx(k-d,i,13)-3*laypx(k-d,i,14);
                neighbour_y(k-1,i,14) = laypy(k-d,i+4,14)+laypy(k-d,i+51,14)+laypy(k-d,i,13)-3*laypy(k-d,i,14);
                neighbour_xd(k-1,i,14) = laypxd(k-d,i+4,14)+laypxd(k-d,i+51,14)+laypxd(k-d,i,13)-3*laypxd(k-d,i,14);
                neighbour_yd(k-1,i,14) = laypyd(k-d,i+4,14)+laypyd(k-d,i+51,14)+laypyd(k-d,i,13)-3*laypyd(k-d,i,14);
             elseif i>=5 && i<=52
                neighbour_x(k-1,i,14) = laypx(k-d,i-4,14)+laypx(k-d,i+4,14)+laypx(k-d,i,13)-3*laypx(k-d,i,14);
                neighbour_y(k-1,i,14) = laypy(k-d,i-4,14)+laypy(k-d,i+4,14)+laypy(k-d,i,13)-3*laypy(k-d,i,14);
                neighbour_xd(k-1,i,14) = laypxd(k-d,i-4,14)+laypxd(k-d,i+4,14)+laypxd(k-d,i,13)-3*laypxd(k-d,i,14);
                neighbour_yd(k-1,i,14) = laypyd(k-d,i-4,14)+laypyd(k-d,i+4,14)+laypyd(k-d,i,13)-3*laypyd(k-d,i,14);
             elseif i>=53 && i<=55
                neighbour_x(k-1,i,14) = laypx(k-d,i-4,14)+laypx(k-d,i-51,14)+laypx(k-d,i-4,13)-3*laypx(k-d,i,14);
                neighbour_y(k-1,i,14) = laypy(k-d,i-4,14)+laypy(k-d,i-51,14)+laypy(k-d,i-4,13)-3*laypy(k-d,i,14);
                neighbour_xd(k-1,i,14) = laypxd(k-d,i-4,14)+laypxd(k-d,i-51,14)+laypxd(k-d,i-4,13)-3*laypxd(k-d,i,14);
                neighbour_yd(k-1,i,14) = laypyd(k-d,i-4,14)+laypyd(k-d,i-51,14)+laypyd(k-d,i-4,13)-3*laypyd(k-d,i,14);
             end
                neighbour_x(k-1,1,14) = laypx(k-d,5,14)+laypx(k-d,56,14)+laypx(k-d,1,13)-3*laypx(k-d,1,14);
                neighbour_y(k-1,1,14) = laypy(k-d,5,14)+laypy(k-d,56,14)+laypy(k-d,1,13)-3*laypy(k-d,1,14);
                neighbour_xd(k-1,1,14) = laypxd(k-d,5,14)+laypxd(k-d,56,14)+laypxd(k-d,1,13)-3*laypxd(k-d,1,14);
                neighbour_yd(k-1,1,14) = laypyd(k-d,5,14)+laypyd(k-d,56,14)+laypyd(k-d,1,13)-3*laypyd(k-d,1,14);
                neighbour_x(k-1,56,14) = laypx(k-d,1,14)+laypx(k-d,52,14)+laypx(k-d,52,13)-3*laypx(k-d,56,14);
                neighbour_y(k-1,56,14) = laypy(k-d,1,14)+laypy(k-d,52,14)+laypy(k-d,52,13)-3*laypy(k-d,56,14);
                neighbour_xd(k-1,56,14) = laypxd(k-d,1,14)+laypxd(k-d,52,14)+laypxd(k-d,52,13)-3*laypxd(k-d,56,14);
                neighbour_yd(k-1,56,14) = laypyd(k-d,1,14)+laypyd(k-d,52,14)+laypyd(k-d,52,13)-3*laypyd(k-d,56,14);      
         end
       %calculate the angle for disturbance state dependence
       theta(k-1,i,j) = theta(k-2,i,j)+((-1/2/l)*layvx(k-2,i,j)*sin(theta(k-2,i,j))+(1/2/l)*layvy(k-2,i,j)*cos(theta(k-2,i,j)))*tg;       
       %control input
       xacc = ax(k-1,1)+kp*(neighbour_x(k-1,i,j)-neighbour_xd(k-1,i,j))...
           +kp0*(laypxd(k-1,i,j)-laypx(k-1,i,j))+kv0*(vx(k-1,1)-layvx(k-1,i,j))+[1/m*cos(theta(k-1,i,j)) -l/I*sin(theta(k-1,i,j))]*[d11(k-1,i,j);d22(k-1,i,j)];
       yacc = ay(k-1,1)+kp*(neighbour_y(k-1,i,j)-neighbour_yd(k-1,i,j))...
           +kp0*(laypyd(k-1,i,j)-laypy(k-1,i,j))+kv0*(vy(k-1,1)-layvy(k-1,i,j))+[1/m*sin(theta(k-1,i,j)) l/I*cos(theta(k-1,i,j))]*[d11(k-1,i,j);d22(k-1,i,j)];
       layvx(k,i,j) = layvx(k-1,i,j)+tg*xacc;
       layvy(k,i,j) = layvy(k-1,i,j)+tg*yacc;
       laypx(k,i,j) = laypx(k-1,i,j)+(layvx(k-1,i,j)+layvx(k,i,j))/2*tg;
       laypy(k,i,j) = laypy(k-1,i,j)+(layvy(k-1,i,j)+layvy(k,i,j))/2*tg;
       positiondev(k,i,j) = norm([laypx(k,i,j)-laypxd(k,i,j),laypy(k,i,j)-laypyd(k,i,j)]);
      end
   end
end 
%% plot state deviation
figure
for j=1:layer
    for i = 1:4*j
        if j == 1
            if any(i==ID(:,j))
                print 0
            else
                plot(t(d+1:end),positiondev(d+1:end,i,j),'r','linewidth',1);hold on
            end
        elseif j ==2
            plot(t(d+1:end),positiondev(d+1:end,i,j),'Color',[1 0.07 0],'linewidth',1);
        elseif j ==3
            plot(t(d+1:end),positiondev(d+1:end,i,j),'Color',[1 0.13 0],'linewidth',1)
        elseif j==4
            plot(t(d+1:end),positiondev(d+1:end,i,j),'Color',[1 0.19 0],'linewidth',1)
        elseif j==5
            plot(t(d+1:end),positiondev(d+1:end,i,j),'Color',[1 0.25 0],'linewidth',1)
        elseif j==6
            plot(t(d+1:end),positiondev(d+1:end,i,j),'Color',[1 0.31 0],'linewidth',1)
        elseif j==7
            plot(t(d+1:end),positiondev(d+1:end,i,j),'Color',[1 0.37 0],'linewidth',1)
        elseif j==8
            plot(t(d+1:end),positiondev(d+1:end,i,j),'Color',[1 0.43 0],'linewidth',1)
        elseif j ==9
            plot(t(d+1:end),positiondev(d+1:end,i,j),'Color',[1 0.50 0],'linewidth',1)
        elseif j==10
            plot(t(d+1:end),positiondev(d+1:end,i,j),'Color',[1 0.56 0],'linewidth',1)
        elseif j==11
            plot(t(d+1:end),positiondev(d+1:end,i,j),'Color',[1 0.7 0],'linewidth',1)
        elseif j==12
            plot(t(d+1:end),positiondev(d+1:end,i,j),'Color',[1 0.8 0],'linewidth',1)
        elseif j==13
            plot(t(d+1:end),positiondev(d+1:end,i,j),'Color',[1 0.9 0],'linewidth',1)
        elseif j==14
            plot(t(d+1:end),positiondev(d+1:end,i,j),'Color',[1 1 0],'linewidth',1)
        end
    end
end
set(gca,'FontSize',22)
xlabel('$t[s]$','Interpreter','latex','FontSize',24)
ylabel({'$\vert \eta_i(t)-\eta_i^d(t) \vert_2$'},'Interpreter','latex','FontSize',24)
