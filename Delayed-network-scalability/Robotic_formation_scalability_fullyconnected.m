clc
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

ID = randsample(4,1);%perturbed agent ID
d1(:,ID,1) = 2*sin(t).*exp(-0.2*t);
d2(:,ID,1) = 2*sin(t).*exp(-0.2*t);

%gains and weights
kp0 = 0.7;
kv0 = 1;
kp = 0.035;

veq = 2;%constant linear speed throughout
omega = pi/20;%constant angular velocity during maneuvering
d = 11;%delay=(d-1)*tg

%creat position/angle vectors
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
%% define followers dynamics in normal disturbance
neighbour_x = zeros(size(t,2),layer*4,layer);
neighbour_y = zeros(size(t,2),layer*4,layer);
neighbour_xd = zeros(size(t,2),layer*4,layer);
neighbour_yd = zeros(size(t,2),layer*4,layer);
theta = zeros(size(t,2),layer*4,layer);
for k = d+1:size(t,2)
   for j = 1:layer
      for i = 1:4*j
          neighbour_x(k-1,i,j) = sum(laypx(k-d,1:4*layer,1:layer),'all')-(2+2*layer)*layer*laypx(k-d,i,j);
          neighbour_y(k-1,i,j) = sum(laypy(k-d,1:4*layer,1:layer),'all')-(2+2*layer)*layer*laypy(k-d,i,j);
          neighbour_xd(k-1,i,j) = sum(laypxd(k-d,1:4*layer,1:layer),'all')-(2+2*layer)*layer*laypxd(k-d,i,j);
          neighbour_yd(k-1,i,j) = sum(laypyd(k-d,1:4*layer,1:layer),'all')-(2+2*layer)*layer*laypyd(k-d,i,j);
       %calculate the angle for disturbance state dependence
       theta(k-1,i,j) = theta(k-2,i,j)+((-1/2/l)*layvx(k-2,i,j)*sin(theta(k-2,i,j))+(1/2/l)*layvy(k-2,i,j)*cos(theta(k-2,i,j)))*tg;       
       %control input
       xacc = ax(k-1,1)+kp*(neighbour_x(k-1,i,j)-neighbour_xd(k-1,i,j))...
           +kp0*(laypxd(k-1,i,j)-laypx(k-1,i,j))+kv0*(vx(k-1,1)-layvx(k-1,i,j))+[1/m*cos(theta(k-1,i,j)) -l/I*sin(theta(k-1,i,j))]*[d1(k-1,i,j);d2(k-1,i,j)];
       yacc = ay(k-1,1)+kp*(neighbour_y(k-1,i,j)-neighbour_yd(k-1,i,j))...
           +kp0*(laypyd(k-1,i,j)-laypy(k-1,i,j))+kv0*(vy(k-1,1)-layvy(k-1,i,j))+[1/m*sin(theta(k-1,i,j)) l/I*cos(theta(k-1,i,j))]*[d1(k-1,i,j);d2(k-1,i,j)];
       layvx(k,i,j) = layvx(k-1,i,j)+tg*xacc;
       layvy(k,i,j) = layvy(k-1,i,j)+tg*yacc;
       laypx(k,i,j) = laypx(k-1,i,j)+(layvx(k-1,i,j)+layvx(k,i,j))/2*tg;
       laypy(k,i,j) = laypy(k-1,i,j)+(layvy(k-1,i,j)+layvy(k,i,j))/2*tg;
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
                plot(t(d+1:end),laypx(d+1:end,i,j)-laypxd(d+1:end,i,j),'r','linewidth',1);hold on
            end
        elseif j==2
            plot(t(d+1:end),laypx(d+1:end,i,2)-laypxd(d+1:end,i,2),'Color',[1 0.07 0],'linewidth',1)
        elseif j ==3
            plot(t(d+1:end),laypx(d+1:end,i,j)-laypxd(d+1:end,i,j),'Color',[1 0.13 0],'linewidth',1)
        elseif j==4
            plot(t(d+1:end),laypx(d+1:end,i,j)-laypxd(d+1:end,i,j),'Color',[1 0.19 0],'linewidth',1)
        elseif j==5
            plot(t(d+1:end),laypx(d+1:end,i,j)-laypxd(d+1:end,i,j),'Color',[1 0.25 0],'linewidth',1)
        elseif j==6
            plot(t(d+1:end),laypx(d+1:end,i,j)-laypxd(d+1:end,i,j),'Color',[1 0.31 0],'linewidth',1)
        elseif j==7
            plot(t(d+1:end),laypx(d+1:end,i,j)-laypxd(d+1:end,i,j),'Color',[1 0.37 0],'linewidth',1)
        elseif j==8
            plot(t(d+1:end),laypx(d+1:end,i,j)-laypxd(d+1:end,i,j),'Color',[1 0.43 0],'linewidth',1)
        elseif j ==9
            plot(t(d+1:end),laypx(d+1:end,i,j)-laypxd(d+1:end,i,j),'Color',[1 0.50 0],'linewidth',1)
        elseif j==10
            plot(t(d+1:end),laypx(d+1:end,i,j)-laypxd(d+1:end,i,j),'Color',[1 0.56 0],'linewidth',1)
        elseif j==11
            plot(t(d+1:end),laypx(d+1:end,i,j)-laypxd(d+1:end,i,j),'Color',[1 0.7 0],'linewidth',1)
        elseif j==12
            plot(t(d+1:end),laypx(d+1:end,i,j)-laypxd(d+1:end,i,j),'Color',[1 0.8 0],'linewidth',1)
        elseif j==13
            plot(t(d+1:end),laypx(d+1:end,i,j)-laypxd(d+1:end,i,j),'Color',[1 0.9 0],'linewidth',1)
        elseif j==14
            plot(t(d+1:end),laypx(d+1:end,i,j)-laypxd(d+1:end,i,j),'Color',[0 0 0],'linewidth',1)
        end
    end
end
set(gca,'FontSize',24)
xlabel('t[s]','FontSize',30)
ylabel('x-deviations[m]','FontSize',30)

figure
for j=1:layer
    for i = 1:4*j
        if j == 1
            if any(i==ID(:,j))
                print 0
            else
                plot(t(d+1:end),laypy(d+1:end,i,j)-laypyd(d+1:end,i,j),'r','linewidth',1);hold on
            end
        elseif j==2
            plot(t(d+1:end),laypy(d+1:end,i,j)-laypyd(d+1:end,i,j),'Color',[1 0.07 0],'linewidth',1)
        elseif j ==3
            plot(t(d+1:end),laypy(d+1:end,i,j)-laypyd(d+1:end,i,j),'Color',[1 0.13 0],'linewidth',1)
        elseif j==4
            plot(t(d+1:end),laypy(d+1:end,i,j)-laypyd(d+1:end,i,j),'Color',[1 0.19 0],'linewidth',1)
        elseif j==5
            plot(t(d+1:end),laypy(d+1:end,i,j)-laypyd(d+1:end,i,j),'Color',[1 0.25 0],'linewidth',1)
        elseif j==6
            plot(t(d+1:end),laypy(d+1:end,i,j)-laypyd(d+1:end,i,j),'Color',[1 0.31 0],'linewidth',1)
        elseif j==7
            plot(t(d+1:end),laypy(d+1:end,i,j)-laypyd(d+1:end,i,j),'Color',[1 0.37 0],'linewidth',1)
        elseif j==8
            plot(t(d+1:end),laypy(d+1:end,i,j)-laypyd(d+1:end,i,j),'Color',[1 0.43 0],'linewidth',1)
        elseif j ==9
            plot(t(d+1:end),laypy(d+1:end,i,j)-laypyd(d+1:end,i,j),'Color',[1 0.50 0],'linewidth',1)
        elseif j==10
            plot(t(d+1:end),laypy(d+1:end,i,j)-laypyd(d+1:end,i,j),'Color',[1 0.56 0],'linewidth',1)
        elseif j==11
            plot(t(d+1:end),laypy(d+1:end,i,j)-laypyd(d+1:end,i,j),'Color',[1 0.7 0],'linewidth',1)
        elseif j==12
            plot(t(d+1:end),laypy(d+1:end,i,j)-laypyd(d+1:end,i,j),'Color',[1 0.8 0],'linewidth',1)
        elseif j==13
            plot(t(d+1:end),laypy(d+1:end,i,j)-laypyd(d+1:end,i,j),'Color',[1 0.9 0],'linewidth',1)
        elseif j==14
            plot(t(d+1:end),laypy(d+1:end,i,j)-laypyd(d+1:end,i,j),'Color',[0 0 0],'linewidth',1)
        end
    end
end
set(gca,'FontSize',24)
xlabel('t[s]','FontSize',30)
ylabel('y-deviations[m]','FontSize',30)