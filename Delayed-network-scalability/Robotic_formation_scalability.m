clc;
close all;

%robot parameters
l = 0.12;
m = 10.1;
I = 0.13;

layer = 6;%number of layers of the formation pattern
node = zeros(layer,1);
N = 2*layer*(layer+1);%number of follower robots

time = 40;tg = 0.01;t = 0:tg:time;%simulation time

%define the disturbance
d1 = zeros(size(t,2),N+1);%disturbance on x axis
d2 = zeros(size(t,2),N+1);%disturbance on y axis
perturbedID = 1+randsample(N,N-5);%select the ID of perturbed robots. Robot 1 should not be selected because it is the virtual leader
for i = 1:N-5
   am1(i) = rand(1);
   am2(i) = rand(1);
   d1(:,perturbedID(i)) = 2*am1(i)*sin(t).*exp(-0.2*t);
   d2(:,perturbedID(i)) = 2*am2(i)*sin(t).*exp(-0.2*t);
end

%gains and weights
kp0 = 0.7;
kv0 = 1;
kp = 0.7;
kv = 0;
w = 0.1;

veq = 2;%constant linear speed throughout
omega = pi/20;%constant angular velocity during maneuvering
d = 11;%delay=(d-1)*tg

%creat position/angle vectors
px = zeros(size(t,2),N+1);
py = zeros(size(t,2),N+1);
pxd = zeros(size(t,2),N+1);
pyd = zeros(size(t,2),N+1);
theta = pi/2*ones(size(t,2),N+1);

%define virtual leader dynamics, leader ID = 1. The initial condtions are
%definded in the time interval [0,d*tg] and equal to desired state at 0s.
vx(1:d,1) = veq*sin(omega*zeros(d,1));
vy(1:d,1) = veq*cos(omega*zeros(d,1));
ax(1:d,1) = veq*omega*cos(omega*zeros(d,1));
ay(1:d,1) = -veq*omega*sin(omega*zeros(d,1));
vx(d+1:500+d,1) = veq*sin(omega*t(1:500));
vx(501+d:d+2500,1) = veq/sqrt(2);
vx(2501+d:d+4001,1) = veq*sin(pi/4+omega*t(1:1501));
vy(1+d:d+500,1) = veq*cos(omega*t(1:500));
vy(501+d:d+2500,1) = veq/sqrt(2);
vy(2501+d:4001+d,1) = veq*cos(pi/4+omega*t(1:1501));
ax(1+d:500+d,1) = veq*omega*cos(omega*t(1:500));
ax(501+d:d+2500,1) = zeros(2000,1);
ax(2501+d:4001+d,1) = veq*omega*cos(pi/4+omega*t(1:1501));
ay(1+d:500+d,1) = -veq*omega*sin(omega*t(1:500));
ay(501+d:d+2500,1) = zeros(2000,1);
ay(2501+d:4001+d,1) = -veq*omega*sin(pi/4+omega*t(1:1501));
%% the leader position and desired followers position
for i = d+1:size(t,2)
    px(i,1) = px(i-1,1)+(vx(i-1,1)+vx(i,1))/2*tg;
    py(i,1) = py(i-1,1)+(vy(i-1,1)+vy(i,1))/2*tg;
end
for i = 1:layer
   node(i) = 4*i;%number of nodes in layer i
   for j = 1:node(i)
       pxd(:,2*i*(i-1)+j+1) = px(:,1)+3*i*cos(2*pi/node(i)*j);
       pyd(:,2*i*(i-1)+j+1) = py(:,1)+3*i*sin(2*pi/node(i)*j);
   end
end
%set the initial condition(position/velocity) to be at the desired state.
for i = 2:N+1
    px(1:d,i) = pxd(1:d,i);
    py(1:d,i) = pyd(1:d,i);
    vx(1:d,i) = vx(1:d,1);
    vy(1:d,i) = vy(1:d,1);
end
pxd(:,1) = px(:,1);
pyd(:,1) = py(:,1);
%% define followers dynamics in normal disturbance
for i = d+1:size(t,2)
   for j = 2:N+1
       %x axis
       pxVh = px(i-1,j);vxVh = vx(i-1,j);%agent j without delay
       pxVhRef = px(i-1,1);vxVhRef = vx(i-1,1);%reference without delay
       pxVhlag = px(i-d,j);vxVhlag = vx(i-d,j);%agent j with delay
       pxdVh = pxd(i-1,j);pxdVhlag = pxd(i-d,j);%reference with delay
       if j == 2%neighbour 1
           pxVhPr = px(i-d,j+3);vxVhPr = vx(i-d,j+3);pxdVhPr = pxd(i-d,j+3);
       else
           pxVhPr = px(i-d,j-1);vxVhPr = vx(i-d,j-1);pxdVhPr = pxd(i-d,j-1);
       end
       if j ~= N+1%neighbour 2
           pxVhFl = px(i-d,j+1);vxVhFl = vx(i-d,j+1);pxdVhFl = pxd(i-d,j+1);
       else
           pxVhFl = px(i-d,j-2);vxVhFl = vx(i-d,j-2);pxdVhFl = pxd(i-d,j-2);
       end
       %y axis
       pyVh = py(i-1,j);vyVh = vy(i-1,j);
       pyVhRef = py(i-1,1);vyVhRef = vy(i-1,1);
       pyVhlag = py(i-d,j);vyVhlag = vy(i-d,j);
       pydVh = pyd(i-1,j);pydVhlag = pyd(i-d,j);
       if j == 2
           pyVhPr = py(i-d,j+3);vyVhPr = vy(i-d,j+3);pydVhPr = pyd(i-d,j+3);
       else
           pyVhPr = py(i-d,j-1);vyVhPr = vy(i-d,j-1);pydVhPr = pyd(i-d,j-1);
       end
       if j ~= N+1
           pyVhFl = py(i-d,j+1);vyVhFl = vy(i-d,j+1);pydVhFl = pyd(i-d,j+1);
       else
           pyVhFl = py(i-d,j-2);vyVhFl = vy(i-d,j-2);pydVhFl = pyd(i-d,j-2);
       end
       %calculate the angle for disturbance state dependence
       theta(i-1,j) = theta(i-2,j)+((-1/2/l)*vx(i-2,j)*sin(theta(i-2,j))+(1/2/l)*vy(i-2,j)*cos(theta(i-2,j)))*tg;       
       %control input
       xacc = ax(i-1,1)+w*(kp*(pxVhPr-pxVhlag-pxdVhPr+pxdVhlag)+kv*(vxVhPr-vxVhlag)+kp*(pxVhFl-pxVhlag-pxdVhFl+pxdVhlag)+kv*(vxVhFl-vxVhlag))...
           +kp0*(pxVhRef-pxVh-pxVhRef+pxdVh)+kv0*(vxVhRef-vxVh)+[1/m*cos(theta(i-1,j)) -l/I*sin(theta(i-1,j))]*[d1(i-1,j);d2(i-1,j)];
       yacc = ay(i-1,1)+w*(kp*(pyVhPr-pyVhlag-pydVhPr+pydVhlag)+kv*(vyVhPr-vyVhlag)+kp*(pyVhFl-pyVhlag-pydVhFl+pydVhlag)+kv*(vyVhFl-vyVhlag))...
           +kp0*(pyVhRef-pyVh-pyVhRef+pydVh)+kv0*(vyVhRef-vyVh)+[1/m*sin(theta(i-1,j)) l/I*cos(theta(i-1,j))]*[d1(i-1,j);d2(i-1,j)];
       vx(i,j) = vxVh+tg*xacc;
       vy(i,j) = vyVh+tg*yacc;
       px(i,j) = pxVh+(vxVh+vx(i,j))/2*tg;
       py(i,j) = pyVh+(vyVh+vy(i,j))/2*tg;
   end 
end
%% plot state deviation
figure
for i = 2:N+1
    if any(i==perturbedID)
       ln1 = plot(t(d+1:end),px(d+1:end,i)-pxd(d+1:end,i),'k','linewidth',1);hold on
    else
       ln2 = plot(t(d+1:end),px(d+1:end,i)-pxd(d+1:end,i),'r','linewidth',1)
    end
end
legend([ln1,ln2],'Perturbed agents','Unperturbed agents')
set(gca,'FontSize',14)
xlabel('t[s]','FontSize',16)
ylabel('x-deviations[m]','FontSize',16)

figure
for i = 2:N+1
    if any(i==perturbedID)
        ln3 = plot(t(d+1:end),py(d+1:end,i)-pyd(d+1:end,i),'k','linewidth',1);hold on
    else
        ln4 = plot(t(d+1:end),py(d+1:end,i)-pyd(d+1:end,i),'r','linewidth',1)
    end
end
legend([ln3,ln4],'Perturbed agents','Unperturbed agents')
set(gca,'FontSize',14)
xlabel('t[s]','FontSize',16)
ylabel('y-deviations[m]','FontSize',16)
%% plot formation configuration
figure
leader = plot(pxd(1,1),pyd(1,1),'p','Color','b','MarkerSize',10);hold on
plot(pxd(size(t,2),1),pyd(size(t,2),1),'p','Color','b','MarkerSize',10)
for j = 1:node(1)
    layer1 = plot(pxd(1,j+1),pyd(1,j+1),'o','Color','b','MarkerSize',10);hold on
    plot(pxd(size(t,2),j+1),pyd(size(t,2),j+1),'o','Color','b','MarkerSize',10);hold on
end
for j = 1:node(2)
    layer2 = plot(pxd(1,4+j+1),pyd(1,4+j+1),'x','Color','r','MarkerSize',10);hold on
    plot(pxd(size(t,2),4+j+1),pyd(size(t,2),4+j+1),'x','Color','r','MarkerSize',10);hold on
end
for j = 1:node(3)
    layer3 = plot(pxd(1,12+j+1),pyd(1,12+j+1),'*','Color','k','MarkerSize',10);hold on
    plot(pxd(size(t,2),12+j+1),pyd(size(t,2),12+j+1),'*','Color','k','MarkerSize',10);hold on
end
plot(pxd(:,1),pyd(:,1))
legend([leader,layer1,layer2,layer3],{'Virtual reference','Layer 1','Layer 2','Layer 3'},'Location','Best')
xlim([-15,65])
ylim([-15,45])
set(gca,'FontSize',14)
xlabel('x','FontSize',16)
ylabel('y','FontSize',16)
axis equal
