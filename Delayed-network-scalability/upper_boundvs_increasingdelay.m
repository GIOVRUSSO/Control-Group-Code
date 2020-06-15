clc;
close all;

layer = 6;%number of layers
node = zeros(layer,1);
N = 2*layer*(layer+1);%number of follower robots

%robot parameters
l = 0.12;m = 10.1;I = 0.13;
t = 0:0.01:40;
%disturbace, the same group of robots are perturbed by same disturbance
%throughout the simulation
perturbedID = 1+randsample(N,N-5);%%select the ID of perturbed robots. Robot 1 should not be selected because it is the virtual leader
for i = 1:N-5
   am1(i) = rand(1);
   am2(i) = rand(1);
   d11(:,perturbedID(i)) = 2*am1(i)*sin(t).*exp(-0.2*t);
   d22(:,perturbedID(i)) = 2*am2(i)*sin(t).*exp(-0.2*t);
end
%gains and weigts
kp0 = 0.7;kv0 = 1;kp = 0.7;kv = 0;w = 0.1;
%% different delay
for d = 1:2000 %delay=(d-1)*tg
    tg = 0.01;time = 40+d*tg;t = 0:tg:time;
    %disturbance starts at (d+1)*tg[s], when the robots start to move.
    d1 = zeros(size(t,2),N+1);%disturbance on x axis
    d1(d+1:end,:) = d11;
    d2 = zeros(size(t,2),N+1);%disturbance on y axis
    d2(d+1:end,:) = d22;
    %creat vector
    px = zeros(d,N+1);py = zeros(d,N+1);vx = zeros(d,N+1);vy = zeros(d,N+1);ax = zeros(d,N+1);ay = zeros(d,N+1);
    %define leader dynamics, leader ID = 1
    veq = 2;%constant linear speed
    omega = pi/20;%constant angular speed during maneuvering
    %the initial consition are set to be desired state when robots start to move
    vx(1:d,1) = veq*sin(omega*zeros(d,1));
    vy(1:d,1) = veq*cos(omega*zeros(d,1));
    ax(1:d,1) = veq*omega*cos(omega*zeros(d,1));
    ay(1:d,1) = -veq*omega*sin(omega*zeros(d,1));
    vx(d+1:500+d,1) = veq*sin(omega*t(1:500));
    vx(501+d:2500+d,1) = veq/sqrt(2);
    vx(2501+d:4001+d,1) = veq*sin(pi/4+omega*t(1:1501));
    vy(d+1:500+d,1) = veq*cos(omega*t(1:500));
    vy(501+d:2500+d,1) = veq/sqrt(2);
    vy(2501+d:4001+d,1) = veq*cos(pi/4+omega*t(1:1501));
    theta = pi/2*ones(size(t,2),N+1);
    ax(d+1:500+d,1) = veq*omega*cos(omega*t(1:500));
    ax(501+d:2500+d,1) = zeros(2000,1);
    ax(2501+d:4001+d,1) = veq*omega*cos(pi/4+omega*t(1:1501));
    ay(d+1:500+d,1) = -veq*omega*sin(omega*t(1:500));
    ay(501+d:2500+d,1) = zeros(2000,1);
    ay(2501+d:4001+d,1) = -veq*omega*sin(pi/4+omega*t(1:1501));
    %% leader position and desired followers position
    for i = d:size(t,2)-1
        px(i+1,1) = px(i,1)+(vx(i,1)+vx(i+1,1))/2*tg;
        py(i+1,1) = py(i,1)+(vy(i,1)+vy(i+1,1))/2*tg;
    end
    pxd = zeros(size(t,2),N+1);
    pyd = zeros(size(t,2),N+1);
    for i = 1:layer
        node(i) = 4*i;%number of nodes in layer i
        for j = 1:node(i)
            pxd(:,2*i*(i-1)+j+1) = px(:,1)+3*i*cos(2*pi/node(i)*j);
            pyd(:,2*i*(i-1)+j+1) = py(:,1)+3*i*sin(2*pi/node(i)*j);
        end
    end
    for i = 2:N+1
        px(1:d,i) = pxd(1:d,i);
        py(1:d,i) = pyd(1:d,i);
        vx(1:d,i) = vx(1:d,1);
        vy(1:d,i) = vy(1:d,1);
    end
    pxd(:,1) = px(:,1);
    pyd(:,1) = py(:,1);
    dev_norm = zeros(size(t,2),N+1);
    %define followers dynamics in normal disturbance
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
            if j ~= size(px,2)%neighbour 2
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
            if j ~= size(py,2)
                pyVhFl = py(i-d,j+1);vyVhFl = vy(i-d,j+1);pydVhFl = pyd(i-d,j+1);
            else
                pyVhFl = py(i-d,j-2);vyVhFl = vy(i-d,j-2);pydVhFl = pyd(i-d,j-2);
            end
            %calculate the orentation for disturbance state dependence  
            theta(i,j) = theta(i-1,j)+((-1/2/l)*vx(i-1,j)*sin(theta(i-1,j))+(1/2/l)*vy(i-1,j)*cos(theta(i-1,j)))*tg;
            %control input
            xacc = ax(i-1,1)+w*(kp*(pxVhPr-pxVhlag-pxdVhPr+pxdVhlag)+kv*(vxVhPr-vxVhlag)+kp*(pxVhFl-pxVhlag-pxdVhFl+pxdVhlag)+kv*(vxVhFl-vxVhlag))...
            +kp0*(pxVhRef-pxVh-pxVhRef+pxdVh)+kv0*(vxVhRef-vxVh)+[1/m*cos(theta(i-1,j)) -l/I*sin(theta(i-1,j))]*[d1(i-1,j);d2(i-1,j)];
            yacc = ay(i-1,1)+w*(kp*(pyVhPr-pyVhlag-pydVhPr+pydVhlag)+kv*(vyVhPr-vyVhlag)+kp*(pyVhFl-pyVhlag-pydVhFl+pydVhlag)+kv*(vyVhFl-vyVhlag))...
            +kp0*(pyVhRef-pyVh-pyVhRef+pydVh)+kv0*(vyVhRef-vyVh)+[1/m*sin(theta(i-1,j)) l/I*cos(theta(i-1,j))]*[d1(i-1,j);d2(i-1,j)];
            vx(i,j) = vxVh+tg*xacc;
            vy(i,j) = vyVh+tg*yacc;
            px(i,j) = pxVh+(vxVh+vx(i,j))/2*tg;
            py(i,j) = pyVh+(vyVh+vy(i,j))/2*tg;
            dev_norm(i,j) = norm([px(i,j)-pxd(i,j),py(i,j)-pyd(i,j)],2);%hand position deviation for agent j at time step i
        end
    end
    state_dev(d) = max(dev_norm,[],'all')%maximum hand position deviation when delay is (d-1)*tg[s]
end
figure
plot(0:0.01:(d-1)*tg,state_dev)
set(gca,'FontSize',14)
xlabel({'$\tau(s)$'},'Interpreter','latex','FontSize',16)
ylabel({'$\max_i\Vert \eta_i(t)-\eta_i^d(t) \Vert_{\mathcal{L}_{\infty}}$'},'Interpreter','latex','FontSize',16)
