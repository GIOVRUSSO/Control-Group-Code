clc

N = 50;%number of neurons
%the condition 2&3&4 are satisfied with the following chosen parameters
c = 27;
w1 = zeros(N,N);%the weights for the delayed communication
w2 = zeros(N,N);
for i = 3:N
    if mod(i,2)==1
        w2(i,i-2) = 15;
        w2(i,i-1) = 15;
    else
        w2(i,i-3) = 15;
        w2(i,i-2) = 15;
    end
end
%define the simulation time
time = 40;tg = 0.01;t = 0:tg:time;lengtht = size(t,2);
d = 11;%communication delay=(d-1)*tg
dis = zeros(lengtht,N);
perturbedID = [1 2];
for i = 1:2
    dis(:,perturbedID(i))=-10*sin(t).*exp(-0.2*t);
end
u = zeros(N,1);
u(1:2) = [7 1];%the constant interger input
x = zeros(lengtht,N);%initial state
dotx = zeros(lengtht,N);
y = sym('y',[N 1]);
xequ = struct2array(solve(-c*y+w1*tanh(y)+w2*tanh(y)+u==zeros(N,1)));
for i = 1:d
    x(i,:) = double(xequ);
end
%% equilibrium
for i = d+1:lengtht
    for j = 1:N
        dotx(i,j) = -c*x(i-1,j)+w1(j,:)*tanh(x(i-1,:)')+w2(j,:)*tanh(x(i-d,:)')+u(j);
        x(i,j) = x(i-1,j)+dotx(i,j)*tg;
    end
end
%% perturbed network
for i = d+1:lengtht
    for j = 1:N
        dotx(i,j) = -c*x(i-1,j)+w1(j,:)*tanh(x(i-1,:))'+w2(j,:)*tanh(x(i-d,:))'+u(j)+dis(i,j);
        x(i,j) = x(i-1,j)+dotx(i,j)*tg;
    end
end
figure
subplot(1,2,1)
for i = 1:N
    if any(i==perturbedID)
        print 0
    else
        ln2 = plot(0:tg:time-d*tg,(x(d+1:end,i)-double(xequ(i))),'Color',[1 0.02*i 0]);hold on
    end
end
set(gca,'FontSize',22);xlabel('$t[s]$','Interpreter','latex','FontSize',24);ylabel('$Deviations$','Interpreter','latex','FontSize',24)
ylim([-1.2 0.3])
yticks([-1.2 -0.9 -0.6 -0.3 0 0.3])
%% 
clc
N = 50;%number of neurons
%the condition 2&3&4 are satisfied with the following chosen parameters
c = 32;
w1 = zeros(N,N);%the weights for the delayed communication
w2 = zeros(N,N);
for i = 3:N
    if mod(i,2)==1
        w2(i,i-2) = 15;
        w2(i,i-1) = 15;
    else
        w2(i,i-3) = 15;
        w2(i,i-2) = 15;
    end
end
%define the simulation time
time = 40;tg = 0.01;t = 0:tg:time;lengtht = size(t,2);
d = 11;%communication delay=(d-1)*tg
dis = zeros(lengtht,N);
perturbedID = [1 2];
for i = 1:2
    dis(:,perturbedID(i))=-10*sin(t).*exp(-0.2*t);
end
u = zeros(N,1);
u(1:2) = [7 1];%the constant interger input
x = zeros(lengtht,N);%initial state
dotx = zeros(lengtht,N);
y = sym('y',[N 1]);
xequ = struct2array(solve(-c*y+w1*tanh(y)+w2*tanh(y)+u==zeros(N,1)));
for i = 1:d
    x(i,:) = double(xequ);
end
%% equilibrium
for i = d+1:lengtht
    for j = 1:N
        dotx(i,j) = -c*x(i-1,j)+w1(j,:)*tanh(x(i-1,:)')+w2(j,:)*tanh(x(i-d,:)')+u(j);
        x(i,j) = x(i-1,j)+dotx(i,j)*tg;
    end
end
%% perturbed network
for i = d+1:lengtht
    for j = 1:N
        dotx(i,j) = -c*x(i-1,j)+w1(j,:)*tanh(x(i-1,:))'+w2(j,:)*tanh(x(i-d,:))'+u(j)+dis(i,j);
        x(i,j) = x(i-1,j)+dotx(i,j)*tg;
    end
end
subplot(1,2,2)
for i = 1:N
    if any(i==perturbedID)
        print 0
    else
        ln2 = plot(0:tg:time-d*tg,(x(d+1:end,i)-double(xequ(i))),'Color',[1 0.02*i 0]);hold on
    end
end
set(gca,'FontSize',22);xlabel('$t[s]$','Interpreter','latex','FontSize',24);
ylim([-1.2 0.3])
yticks([-1.2 -0.9 -0.6 -0.3 0 0.3])
