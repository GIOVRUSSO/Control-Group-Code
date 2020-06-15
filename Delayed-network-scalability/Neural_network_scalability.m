clc
close all

N = 60;%number of neurons
%the condition 2&3&4 are satisfied with the following chosen parameters
c = 10;
w1 = zeros(N,N);%the weights for the delayed communication
w2 = csvread('w2.csv');%the weights for the delayed communication
%define the simulation time
time = 40;tg = 0.01;t = 0:tg:time;lengtht = size(t,2);
d = 101;%communication delay=(d-1)*tg
dis = zeros(lengtht,N);
perturbedID = randsample(N,N-5);%select the N-5 perturbed neurons
%disturbance in time interval [5s,6s] and [15s,16s]
for i = 1:N-5
    dis(1501+d:1601+d,perturbedID(i))=10*rand(1);
    dis(501+d:601+d,perturbedID(i))=10*rand(1);
end
u = csvread('u.csv');%the constant interger input
x = zeros(lengtht,N);%initial state
%% equilibrium
for i = d+1:lengtht
    for j = 1:N
        dotx(i,j) = -c*x(i-1,j)+w1(j,:)*tanh(x(i-1,:))'+w2(j,:)*tanh(x(i-d,:))'+u(j);
        x(i,j) = x(i-1,j)+dotx(i,j)*tg;
    end
end
figure
for i = 1:N
    plot(t(d+1:end),x(d+1:end,i));hold on
end
set(gca,'FontSize',14);xlabel('t[s]','FontSize',16);ylabel('x_i','FontSize',16)
%compute the equilibrium
y = sym('y',[1 N]);
xequ = struct2array(solve(-c*eye(N)*y'+w1*tanh(y)'+w2*tanh(y)'+u==zeros(N,1)));
%% perturbed network
for i = d+1:lengtht
    for j = 1:N
        dotx(i,j) = -c*x(i-1,j)+w1(j,:)*tanh(x(i-1,:))'+w2(j,:)*tanh(x(i-d,:))'+u(j)+dis(i,j);
        x(i,j) = x(i-1,j)+dotx(i,j)*tg;
    end
end
figure
for i = 1:N
    if any(i==perturbedID)
        ln1 = plot(t(d+1:end),(x(d+1:end,i)-xequ(i)),'k');hold on
    else
        ln2 = plot(t(d+1:end),(x(d+1:end,i)-xequ(i)),'r');
    end
end
set(gca,'FontSize',14);xlabel('t[s]','FontSize',16);ylabel('Deviation','FontSize',16)
legend([ln1,ln2],{'Perturbed neurons','Unperturbed neurons'})
