%Simulation of epidemic dynamics over a discrete time scale 
% with randomnly chosen values of \mu(t)


% Preparing the time scale

tf = 10;
b = 0.24;
mu = (b).*rand(100,1);
t(1) = 0;
for j =2:100
    t = [t;t(j-1)+mu(j-1)];
end

% First time step and then plot

y0 = [5 5 5 5];
yDelta = dstate_discrete(mu(1),y0);
colorstring = 'rgbk';
for j =1:4
    plot(t(1),yDelta(j,:),'o','Color',colorstring(j))
    hold on
end

% Start the iteration loop

for i = 2:length(t)
    y0 = yDelta;
    yDelta = dstate_discrete(mu(i),y0);
    for j =1:4
        plot(t(i),yDelta(j,:),'o','Color',colorstring(j))
        hold on
    end   
end

% Adding lengend and lables

xlabel('x')
ylabel('y')
legend('S(t)','I(t)','Q(t)','R(t)','FontSize',16)
axis([0,10,0,10])

% Function for the time scale dynamics when \mu(t) \ne 0

function yDelta = dstate_discrete (mu,y0)
alpha1 = 1;
alpha2 = 1;
Lambda=10;
beta = 0.1;
d = 1;
zeta = 1;
epsilon = 0.1;
gamma = 0.1;
S = y0(1);
I = y0(2);
Q = y0(3);
R = y0(4);

SDelta = S + mu*(Lambda -beta*S*I - d*S);
IDelta = I + mu*(beta*S*I - (gamma + zeta +d + alpha1)*I);
QDelta = Q + mu*(zeta*I-(d+alpha2+epsilon)*Q);
RDelta = R + mu*(gamma*I + epsilon*Q - d*R);

yDelta = [SDelta, IDelta, QDelta, RDelta]';
end
