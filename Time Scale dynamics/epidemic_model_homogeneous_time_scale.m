%Simulation of epidemic dynamics over the homogeneous time scale 
%P_{a,b} = U_{k=0}^∞[k(a+b),k(a+b)+a] (note: is either 0 or mu = b)

% Preparing the time scale

tf = 10;
a = 1;
b = 0.24;
tin = [0:tf]*(a+b);
tin = tin(tin<tf);
tfin = [0:tf]*(a+b) + a;
tfin = tfin(tfin<=tf);

% First simulation is in continuous time and then discrete

y0 = [5 5 5 5];
tspan = [tin(1) tfin(1)];
[t,y] = ode45(@dstate ,tspan ,y0);
yDelta = dstate_discrete(b,y(length(t),:));

% Plotting on the same figure the results

colorstring = 'rgbk';
for j =1:4
    plot(t,y(:,j),'Color',colorstring(j))
    hold on
end

% Start the iteration loop: contiuons/discrete simulations

for i = 2:length(tfin)
    y0 = yDelta;
    tspan = [tin(i) tfin(i)];
    [t_temp,y_temp] = ode45(@dstate ,tspan ,y0);
    for j =1:4
        plot(t_temp,y_temp(:,j),'Color',colorstring(j))
        hold on
    end
    t= [t;t_temp];
    y = [y;y_temp];
    yDelta = dstate_discrete(b,y(length(t),:));
end

% Adding lengend and lables

xlabel('x')
ylabel('y')
legend('S(t)','I(t)','Q(t)','R(t)','FontSize',16)

% Function for the time scale dynamics when \mu(t)=0

function dydt = dstate (t,y)
alpha1 = 1;
alpha2 = 1;
Lambda=10;
beta = 0.1;
d = 1;
zeta = 1;
epsilon = 0.1;
gamma = 0.1;
S = y(1);
I = y(2);
Q = y(3);
R = y(4);

dSdt = Lambda -beta*S*I - d*S;
dIdt = beta*S*I - (gamma + zeta +d + alpha1)*I;
dQdt = zeta*I-(d+alpha2+epsilon)*Q;
dRdt = gamma*I + epsilon*Q - d*R;

dydt = [dSdt, dIdt, dQdt, dRdt]';
end

% Function for the time scale dynamics when \mu(t) \ne 0

function yDelta = dstate_discrete (mu,y)
alpha1 = 1;
alpha2 = 1;
Lambda=10;
beta = 0.1;
d = 1;
zeta = 1;
epsilon = 0.1;
gamma = 0.1;
S = y(1);
I = y(2);
Q = y(3);
R = y(4);

SDelta = S + mu*(Lambda -beta*S*I - d*S);
IDelta = I + mu*(beta*S*I - (gamma + zeta +d + alpha1)*I);
QDelta = Q + mu*(zeta*I-(d+alpha2+epsilon)*Q);
RDelta = R + mu*(gamma*I + epsilon*Q - d*R);

yDelta = [SDelta, IDelta, QDelta, RDelta]';
end
