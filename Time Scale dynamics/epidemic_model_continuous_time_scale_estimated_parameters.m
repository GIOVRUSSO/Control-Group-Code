% Simulation of epidemic dynamics when the time scale is R
% Parameters are taken from 
% M. Pedersen, M. Meneghini, Quantifying  undetected  covid19  cases  
% and effects  of  containment  measures  in  Italy

% This is just going to be an ODE...
tf = 200;

N = 6*10^7;
y0 = [25*N/100 25*N/100 25*N/100 25*N/100];
tspan = [0, tf];
[t,y] = ode45(@dstate ,tspan ,y0);

plot(t,y)

xlabel('x')
ylabel('y')
legend('S(t)','I(t)','Q(t)','R(t)','FontSize',16)

% Check on the norm to verify contractivity in the 1-norm
figure
for i=1:length(y(:,1))
    err(i)= norm(y(i,:)-[N,0,0,0],1);
end
plot(t,err)

% Dynamics 
function dydt = dstate (t,y)
alpha1= 0;
alpha2 = 0;
beta = 0.0373; % this is 10% of the beta considered when there is no lockdown
N=6*10^7;
d = beta;
epsilon = 0.036;
zeta = 0.067;
gamma = 0.067;
Lambda = beta*N;

S = y(1);
I = y(2);
Q = y(3);
R = y(4);

dSdt = Lambda -beta*S*I/N - d*S;
dIdt = beta*S*I/N - (gamma + zeta +d + alpha1)*I;
dQdt = zeta*I-(d+alpha2+epsilon)*Q;
dRdt = gamma*I + epsilon*Q - d*R;

dydt = [dSdt, dIdt, dQdt, dRdt]';
end
