%Simulation of epidemic dynamics over the homogeneous time scale 
%P_{t_{σ_k},t_k}:=U^{+∞}_{k=0}[t_{σ_k}, t_k], t_k < t_{σ_k}< t_{k+1} ∀k.

% Get the network matrices: note that the Matlab function 'WattsStrogatz' 
% that can be found at https://www.mathworks.com/help/matlab/math/build-watts-strogatz-small-world-graph-model.html
% The above lines are commented out (see note below). Uncomment if you want
% to use matrices different from that of the paper.

% NOTE: if you want to use exactly the data from the paper, load the
% matrices L and P from the file "data.mat"

% h = WattsStrogatz(100,3,0.7);
% L = full(laplacian(h));
% P = eye(100,100);
%   for i = 1:70
%       r = randi([1 100],1,1);
%       P(r,r) =0;
%  end

sigmar = 1;
sigma = 0.1;
tildeL = sigma*L + sigmar*P;

% Preparing the time scale: this is obtained by perturbing the homogeneous
% time scale

tf = 15;
a = 1;
b = 0.2;
tin = [0:tf]*(a+b);
tin = tin(tin<tf);
tfin = tin + (1+0.0005*randn(1,length(tin)));
tfin = tfin(tfin<=tf);

%First simulation: continuous and then discrete. Plotting also starts here.

y0 = randn(100,1);
y0 = [-2;y0];
tspan = [tin(1) tfin(1)];
[t,y] = ode45(@(t,y) dstate(t,y,sigma*L,sigmar*P) ,tspan,y0);
yDelta = dstate_discrete(tin(2)-tfin(1),y(length(t),:)',sigma*L,sigmar*P);
plot(t,y(:,2:101))
hold on
plot(t,y(:,1),'k','lineWidth',3,'LineStyle',':')


% Start the iteration loop: contiuons/discrete simulations

for i = 2:length(tfin)-1
    y0 = yDelta;
    tspan = [tin(i) tfin(i)];
    [t_temp,y_temp] = ode45(@(t,y) dstate(t,y,sigma*L,sigmar*P) ,tspan ,y0');
    plot(t_temp,y_temp(:,2:101))
    hold on
    plot(t_temp,y_temp(:,1),'k','lineWidth',3,'LineStyle',':')
    t= [t;t_temp];
    y = [y;y_temp];
    yDelta = dstate_discrete(tin(i+1)-tfin(i),y(length(t),:)',sigma*L,sigmar*P);
end

xlabel('x')
ylabel('y')

% Check conditions from the paper

% This computes the \mu's from the simulation
for i = 1:length(tfin)
    mu(i) = tin(i+1)-tfin(i);
end
mu_max= max(mu(i)); %max \mu

eigs = eig(tildeL); %eifs of \tilde{L}

% Numerical computation of the 2-measure, needed to check the second term
% in (42)
for i= 1:length(eigs)
measure(i) = 1/(2*mu_max)*(abs(1-2*mu_max*eigs(i))-1);
end

barS = 1; % comes from the derivative of S(x)
d= 0.9;
cbar2 = max(-d+barS, -1/mu_max+d)+max(measure); 
%The above has to be negative in order to satisfy the condition, the output
%is displayed next
disp(['The value of $-\bar{c}^2$ in (42) of the paper is:', num2str(cbar2)])

% Function for the time scale dynamics when \mu(t)=0

function dydt = dstate (t,y,sL,sP)
d = 0.9;
yr = y(1);
yn = y(2:101);
dyrdt = -d*yr +atan(yr);
dyndt = -d*yn + atan(yn) - sL*yn - sP*yn + sP*(ones(100,1)*yr);
dydt = [dyrdt; dyndt];
end

% Function for the time scale dynamics when \mu(t) \ne 0

function yDelta = dstate_discrete (mu,y,sL,sP)
d = 0.9;
yr = y(1);
yn = y(2:101);
yrDelta = yr + mu*(-d*yr +atan(yr));
ynDelta = yn + mu*(-d*yn + atan(yn) - sL*yn - sP*yn + sP*(ones(100,1)*yr));
yDelta = [yrDelta; ynDelta];
end
