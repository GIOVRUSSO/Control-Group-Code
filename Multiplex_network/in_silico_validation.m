clear all
close all

layer = 30;
N = 2*layer*(layer+1);
iterations = 2350 + 50;
ts = 0.033;
time = 0:ts:(iterations-1)*ts;

delay = zeros(N, iterations);
for i = 1:N
    delay(i,:) = 0.1+0.1*sin(time);
    delay(i,:) = round(delay(i,:),3);
end
delayind = fix(delay/ts);

% control gains
k0 = 1.4155;
k1 = 1.5103;
k2 = 0.4803;
k0tau = 0.642;
k1tau = 0.872;
k2tau = 0.425;
kpsi = 0.1;

% constant linear speed
veq = 0.04;

%creat state vectors
xl = zeros(2, iterations);
vl = zeros(2, iterations);

xi = zeros(2, N, iterations);
vi = zeros(2, N, iterations);

dr1 = zeros(2, N, iterations);
r1 = zeros(2, N, iterations);

dr0 = zeros(2, N, iterations);
r0 = zeros(2, N, iterations);

xd = zeros(2, N, iterations);

xl(:, 1:50) = [-0.7*ones(1,50); 0.09*ones(1,50)];
vl(:, 1:50) = [veq * ones(1, 50); zeros(1, 50)];

% design movement
vl(:, 50+1:50+1000) = [veq * ones(1, 1000); zeros(1, 1000)];
for k = 1:200
    vl(:, 50+1000+k) = [veq*(1-k/200); -veq*k/200];
end
for k = 1:200
    vl(:, 50+1200+k) = [-veq*k/200; -veq*(1-k/200)];
end
vl(:, 50+1401:iterations) = [-veq * ones(1, 950); zeros(1, 950)];

% leader dynamics
for t = 50+1:iterations
    xl(:, t) = xl(1:2, t-1) + vl(:, t-1) * ts;
end

% desired follower position
for i = 1:layer
    for j = 1:4*i
        xd(:, 2*i*(i-1)+j, :) = xl(1:2, :) + 0.4 * i * [cosd(90 + (j-1) * 90 / i); sind(90 + (j-1) * 90 / i)];
    end
end

xi(:, :, 1:50+1) = xd(:, :, 1:50+1);

% coupling matrix
L = zeros(N, N);
for i = 1:layer
   L(2*i*(i-1)+1:2*i*(i-1)+4*i, 2*i*(i-1)+1:2*i*(i-1)+4*i) = triu(ones(4*i),1) -  triu(ones(4*i),2) + (triu(ones(4*i),1) -  triu(ones(4*i),2))';
   L(2*i*(i-1)+1, 2*i*(i-1)+4*i) = 1;
   L(2*i*(i-1)+4*i, 2*i*(i-1)+1) = 1;
end
for k = 1:4
   for i = 2:layer 
        for j = (k-1)*i+1 : k*i
            if j-(k-1)*i <= (i+1)/2
                L(2*i*(i-1)+j, 2*(i-1)*(i-2) + j-(k-1)) = 1;
            else
                L(2*i*(i-1)+j, 2*(i-1)*(i-2) + j-(k-1)-1) = 1;
            end
        end
   end
end

%% disturbances
d =  zeros(2, N, length(time));
t1 = 0:0.033:120;
d(:, 1, 1:length(t1)) = [0.04*ones(1,length(t1))+0.4*sin(0.5*t1).*exp(-0.1*t1); 0.04*ones(1,length(t1))+0.4*sin(0.5*t1).*exp(-0.1*t1)];
d(:, 3, 1:length(t1)) = [-0.05*t1+ 0.4*sin(0.5*t1).*exp(-0.1*t1); -0.05*t1+ 0.4*sin(0.5*t1).*exp(-0.1*t1)];

%% iterations
for k = 50+1:iterations
    for i = 1:N  
        % get the topological neighbors of agent i        
        neighbors = find(L(i,:) ~= 0);   
        % leader coupling
        dr1(:, i, k-1) = k2 * (xd(:, i, k-1) - xi(:, i, k-1));
        dr0(:, i, k-1) = k1 * (xd(:, i, k-1) - xi(:, i, k-1));
        vi(:, i, k) = k0 * (xd(:, i, k-1) - xi(:, i, k-1)) + vl(:, k);
        for j = neighbors
            dr1(:, i, k-1) = dr1(:, i, k-1) + k2tau * tanh(kpsi*((xi(:, j, k-delayind(i,k)) - xi(:, i, k-delayind(i,k))) - (xd(:, j, k-delayind(i,k)) - xd(:, i, k-delayind(i,k)))));
            dr0(:, i, k-1) = dr0(:, i, k-1) + k1tau * tanh(kpsi*((xi(:, j, k-delayind(i,k)) - xi(:, i, k-delayind(i,k))) - (xd(:, j, k-delayind(i,k)) - xd(:, i, k-delayind(i,k)))));
        end    
        r1(:, i, k) = r1(:, i, k-1) + dr1(:, i, k-1) * ts;
        dr0(:, i, k-1) = dr0(:, i, k-1) + r1(:, i, k);        
        r0(:, i, k) = r0(:, i, k-1) + dr0(:, i, k-1) * ts;
        vi(:, i, k) = vi(:, i, k) + r0(:, i, k) + d(:, i, k); 
        for j = neighbors
            vi(:, i, k) = vi(:, i, k) + k0tau * tanh(kpsi*((xi(:, j, k-delayind(i,k)) - xi(:, i, k-delayind(i,k))) - (xd(:, j, k-delayind(i,k)) - xd(:, i, k-delayind(i,k)))));
        end
        xi(:, i, k+1) = xi(:, i, k) + vi(:, i, k) * ts;
    end     
end

positiondev = zeros(N, iterations-50);
for i = 1:N
    for j = 1:iterations-50
       positiondev(i,j) = norm(xi(:,i,j+50) - xd(:,i,j+50)); 
    end
end
%% plot state deviation (unperturbed robots only)
set(0,'DefaultAxesFontSize',8)
set(0,'DefaultFigureColor','w')
set(0,'defaulttextinterpreter','tex')
set(0, 'DefaultAxesFontName', 'times new roman');
fig=figure
set(fig, 'Units','centimeters')
set(fig,'Position', [0 0 8.89 4.1])
set(gca, 'Units','centimeters')
set(gca, 'Position',[1.7 1 5 2.9])

plot(time(1:end-50),positiondev(2,:),'Color','r','linewidth',0.5);hold on
plot(time(1:end-50),positiondev(4,:),'Color','r','linewidth',0.5);hold on
for i = 2:layer
    plot(time(1:end-50),positiondev(2*i*(i-1)+1:2*i*(i+1),:),'Color',[1 -1/900*i*(i-60) 0],'linewidth',0.5);hold on
end
xlabel('$t[s]$','Interpreter','latex')
xlim([0,80])
ylim([0,0.025])
ylabel({'$\vert \eta_i(t)-\eta_i^*(t) \vert_2$'},'Interpreter','latex')
colormap(autumn)
H=colorbar('Ticks',[0,1],'TickLabels',{'Circle 1','Circle 30'})
set(H, 'Units','centimeters')
set(H,'Position',[6.9 1 0.3 2.9])

fig.PaperUnits = 'centimeters';  
fig.PaperPosition = [0 0 8.89 4.1]; 
fig.Units = 'centimeters'; 
fig.PaperSize=[8.89 4.1];  
print(fig, '-dpsc2','unperturbed_robots.ps', '-painters')

%% plot state deviation (all robots)
set(0,'DefaultAxesFontSize',8);
set(0,'DefaultFigureColor','w')
set(0,'defaulttextinterpreter','tex') 
set(0, 'DefaultAxesFontName', 'Times New Roman');
fig2=figure
set(fig2, 'Units','centimeters')
set(fig2,'Position', [0 0 8.89 4.1])
set(gca, 'Units','centimeters')
set(gca, 'Position',[1.7 1 5.5 2.9])

plot(time(1:end-50),positiondev(1,:),'Color','k','linewidth',0.5);hold on
plot(time(1:end-50),positiondev(2,:),'Color','r','linewidth',0.5)
plot(time(1:end-50),positiondev(3,:),'Color','k','linewidth',0.5)
plot(time(1:end-50),positiondev(4,:),'Color','r','linewidth',0.5)

for i = 2:layer
    plot(time(1:end-50),positiondev(2*i*(i-1)+1:2*i*(i+1),:),'r','linewidth',0.5);hold on
end

xlabel('$t[s]$','Interpreter','latex')
xlim([0,80])
ylabel({'$\vert \eta_i(t)-\eta_i^*(t) \vert_2$'},'Interpreter','latex')
legend('Perturbed robots','Unperturbed robots')

fig2.PaperUnits = 'centimeters';  
fig2.PaperPosition = [0 0 8.89 4.1]; 
fig2.Units = 'centimeters'; 
fig2.PaperSize=[8.89 4.1];  
print(fig2, '-dpsc2','all_robots.ps', '-painters')
