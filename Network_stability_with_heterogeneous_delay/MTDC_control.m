clear all
close all

N = 5;
ts = 0.001;
delay = [0 randi(300)/1000 0 0 randi(300)/1000;
         randi(300)/1000 0 randi(300)/1000 0 0;
         0 randi(300)/1000 0 randi(300)/1000 0;
         0 0 randi(300)/1000 0 randi(300)/1000;
         randi(300)/1000 0 0 randi(300)/1000 0];
%         [0 0.043 0 0 0.127;
%          0.275 0 0.238 0 0;
%          0 0.288 0  0.197 0;
%          0 0 0.011 0 0.255;
%          0.281 0 0 0.204 0];
delayind = fix(delay/ts);
iterations = 40000+300;
time = 0:ts:(iterations-1)*ts;

%creat state vectors
vi = zeros(N, iterations);
for i = 1:N
    vi(i, 1:301) = normrnd(0,1);
end
dvi = zeros(N, iterations);
ui = zeros(N, iterations);

dr2 = zeros(N, iterations);
r2 = zeros(N, iterations);

dr1 = zeros(N, iterations);
r1 = zeros(N, iterations);

% control gains
k0 = 0.8427;
k1 = 1.4093;
k2 = 0.5314;
g0 = 0.0031;
g1 = 0.0041;
g2 = 0.0026;

% coupling matrix
L = [0 1 0 0 1;
     1 0 1 0 0;
     0 1 0 1 0;
     0 0 1 0 1;
     1 0 0 1 0];
%% disturbances
d =  zeros(N, length(time));
t1 = 0:0.001:40-0.001;
d(1, 301:length(time)) = 3 + t1 + sin(t1).*exp(-0.2*t1);
%% iterations
for k = 301:iterations
    for i = 1:N  
        % get the topological neighbors of agent i        
        neighbors = find(L(i,:) ~= 0);   

        dr2(i, k-1) = -k2 * vi(i, k-1);
        dr1(i, k-1) = -k1 * vi(i, k-1);
        ui(i, k-1) = -k0 * vi(i, k-1);

        for j = neighbors
            dr2(i, k-1) = dr2(i, k-1) - g2 * (vi(i, k-delayind(i,j)) - vi(j, k-delayind(i,j)));
            dr1(i, k-1) = dr1(i, k-1) - g1 * (vi(i, k-delayind(i,j)) - vi(j, k-delayind(i,j)));
            ui(i, k-1) = ui(i, k-1) - g0 * (vi(i, k-delayind(i,j)) - vi(j, k-delayind(i,j)));
        end    
        r2(i, k) = r2(i, k-1) + dr2(i, k-1) * ts;
        dr1(i, k-1) = dr1(i, k-1) + r2(i, k-1);        
        r1(i, k) = r1(i, k-1) + dr1(i, k-1) * ts;
        ui(i, k-1) = ui(i, k-1) + r1(i, k-1);
        
        for j = neighbors
            dvi(i, k-1) = - (vi(i, k-1) - vi(j, k-1));
        end 
        dvi(i, k-1) = dvi(i, k-1) + ui(i, k-1) + d(i, k-1);
   
        vi(i, k) = vi(i, k-1) + dvi(i, k-1)*ts; 
       
    end     
end


%% plot figures
set(0,'DefaultAxesFontSize',8)
set(0,'DefaultFigureColor','w')
set(0,'defaulttextinterpreter','tex')
set(0, 'DefaultAxesFontName', 'times new roman');
fig=figure
set(fig, 'Units','centimeters')
set(fig,'Position', [0 0 8.89 6])

subplot(2,1,1)
plot(time(1:iterations-300),(vi(1,301:end)),'color',[0 0.4470 0.7410]); hold on
plot(time(1:iterations-300),(vi(2,301:end)),'color',[0.8500 0.3250 0.0980]); hold on
plot(time(1:iterations-300),(vi(3,301:end)),'color',[0.9290 0.6940 0.1250]); hold on
plot(time(1:iterations-300),(vi(4,301:end)),'color',[0.4940 0.1840 0.5560]); hold on
plot(time(1:iterations-300),(vi(5,301:end)),'color',[0.4660 0.6740 0.1880]); hold on

ylabel({'$v_i [V]$'},'Interpreter','latex')
subplot(2,1,2)
plot(time(1:iterations-300),(ui(1,301:end)),'color',[0 0.4470 0.7410]); hold on
plot(time(1:iterations-300),(ui(2,301:end)),'color',[0.8500 0.3250 0.0980]); hold on
plot(time(1:iterations-300),(ui(3,301:end)),'color',[0.9290 0.6940 0.1250]); hold on
plot(time(1:iterations-300),(ui(4,301:end)),'color',[0.4940 0.1840 0.5560]); hold on
plot(time(1:iterations-300),(ui(5,301:end)),'color',[0.4660 0.6740 0.1880]); hold on
xlim([0,40])
xlabel('$t[s]$','Interpreter','latex')
ylabel({'$u_i [A]$'},'Interpreter','latex')
axes('Position',[0.6 0.25 .3 .15])
plot(time(1:iterations-25300),(ui(1,301:15300)),'color',[0 0.4470 0.7410]); hold on
plot(time(1:iterations-25300),(ui(2,301:15300)),'color',[0.8500 0.3250 0.0980]); hold on
plot(time(1:iterations-25300),(ui(3,301:15300)),'color',[0.9290 0.6940 0.1250]); hold on
plot(time(1:iterations-25300),(ui(4,301:15300)),'color',[0.4940 0.1840 0.5560]); hold on
plot(time(1:iterations-25300),(ui(5,301:15300)),'color',[0.4660 0.6740 0.1880]); hold on
ylim([-1,1])
xlim([0,15])


fig.PaperUnits = 'centimeters';  
fig.PaperPosition = [0 0 8.89 6]; 
fig.Units = 'centimeters'; 
fig.PaperSize=[8.89 6];  
print(fig, '-dpsc2','terminals', '-painters')
