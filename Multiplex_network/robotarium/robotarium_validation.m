clear all 
close all

% Run the simulation for a specific number of iterations
delay = 11;
iterations = 2350 + delay;
ts = 0.033;
% number of robots
N = 12;
% coupling matrix
L = [0 1 0 1   0 0 0 0 0 0 0 0;
     1 0 1 0   0 0 0 0 0 0 0 0;
     0 1 0 1   0 0 0 0 0 0 0 0;
     1 0 1 0   0 0 0 0 0 0 0 0;
     
     1 0 0 0   0 1 0 0 0 0 0 1;
     1 0 0 0   1 0 1 0 0 0 0 0;
     0 1 0 0   0 1 0 1 0 0 0 0;
     0 1 0 0   0 0 1 0 1 0 0 0; 
     0 0 1 0   0 0 0 1 0 1 0 0;
     0 0 1 0   0 0 0 0 1 0 1 0;
     0 0 0 1   0 0 0 0 0 1 0 1;
     0 0 0 1   1 0 0 0 0 0 1 0];

%Initialize velocity vector
xl = zeros(3, iterations);
dxi = zeros(2, N, iterations);
dxl = zeros(2, iterations);
dr1 = zeros(2, N, iterations);
r1 = zeros(2, N, iterations);
dr0 = zeros(2, N, iterations);
r0 = zeros(2, N, iterations);
xd = zeros(2, N, iterations);
xi = zeros(3, N, iterations);

%constant linear speed throughout
veq = 0.04;

% reference trajectory
dxl(:, 1:1000) = [veq * ones(1, 1000); zeros(1, 1000)];
for k = 1:200
    dxl(:, 1000+k) = [veq*(1-k/200); -veq*k/200];
end
for k = 1:200
    dxl(:, 1200+k) = [-veq*k/200; -veq*(1-k/200)];
end
dxl(:, 1401:iterations + 2) = [-veq * ones(1, 952+delay); zeros(1, 952+delay)];

xl(:, 1:delay+1) = [-0.7*ones(1,delay+1); 0.09*ones(1,delay+1); 0*ones(1,delay+1)];

for t = delay+1:iterations
    xl(1:2, t+1) = xl(1:2, t) + (dxl(:, t)+dxl(:,t+1))/2 * ts;
end

for i = 1:4 
    xd(1:2, i, 1:iterations) = xl(1:2, 1:iterations) + 0.4 * [cosd(i * 90); sind(i * 90)];
end
for i = 5:12
    xd(1:2, i, 1:iterations) = xl(1:2, 1:iterations) + 0.75 * [cosd(90 + 45 * (i-5)); sind(90 + 45 * (i-5))];
end

initial_positions = generate_initial_conditions(N, 'Width', 3.2, 'Height', 2, 'Spacing', 0.3);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

% These are gains for our formation control algorithm
k0 = 1.2674;
k1 = 0.6312;
k2 = 0.133;
k0tau = 0.325;
k1tau = 0.162;
k2tau = 0.06;
kpsi = 0.1;

%% Grab tools we need to convert from single-integrator to unicycle dynamics
% Single-integrator -> unicycle dynamics mapping
si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 1);
% Single-integrator barrier certificates
uni_barrier_cert = create_uni_barrier_certificate_with_boundary();      
%Get randomized initial conditions in the robotarium arena
final_goal_points = zeros(3, N);
final_goal_points(1:2, :) = xd(1:2,:,delay+1);

args = {'PositionError', 0.02, 'RotationError', 0.02};
init_checker = create_is_initialized(args{:});
controller = create_waypoint_controller(args{:});

% Get initial location data for while loop condition.
x=r.get_poses();
r.step();

while(~init_checker(x, final_goal_points))
    x = r.get_poses();
    dxu = controller(x, final_goal_points);
    dxu = uni_barrier_cert(dxu, x);      

    r.set_velocities(1:N, dxu);
    r.step();   
end

%% start the robots from static condition
for k = delay+2:50
    % Retrieve the most recent poses from the Robotarium.  The time delay is approximately 0.033 seconds
    x = r.get_poses(); 
    xi(:, :, k) = x;  
    %% Algorithm
    for i = 1:N  
       %get the topological neighbors of agent i        
        neighbors = find(L(i,:) ~= 0);   
        %leader coupling
        dr1(:, i, k) = k2 * (xl(1:2, k) - xi(1:2, i, k) - (xl(1:2, k) - xd(1:2, i, k)));
        dr0(:, i, k) = k1 * (xl(1:2, k) - xi(1:2, i, k) - (xl(1:2, k) - xd(1:2, i, k)));
        dxi(:, i, k) = k0 * (xl(1:2, k) - xi(1:2, i, k) - (xl(1:2, k) - xd(1:2, i, k))) + dxl(:, k);
        %neighbour coupling
        for j = neighbors
            dr1(:, i, k) = dr1(:, i, k) + k2tau * tanh(kpsi*((xi(1:2, j, k-delay) - xi(1:2, i, k-delay)) - (xd(1:2, j, k-delay) - xd(1:2, i, k-delay))));
            dr0(:, i, k) = dr0(:, i, k) + k1tau * tanh(kpsi*((xi(1:2, j, k-delay) - xi(1:2, i, k-delay)) - (xd(1:2, j, k-delay) - xd(1:2, i, k-delay))));
        end    
        r1(:, i, k) = r1(:, i, k-1) + dr1(:, i, k-1) * ts;
        dr0(:, i, k) = dr0(:, i, k) + r1(:, i, k);        
        r0(:, i, k) = r0(:, i, k-1) + dr0(:, i, k-1) * ts;
        dxi(:, i, k) = dxi(:, i, k) + r0(:, i, k);    
        %control input--velocity
        for j = neighbors
            dxi(:, i, k) = dxi(:, i, k) + k0tau * tanh(kpsi*((xi(1:2, j, k-delay) - xi(1:2, i, k-delay)) - (xd(1:2, j, k-delay) - xd(1:2, i, k-delay))));
        end
    end 
    %% Use barrier certificate and convert to unicycle dynamics
    dxu = si_to_uni_dyn(dxi(:, :, k),x);
    dxu = uni_barrier_cert(dxu, x);   
    %% Send velocities to agents 
    %Set velocities
    r.set_velocities(1:N, dxu);    
    %Iterate experiment   
    r.step();   
end

%% start simulation and data collection
%disturbances
time = 0:0.033:120;
d =  zeros(2, N, length(time));
t1 = 0:0.033:120;
d(:, 1, 1:length(t1)) = [0.04*ones(1,length(t1)) + 0.4*sin(0.5*t1).*exp(-0.1*t1);0.04*ones(1,length(t1)) + 0.4*sin(0.5*t1).*exp(-0.1*t1)];
d(:, 3, 1:length(t1)) = [-0.05*t1 + 0.4*sin(0.5*t1).*exp(-0.1*t1); -0.05*t1+ 0.4*sin(0.5*t1).*exp(-0.1*t1)];

% count the time step in hardware experiment using tictoc at every iteration
tstart = tic;
tend = zeros(iterations,1);
stepsize = zeros(iterations,1);
for k = 51:iterations
    % Retrieve the most recent poses from the Robotarium.  The time delay is approximately 0.033 seconds
    x = r.get_poses(); 
    xi(:, :, k) = x;  
    %% Algorithm
    for i = 1:N
        %get the topological neighbors of agent i        
        neighbors = find(L(i,:) ~= 0);
        %leader coupling
        dr1(:, i, k) = k2 * (xl(1:2, k) - xi(1:2, i, k) - (xl(1:2, k) - xd(1:2, i, k)));
        dr0(:, i, k) = k1 * (xl(1:2, k) - xi(1:2, i, k) - (xl(1:2, k) - xd(1:2, i, k)));
        dxi(:, i, k) = k0 * (xl(1:2, k) - xi(1:2, i, k) - (xl(1:2, k) - xd(1:2, i, k))) + dxl(:, k);
        %neighbour coupling
        for j = neighbors
            dr1(:, i, k) = dr1(:, i, k) + k2tau * tanh(kpsi*((xi(1:2, j, k-delay) - xi(1:2, i, k-delay)) - (xd(1:2, j, k-delay) - xd(1:2, i, k-delay))));
            dr0(:, i, k) = dr0(:, i, k) + k1tau * tanh(kpsi*((xi(1:2, j, k-delay) - xi(1:2, i, k-delay)) - (xd(1:2, j, k-delay) - xd(1:2, i, k-delay))));
        end 
        r1(:, i, k) = r1(:, i, k-1) + dr1(:, i, k-1) * ts;
        dr0(:, i, k) = dr0(:, i, k) + r1(:, i, k);        
        r0(:, i, k) = r0(:, i, k-1) + dr0(:, i, k-1) * ts;
        %insert disturbance
        dxi(:, i, k) = dxi(:, i, k) + r0(:, i, k)+ d(:, i, (k-50));
        %control input--velocity
        for j = neighbors
            dxi(:, i, k) = dxi(:, i, k) + k0tau * tanh(kpsi*((xi(1:2, j, k-delay) - xi(1:2, i, k-delay)) - (xd(1:2, j, k-delay) - xd(1:2, i, k-delay))));
        end
    end
    %convert to unicycle dynamics
    dxu = si_to_uni_dyn(dxi(:, :, k),x); 
    %Send velocities to agents
    r.set_velocities(1:N, dxu);
    %Iterate experiment   
     r.step();
     tend(k) = toc(tstart);
     stepsize(k) = tend(k) - tend(k-1);%count stepsize

end
r.debug();
%% collect data from simulator/hardware
posdev = zeros(N, iterations);
xdev = zeros(N, iterations);
ydev = zeros(N, iterations);
for i = 1:N
    for j = 51:iterations
       posdev(i,j) = norm(xi(1:2,i,j) - xd(:,i,j)); 
       xdev(i,j) = xi(1,i,j) - xd(1,i,j); 
       ydev(i,j) = xi(2,i,j) - xd(2,i,j); 
    end
end

save('position_deviation.mat','posdev');
%save('control_effort.mat','dxi');
save('stepsize.mat','stepsize');

%% plot position deviation from simulator
set(0,'DefaultAxesFontSize',8)
set(0,'DefaultFigureColor','w')
set(0,'defaulttextinterpreter','tex')
set(0, 'DefaultAxesFontName', 'Times New Roman');
fig=figure
set(fig, 'Units','centimeters')
set(fig,'Position', [0 0 8.89 4.1])
set(gca, 'Units','centimeters')
set(gca, 'Position',[1.7 1 5.5 2.9])
time = 0:0.033:(iterations-51)*0.033;
plot(time, posdev(1,51:end),'k')
hold on
plot(time, posdev(2,51:end),'r')
plot(time, posdev(3,51:end),'k')
for i = 4:12
    plot(time, posdev(i,51:end),'r')
end
xlabel('$t[s]$','Interpreter','latex')
ylabel({'$\vert \eta_i(t)-\eta_i^*(t) \vert_2$'},'Interpreter','latex')
legend('Perturbed robots','Unperturbed robots')

fig.PaperUnits = 'centimeters';  
fig.PaperPosition = [0 0 8.89 4.1]; 
fig.Units = 'centimeters'; 
fig.PaperSize=[8.89 4.1];  
print(fig, '-dpsc2','robotarium_simulation.ps', '-painters')

%% load the data from hardware to plot in this section
%% plot position deviation from the data collected from robotarium hardware
load('posdev_hardware.mat')
set(0,'DefaultAxesFontSize',8)
set(0,'DefaultFigureColor','w')
set(0,'defaulttextinterpreter','tex')
set(0, 'DefaultAxesFontName', 'Times New Roman');
fig2=figure
set(fig2, 'Units','centimeters')
set(fig2,'Position', [0 0 8.89 4.1])
set(gca, 'Units','centimeters')
set(gca, 'Position',[1.7 1 5.5 2.9])

posdevmean = (posdev0+posdev1+posdev2+posdev3+posdev4+posdev5+posdev6+posdev7+posdev8+posdev9)/10;
%standard deviation
for i =1:N
   posdevstd(i,:) = std([posdev0(i,:)' posdev1(i,:)' posdev2(i,:)' posdev3(i,:)' posdev4(i,:)' posdev5(i,:)' posdev6(i,:)' posdev7(i,:)' posdev8(i,:)' posdev9(i,:)'],0,2);
end
%plot the confidence interval
for i =4:N
    patch([time fliplr(time)], [posdevmean(i,51:end)+posdevstd(i,51:end)  fliplr(posdevmean(i,51:end)-posdevstd(i,51:end))], [1 0.8 0.8], 'EdgeColor','none')
end
patch([time fliplr(time)], [posdevmean(2,51:end)+posdevstd(2,51:end)  fliplr(posdevmean(2,51:end)-posdevstd(2,51:end))], [1 0.8 0.8], 'EdgeColor','none')
patch([time fliplr(time)], [posdevmean(1,51:end)+posdevstd(1,51:end)  fliplr(posdevmean(1,51:end)-posdevstd(1,51:end))], [0.8 0.8 0.8], 'EdgeColor','none')
patch([time fliplr(time)], [posdevmean(3,51:end)+posdevstd(3,51:end)  fliplr(posdevmean(3,51:end)-posdevstd(3,51:end))], [0.8 0.8 0.8], 'EdgeColor','none')
%plot mean position deviation from experiments
plot(time, posdevmean(1,51:end),'k');hold on
plot(time, posdevmean(2,51:end),'r')
for i = 4:12
    plot(time, posdevmean(i,51:end), 'r')
end
plot(time, posdevmean(1,51:end),'k')
plot(time, posdevmean(3,51:end),'k')
ylim([0,0.4])
xlabel('$t[s]$','Interpreter','latex')
ylabel({'$\vert \eta_i(t)-\eta_i^*(t) \vert_2$'},'Interpreter','latex')
fig2.PaperUnits = 'centimeters';  
fig2.PaperPosition = [0 0 8.89 4.1]; 
fig2.Units = 'centimeters'; 
fig2.PaperSize=[8.89 4.1];  
print(fig2, '-dpsc2','robotarium_experiment.ps', '-painters')

%% plot step size from hardware experimnents
load('stepsize_hardware.mat')
time = 0:0.033:(iterations-51)*0.033;
set(0,'DefaultAxesFontSize',8)
set(0,'DefaultFigureColor','w')
set(0,'defaulttextinterpreter','tex') 
set(0, 'DefaultAxesFontName', 'Times New Roman');
fig3=figure
stepsizeall = [stepsize0(51:end) stepsize1(51:end) stepsize2(51:end) stepsize3(51:end) stepsize4(51:end) stepsize5(51:end) stepsize6(51:end) stepsize7(51:end) stepsize8(51:end) stepsize9(51:end)]';

meanstepsize = (stepsize0(51:end)+stepsize1(51:end)+stepsize2(51:end)+stepsize3(51:end)+stepsize4(51:end)+stepsize5(51:end)+stepsize6(51:end)+stepsize7(51:end)+stepsize8(51:end)+stepsize9(51:end))/10;
stepsizestd = zeros(iterations-50,1);
%standard deviation
for i=1:(iterations-50)
   stepsizestd(i) = std(stepsizeall(:,i));
end
%plot confidence interval
patch([time fliplr(time)], [meanstepsize'+stepsizestd'  fliplr(meanstepsize'-stepsizestd')], [1 0.8 0.8], 'EdgeColor','none')
ylim([0 0.2])
hold on
%plot mean stepsize
plot(time, meanstepsize,'r.','MarkerSize',6)

xlabel('$t[s]$','Interpreter','latex')
ylabel({'Step size[s]'},'Interpreter','tex')
box on
fig3.PaperUnits = 'centimeters';  
fig3.PaperPosition = [0 0 8.89 4.1]; 
fig3.Units = 'centimeters'; 
fig3.PaperSize=[8.89 4.1];  
print(fig3, '-dpsc2','robotarium_stepsize.ps', '-painters')

