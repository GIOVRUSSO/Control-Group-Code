clc
clear all 
close all

% Run the simulation for a specific number of iterations
iterations = 2350 + 11;
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
dxl(:, 1401:iterations + 2) = [-veq * ones(1, 952+11); zeros(1, 952+11)];

xl(:, 1:11+1) = [-0.7*ones(1,11+1); 0.09*ones(1,11+1); 0*ones(1,11+1)];

for t = 11+1:iterations
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
final_goal_points(1:2, :) = xd(1:2,:,11+1);

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
for k = 11+2:50
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
            dr1(:, i, k) = dr1(:, i, k) + k2tau * tanh(kpsi*((xi(1:2, j, k-11) - xi(1:2, i, k-11)) - (xd(1:2, j, k-11) - xd(1:2, i, k-11))));
            dr0(:, i, k) = dr0(:, i, k) + k1tau * tanh(kpsi*((xi(1:2, j, k-11) - xi(1:2, i, k-11)) - (xd(1:2, j, k-11) - xd(1:2, i, k-11))));
        end    
        r1(:, i, k) = r1(:, i, k-1) + dr1(:, i, k-1) * ts;
        dr0(:, i, k) = dr0(:, i, k) + r1(:, i, k);        
        r0(:, i, k) = r0(:, i, k-1) + dr0(:, i, k-1) * ts;
        dxi(:, i, k) = dxi(:, i, k) + r0(:, i, k);    
        %control input--velocity
        for j = neighbors
            dxi(:, i, k) = dxi(:, i, k) + k0tau * tanh(kpsi*((xi(1:2, j, k-11) - xi(1:2, i, k-11)) - (xd(1:2, j, k-11) - xd(1:2, i, k-11))));
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
%% Plotting Initialization
% Color Vector for Plotting
% Note the Robotarium MATLAB instance runs in a docker container which will 
% produce the same rng value every time unless seeded by the user.
CM = rand(N,3);%{'.-k','.-b','.-r','.-g','.-m','.-y','.-c'};
%Marker, font, and line sizes
marker_size_robot = determine_robot_marker_size(r);
font_size = determine_font_size(r, 0.05);
line_width = 5;
for i=1:N       
    robot_caption = sprintf('Robot %d', i);
    % Text with robot position information
    robot_details = sprintf('X-Pos: %d \nY-Pos: %d', x(1,i), x(2,i));
    % Plot colored circles showing robot location.
    g(i) = plot(x(1,i),x(2,i),'o','MarkerSize', marker_size_robot,'LineWidth',5,'Color',CM(i,:));
    % Plot the robot label text 
    robot_labels{i} = text(500, 500, robot_caption, 'FontSize', font_size, 'FontWeight', 'bold');
    % Plot the robot position information text
    robot_details_text{i} = text(500, 500, robot_details, 'FontSize', font_size, 'FontWeight', 'bold'); 
end
% Plot the iteration and time in the lower left. Note when run on your 
% computer, this time is based on your computers simulation time. For a
% better approximation of experiment time on the Robotarium when running
% this simulation on your computer, multiply iteration by 0.033. 
tstart = tic; %The start time to compute time elapsed.
iteration_caption = sprintf('Iteration %d', 0);
time_caption = sprintf('Total Time Elapsed %0.2f', toc(tstart));

iteration_label = text(-1.5, -0.8, iteration_caption, 'FontSize', font_size, 'Color', 'r', 'FontWeight', 'bold');
time_label = text(-1.5, -0.9, time_caption, 'FontSize', font_size, 'Color', 'r', 'FontWeight', 'bold');

% We can change the order of plotting priority, we will plot goals on the 
% bottom and have the iteration/time caption on top.
uistack([iteration_label], 'top'); % Iteration label is on top.
uistack([time_label], 'top'); % Time label is above iteration label.


%% start simulation and data collection
%record video
%% Set up and writing the movie.
% writerObj = VideoWriter('video_simulator', 'MPEG-4'); % movie name.
% writerObj.FrameRate = 540/4; % Frames per second. Larger number correlates to smaller movie time duration. 
% open(writerObj);
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
    delay = 0.1+0.1*sin(k*ts);
    delay = round(delay/ts);
    %% Algorithm
    for i = 1:N
        g(i).XData = x(1,i);
        g(i).YData = x(2,i);   
        robot_labels{i}.Position = x(1:2, i) + [-0.15;0.15];
        robot_details = sprintf('X-Pos: %0.2f \nY-Pos: %0.2f', x(1,i), x(2,i));
        robot_details_text{i}.String = robot_details;
        robot_details_text{i}.Position = x(1:2, i) - [0.2;0.25];
    
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
%     dxu = uni_barrier_cert(dxu, x);   
    %Send velocities to agents
    r.set_velocities(1:N, dxu);
    %Iterate experiment   
    r.step();
    tend(k) = toc(tstart);
    stepsize(k) = tend(k) - tend(k-1);%calculate stepsize
    
    % Update Iteration and Time marker
    iteration_caption = sprintf('Iteration %d', k);
    time_caption = sprintf('Total Time Elapsed %0.2f', toc(tstart));
    iteration_label.String = iteration_caption;
    time_label.String = time_caption;
    
%     frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
%     writeVideo(writerObj, frame)
end
r.debug();
% close(writerObj);% Saves the movie.
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
save('stepsize.mat','stepsize');


%% Helper Functions

% Marker Size Helper Function to scale size of markers for robots with figure window
% Input: robotarium class instance
function marker_size = determine_robot_marker_size(robotarium_instance)

% Get the size of the robotarium figure window in pixels
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Pixels');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the robot size to the x-axis (the axis are
% normalized so you could do this with y and figure height as well).
robot_ratio = (robotarium_instance.robot_diameter + 0.03)/...
    (robotarium_instance.boundaries(2) - robotarium_instance.boundaries(1));

% Determine the marker size in points so it fits the window. cursize(3) is
% the width of the figure window in pixels. (the axis are
% normalized so you could do this with y and figure height as well).
marker_size = cursize(3) * robot_ratio;

end

% Marker Size Helper Function to scale size with figure window
% Input: robotarium instance, desired size of the marker in meters
function marker_size = determine_marker_size(robotarium_instance, marker_size_meters)

% Get the size of the robotarium figure window in pixels
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Pixels');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the robot size to the x-axis (the axis are
% normalized so you could do this with y and figure height as well).
marker_ratio = (marker_size_meters)/(robotarium_instance.boundaries(2) -...
    robotarium_instance.boundaries(1));

% Determine the marker size in points so it fits the window. cursize(3) is
% the width of the figure window in pixels. (the axis are
% normalized so you could do this with y and figure height as well).
marker_size = cursize(3) * marker_ratio;

end

% Font Size Helper Function to scale size with figure window
% Input: robotarium instance, desired height of the font in meters
function font_size = determine_font_size(robotarium_instance, font_height_meters)

% Get the size of the robotarium figure window in point units
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Pixels');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the font height to the y-axis
font_ratio = (font_height_meters)/(robotarium_instance.boundaries(4) -...
    robotarium_instance.boundaries(3));

% Determine the font size in points so it fits the window. cursize(4) is
% the hight of the figure window in points.
font_size = cursize(4) * font_ratio;

end

