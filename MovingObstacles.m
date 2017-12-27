close
clear all

%prosoxi oi arxikes thesis ton robotna upakoyn to |p1 - p(i+1)|<= dmax
%u < umax
%h ≥ 2 and p > 0

GoalPoint = [10 -4]; % Define Goal Point
umax = 2;
gama = 2;
num_obstacles = 2;
num_robots = 3;
ro = 1; % Maximum range in which robot can scan the workspace
do = 0.9; % The desires distance between any two consecutive robots when they are standing
th = 0.1; % The headway time parameter
p = 1;
h = 2;
dmax = 3;

%% Time (sec)
dt = 0.5;
time = 0:dt:50;


%% Obstacle 1
radius_Obs1 = 1;
P_Obs1 = zeros(length(time),3);
crc = @(c,r,p,a) [c(1) + r*cosd(a + p); c(2) +  r*sind(a + p)];
init_pos = -90; % Initial position (�)
radius = 6;
velocity1 = 12; % Constant rotational velocity 
center = [0 0];
locus = crc(center,radius, init_pos, -velocity1*time);
for t = 1:length(time)
    P_Obs1(t,1) = locus(1,t);
    P_Obs1(t,2) = locus(2,t);
    P_Obs1(t,3) = FindHeadingAngle(P_Obs1(t,:),center); % Inverse tangent in degrees (�)  
end

%% Obstacle 2
radius_Obs2 = 1;
P_Obs2 = zeros(length(time),3);
velocity2 = 0.75; % Constant longitudinal velocity
P_Obs2(1,:) = [-14 -4 0]; % Initial position
for i = 2:length(time)
    P_Obs2(i,1) = P_Obs2(i-1,1) + velocity2*dt;    % Update position
    P_Obs2(i,2) = P_Obs2(i-1,2) + velocity2*dt; 
    P_Obs2(i,3) = atand((P_Obs2(i,2) - P_Obs2(i-1,2))/((P_Obs2(i,1) - P_Obs2(i-1,1)))); % The heading angle of the vehicle
end


%% Robot  
num_robots = 3;
velocity_robot  = 2; % Robot constant longitudinal velocity
radius_robot = 2;
P_Robot = zeros(length(time),3);
P_Robot2 = zeros(length(time),3);
P_Robot3 = zeros(length(time),3);
P_Robot(1,:) = [-14 2 0]; % Initial position Robot - Leader
P_Robot2(1,:) = [-15 2 0]; % Initial position Robot - Follower1
P_Robot3(1,:) = [-17 2 0]; % Initial position Robot - Follower2
for i = 2:length(time)
    P_Obs_all = [P_Obs1(i-1,:); P_Obs2(i-1,:); P_Robot2(i-1,:); P_Robot3(i-1,:)];
    u_Obs_all = [velocity1; velocity2; velocity_robot; velocity_robot];
    radius_Obs_all = [radius_Obs1; radius_Obs2; radius_robot; radius_robot];  
    % Leader - Robot 1
    P_Robot(i,:) = H4_PotentialNavigation(num_obstacles + num_robots -1,GoalPoint,P_Robot(i-1,:),radius_robot,velocity_robot,P_Obs_all,radius_Obs_all,u_Obs_all,gama,ro);
    % Follower - Robot 2
    P_Obs_all = [P_Obs1(i-1,:); P_Obs2(i-1,:); P_Robot3(i-1,:)];
    u_Obs_all = [velocity1; velocity2; velocity_robot];
    radius_Obs_all = [radius_Obs1; radius_Obs2; radius_robot];
    index = FindCloserObstacle(P_Robot2(i-1,:),P_Obs_all,num_obstacles);
    GoalPoint2 = goal_determination(do,th,velocity_robot,P_Robot2(i-1,:),P_Robot(i-1,:),P_Obs_all(index,:),radius_Obs_all(index));
    P_Robot2(i,:) = H4_PotentialNavigation(num_obstacles + num_robots - 2,GoalPoint2,P_Robot2(i-1,:),radius_robot,velocity_robot,P_Obs_all,radius_Obs_all,u_Obs_all,gama,ro);
    % Follower - Robot 3
    P_Obs_all = [P_Obs1(i-1,:); P_Obs2(i-1,:); P_Robot(i,:)];
    u_Obs_all = [velocity1; velocity2; velocity_robot];
    radius_Obs_all = [radius_Obs1; radius_Obs2; radius_robot];
    index = FindCloserObstacle(P_Robot3(i-1,:),P_Obs_all,num_obstacles);
    GoalPoint3 = goal_determination(do,th,velocity_robot,P_Robot3(i-1,:),P_Robot2(i-1,:),P_Obs_all(index,:),radius_Obs_all(index)); 
    P_Robot3(i,:) = H4_PotentialNavigation(num_obstacles + num_robots - 2,GoalPoint3,P_Robot3(i-1,:),radius_robot,velocity_robot,P_Obs_all,radius_Obs_all,u_Obs_all,gama,ro);
end


%% Plot Trajectories
figure
for t = 2:length(time)
    %plot(P_Obs1(:,1),P_Obs1(:,2),'--b',P_Obs1(t,1),P_Obs1(t,2),'bo',P_Obs2(:,1),  P_Obs2(:,2),'--m',P_Obs2(t,1),P_Obs2(t,2), 'ms');
    plot(P_Robot(:,1),P_Robot(:,2),'r',P_Robot2(:,1),P_Robot2(:,2),'b',P_Robot3(:,1),P_Robot3(:,2),'m',P_Obs1(:,1),P_Obs1(:,2),'k',P_Obs2(:,1),P_Obs2(:,2),'k')
    % robot range
    robot_range = animatedline('LineWidth',1,'MarkerSize',ro*10,'Color','r');
    addpoints(robot_range,P_Robot(t,1),P_Robot(t,2))
    robot_range.Marker = 'o';  
    %first robot
    robot_range = animatedline('Markerface','r','Color','r');
    addpoints(robot_range,P_Robot(t,1),P_Robot(t,2))
    robot_range.Marker = 'o';   
    % second robot
    robot_range = animatedline('Markerface','b','Color','b');
    addpoints(robot_range,P_Robot2(t,1),P_Robot2(t,2))
    robot_range.Marker = 'o';
    robot_range = animatedline('LineWidth',1,'MarkerSize',ro*10,'Color','b');
    addpoints(robot_range,P_Robot2(t,1),P_Robot2(t,2))
    robot_range.Marker = 'o'; 
    % third robot
    robot_range = animatedline('Markerface','m','Color','m');
    addpoints(robot_range,P_Robot3(t,1),P_Robot3(t,2))
    robot_range.Marker = 'o';
    robot_range = animatedline('LineWidth',1,'MarkerSize',ro*10,'Color','m');
    addpoints(robot_range,P_Robot3(t,1),P_Robot3(t,2))
    robot_range.Marker = 'o';     
    % goal 
    goal = animatedline('Markerface','r','Color','r');
    addpoints(goal,GoalPoint(1),GoalPoint(2))
    goal.Marker = 'o';
    % obstacles
    obstacles = animatedline('Markerface','k','Color','k');
    addpoints(obstacles,P_Obs1(t,1),P_Obs1(t,2))
    obstacles.Marker = 's';
    obstacles = animatedline('Markerface','k','Color','k');
    addpoints(obstacles,P_Obs2(t,1),P_Obs2(t,2))
    obstacles.Marker = 's';
    
    title('Plot Trajectories');
    xlabel('x');
    ylabel('y')
    %axis([-1 4 -7 ]);
    axis([-15 12 -15 12]);
    %axis([-8 -4 0 5]);
    grid
    axis square
    drawnow
    pause(0.1); %wait 0.01 seconds so the plot is displayed
end
