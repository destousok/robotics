close
clear all

GoalPoint = [10 -4]; % Define Goal Point
umax = 2;
gama = 2;
num_obstacles = 2;
num_robots = 3;
ro = 5; % Maximum range in whic� robot can scan the workspace
do = 1.2; % The desires distance between any two consecutive robots when they are standing
th = 0.1; % The headway time parameter
p = 1;
h = 2;
dmax = 3;

%% Time (sec)
dt = 0.5;
maxtime = 50;
time = 0:dt:maxtime;


%% Obstacle 1
u_ob1 = 1;
direction_coef1 = 0.9;

radius_Obs1 = 0.5;
P_Obs1 = zeros(length(time),3);
P_Obs1(1,:) = [-10 -7 rand(1)]; % Initial position
rng('shuffle');
%ang =[0; 2*pi*(rand(mxS,1))];
ang1(1) = P_Obs1(1,3);
for iter1 = 2:maxtime
    if rand(1) > direction_coef1
        ang1(iter1) = 2*pi*(rand(1));
    else
        ang1(iter1) = ang1(iter1-1);
    end
end
%u = rand(60,1)
u1 = 1;
%genika an afhsoume thn taxythta statherh den exoume na allaxoume kati
poX1 = cumsum([P_Obs1(1,1); (u1 .* cos(ang1')) * u_ob1 ]);
poY1 = cumsum([P_Obs1(1,2); (u1 .* sin(ang1')) * u_ob1 ]);
vel1(1) =  sqrt((poY1(1,1) - poY1(2,1))^2 + (poX1(1,1) - poX1(2,1))^2);
for i = 2:maxtime-1
    P_Obs1(i,3) = ang1(i);
    P_Obs1(i,1) = poX1(i,1);    % Update position
    P_Obs1(i,2) = poY1(i,1); 
    vel1(i) =  sqrt((poY1(i) - poY1(i+1))^2 + (poX1(i) - poX1(i+1))^2); 
end
  P_Obs1(maxtime,1) = poX1(maxtime,1);    % Update position
  P_Obs1(maxtime,2) = poY1(maxtime,1); 
  P_Obs1(maxtime,3) = ang1(maxtime); % The heading angle of the vehicle
  vel1(length(time)) = 0; 
  vel1 = vel1./dt;
  velocity1 = vel1(1);

%% Obstacle 2
u_ob2 = 1;
direction_coef = 0.95;

radius_Obs2 = 0.5;
P_Obs2 = zeros(length(time),3);
P_Obs2(1,:) = [2 2 rand(1)]; % Initial position
rng('shuffle');
%ang =[0; 2*pi*(rand(mxS,1))];
ang(1) = P_Obs2(1,3);
for iter = 2:maxtime
    if rand(1) > direction_coef
        ang(iter) = 2*pi*(rand(1));
    else
        ang(iter) = ang(iter-1);
    end
end
%u = rand(60,1)
u = 1;
%genika an afhsoume thn taxythta statherh den exoume na allaxoume kati
poX = cumsum([P_Obs2(1,1); (u .* cos(ang')) * u_ob2 ]);
poY = cumsum([P_Obs2(1,2); (u .* sin(ang')) * u_ob2 ]);
vel(1) =  sqrt((poY(1,1) - poY(2,1))^2 + (poX(1,1) - poX(2,1))^2);
for i = 2:maxtime
    P_Obs2(i,3) = ang(i);
    P_Obs2(i,1) = poX(i,1);    % Update position
    P_Obs2(i,2) = poY(i,1); 
    vel(i) =  sqrt((poY(i) - poY(i+1))^2 + (poX(i) - poX(i+1))^2); 
end
  P_Obs2(maxtime,1) = poX(maxtime,1);    % Update position
  P_Obs2(maxtime,2) = poY(maxtime,1); 
  P_Obs2(maxtime,3) = ang(maxtime); % The heading angle of the vehicle
  vel(length(time)) = 0; 
  vel = vel./dt;
  velocity2 = vel(1);


%% Robot  
num_robots = 3;
velocity_robot  = 0.75; % Robot constant longitudinal velocity
radius_robot = 1;
P_Robot = zeros(length(time),3);
P_Robot2 = zeros(length(time),3);
P_Robot3 = zeros(length(time),3);
P_Robot(1,:) = [-14 2 0]; % Initial position Robot - Leader
P_Robot2(1,:) = [-15 2 0]; % Initial position Robot - Follower1
P_Robot3(1,:) = [-17 2 0]; % Initial position Robot - Follower2

radius_Obs_all = [radius_Obs1; radius_Obs2];
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
