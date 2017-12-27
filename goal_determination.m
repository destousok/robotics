function [goalPoint] = goal_determination(d_o,th,u_robot,P_Robot,P_r_previous,P_Obs,radius_Obs) 

radius = inter_robot_distance(d_o,th,u_robot); % The circumference in the workspace centered in P_r_previous with this calculated radius

% Find the line between the positions of the robots
coef = polyfit([P_Robot(1),P_r_previous(1)],[P_Robot(2),P_r_previous(2)],1);
slope = coef(1);
constant = coef(2);
[x,y] = linecirc(slope,constant,P_r_previous(1),P_r_previous(2),radius);
%[x1,y1;x2,y2]

X1 = [P_Robot(1),P_Robot(2);x(1),y(1)];
X2 = [P_Robot(1),P_Robot(2);x(2),y(2)];
d1 = pdist(X1,'euclidean');
d2 = pdist(X2,'euclidean');
if d1 < d2
    goal = [x(1) y(1)];
else
    goal = [x(2) y(2)];
end

check = sqrt((goal(1)-P_Obs(1))^2 + (goal(2)-P_Obs(2))^2);
if check > radius_Obs
    goalPoint = goal;
else
    %find intersection points 
    [x_circle,y_circle] = circcirc(P_r_previous(1),P_r_previous(2),radius,P_Obs(1),P_Obs(2),radius_Obs);
    X3 = [P_Robot(1),P_Robot(2);x_circle(1),y_circle(1)];
    X4 = [P_Robot(1),P_Robot(2);x_circle(2),y_circle(2)];
    d3 = pdist(X3,'euclidean');
    d4 = pdist(X4,'euclidean');
    if d3 < d4
        goalPoint = [x_circle(1) y_circle(1)];
    else 
        goalPoint = [x_circle(2) y_circle(2)];
end  
end
