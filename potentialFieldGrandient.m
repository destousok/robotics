%this script calculate the symbolic function of gradient of the artificial
%potential field.
      
syms x y P_o1 P_o2 G_r1 G_r2 r_or

% Goal - Obstacle
rG_Obs = sqrt((P_o1-G_r1)^2 + (P_o2-G_r2)^2);
b = r_or / (r_or + rG_Obs);

% Goal Point - Robot
rG = sqrt((G_r1 - x)^2 + (G_r2 - y)^2);

% Obstacle - Robot
rObs = sqrt((P_o1 - x)^2 + (G_r2 - y)^2);
U = (b * log(1/rObs)) - log(1/rG);
E = gradient(U, [x, y])



