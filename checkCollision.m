function  [r_security]= checkCollision(P_Robot,u_Robot,radius_robot,P_Obs,u_Obs,radius_Obs,gama,theta)
p = 1;
h = 2;

% Collision-cone approach for mobile obstacles
d = sqrt((P_Robot(1)-P_Obs(1))^2 + (P_Robot(2)-P_Obs(2))^2);
% Relative speed
U_r = (u_Obs(1) * cos(P_Obs(3)-theta)) - u_Robot(1) * cos(P_Robot(3)-theta)
U_theta =  (u_Obs(1) * sin(P_Obs(3)-theta)) - u_Robot(1) * sin(P_Robot(3)-theta)
%parameter h
hta = abs(U_theta)/(sqrt(U_theta^2 + U_r^2));
%extra radius if objects come closer.
r_double = min(radius_Obs * gama * sigmoid(p,h,hta), d-radius_Obs-radius_robot);

if U_r < 0
    %if the objects get closer add a extra radius in security radius
    r_security = radius_Obs + radius_robot + r_double;
else
    %if the objects move away keep stadar security radius
    r_security = radius_Obs + radius_robot;
end
end
