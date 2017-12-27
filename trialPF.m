function [phi] = trialPF(P_Robot,G_Robot,P_Obs,r_or)
%       KG = 1;
%       %r_or = r_o + r_r
%       
%       syms x y
%       % Goal - Obstacle
%       rG_Obs = sqrt((P_Obs(1)-G_Robot(1))^2 + (P_Obs(2)-G_Robot(2))^2);
%       b = r_or / (r_or + rG_Obs);
%       
%       % Goal Point - Robot
%       rG = sqrt((G_Robot(1) - x)^2 + (G_Robot(2) - y)^2);
%   
%       % Obstacle - Robot
%       rObs = sqrt((P_Obs(1) - x)^2 + (G_Robot(2) - y)^2);
%       
%        
%       U = (b * log(1/rObs)) - log(1/rG);
%       E = gradient(U, [x, y]);
%       
%       Ex1 = subs(E(1), {x,y}, {P_Robot(1),P_Robot(2)});
%       Ey1 = subs(E(2), {x,y}, {P_Robot(1),P_Robot(2)});
%       Ex1 = single(Ex1);
%       Ey1 = single(Ey1);
%       E1 = Ey1/Ex1
%       phi1 = atand(Ey1/Ex1)
%Calculate  angle phi
      P_o1 = P_Obs(1);
      P_o2 = P_Obs(2);
      G_r1 = G_Robot(1);
      G_r2 = G_Robot(2);
      x = P_Robot(1);
      y = P_Robot(2);
 Ex = (-1)*((r_or*(2*P_o1 - 2*x))/(2*((G_r2 - y)^2 + (P_o1 - x)^2)*...
     (r_or + ((G_r1 - P_o1)^2 + (G_r2 - P_o2)^2)^(1/2))) -...
     (2*G_r1 - 2*x)/(2*((G_r1 - x)^2 + (G_r2 - y)^2)));
 
 Ey = (-1)*((r_or*(2*G_r2 - 2*y))/(2*((G_r2 - y)^2 + (P_o1 - x)^2)*...
     (r_or + ((G_r1 - P_o1)^2 + (G_r2 - P_o2)^2)^(1/2))) -...
     (2*G_r2 - 2*y)/(2*((G_r1 - x)^2 + (G_r2 - y)^2)));
      
 phi = atand(Ey/Ex);

end
