function [phi] = PotentialField(P_r,G_r,P_o,r_or)
      KG = 1;
      %r_or = r_o + r_r
      % Goal - Obstacle
      rG_Obs = sqrt((P_o(1)-G_r(1))^2 + (P_o(2)-G_r(2))^2);
      b = r_or / (r_or + rG_Obs);
      
      % Goal Point - Robot
      rG = sqrt((G_r(1) - P_r(1))^2 + (G_r(2) - P_r(2))^2);
      EGx = KG*(G_r(1) - P_r(1))/rG^2;
      EGy = KG*(G_r(2) - P_r(2))/rG^2;
      
      % Obstacle - Robot
      rObs = sqrt((P_o(1) - P_r(1))^2 + (P_o(2) - P_r(2))^2);
      EObsx = -b*(P_o(1) - P_r(1))/rObs^2;
      EObsy = -b*(P_o(2) - P_r(2))/rObs^2;
      
      Ex = (EGx + EObsx);
      Ey = (EGy + EObsy);
      
      phi = atand(Ey/Ex);
end
