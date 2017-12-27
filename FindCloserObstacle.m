function [min_index] = FindCloserObstacle(P_Robot,P_Obs_all,num_obstacles)
 min_distance = sqrt((P_Robot(1)-P_Obs_all(1,1))^2 + (P_Robot(2)-P_Obs_all(1,2))^2);
 min_index = 1;
 for i = 2:1:num_obstacles
     distance = sqrt((P_Robot(1)-P_Obs_all(i,1))^2 + (P_Robot(2)-P_Obs_all(i,2))^2);
     if min_distance > distance 
         min_distance = distance;
         min_index = i;
     end
 end
end

