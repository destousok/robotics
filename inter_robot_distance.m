function [d] = inter_robot_distance(d_o,th,u_robot) 
%String stability

d = d_o + th * u_robot(1);

end
