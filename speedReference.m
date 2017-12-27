function  [u]= speedReference(n,index,p,h,P_Robot,G_Robot,dmax,P_r_follower,umax)
% u for every robot
% SPEED REFERENCE
% n = number of robots
d = sqrt((P_Robot(1)-G_Robot(1))^2 + (P_Robot(2)-G_Robot(2))^2);
s_bar = 1 - sigmoid(p, h , d);
if index ~= n
    distance = sqrt((P_Robot(1)-P_r_follower(1))^2 + (P_Robot(2)-P_r_follower(2))^2);
    if distance < dmax
        u = s_bar * (1 - sigmoid(p, h , dmax - distance));
    else
        u = 0;
    end   
else
    u = s_bar; 
end
end
