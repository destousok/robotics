function [max_val] = Umax(F_plus,a_astro,G1,G2)
    max_val = max(F_plus / (a_astro * G1),...
    4 * F_plus/(3 * G1-a_astro * G2))
end