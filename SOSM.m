function [  ] = SOSM(t,m,J,Nt,Nt_der,Nr,Nr_der,ud,ud_der,ud_2der,ud_2der_plus,wd_2der_plus,wd_der_plus,wd,wd_der,wd_2der)
% m = [m m+ m-]
% J = [J J+ J-]
% Nt = [|Nt| f |Nt+|]
% Nt_der = [|Nt_der| f |Nt_der+|]
% Nr = [|Nr| f |Nr+|]
% Nr_der = [|Nr_der| f |Nr_der+|]
% m_bar = m_minus
% Dm = delta_m
% J_bar = J_minus
% DJ = delta_J
c1 = 0.5;


m_bar = (m(2) + m(3))/2;
Dm_plus = m_bar - m(2);
F1 = - Nt_der/m(1) - ((Dm_plus * ud_2der)/(m_bar + Dm_plus));
g1 = 1/m(1);

J_bar = (J(2) + J(3))/2;
DJ_plus =  J_bar - J(2);
F2 = - c1*Nr/J(1) - Nr_der/J(1) - (DJ_plus * c1 * wd_der)/(J_bar + DJ_plus) - ((DJ_plus * wd_2der)/(J_bar + DJ_plus));
g2 = 1/J(1);

G11 = 1/m(2);
G21 = 1/m(3);
G12 = 1/J(2);
G22 = 1/J(3);
F1_plus = Nt_der(3)/m(3) + Dm_plus * ud_2der_plus;
F2_plus = c1 * Nr(3)/J(3) + Nr_der(3)/J(3) + (DJ_plus * c1 * wd_der_plus)/(J_bar + DJ_plus) - ((DJ_plus * wd_2der_plus)/(J_bar + DJ_plus));
 
%find a1 
if 3*(G11/G21)<1
    a1_astro = 2*(G11/G21);
else
    a1_astro = 0.5;
end

%find a2
if 3*(G12/G22)<1
    a2_astro = 2.5 * (G12/G22);
else
    a2_astro = 0.9;
end

k = 1.2;
% na ta bgalo ap ekso den exei noima na ta ksanaupologizo kathe fora
Umax1 = k * Umax(F1_plus,a1_astro,G11,G21);
Umax2 = k * Umax(F2_plus,a2_astro,G12,G22);

if t ~= 0
    if (z11 - 0.5 * zmax1)*(zmaz1 - z11) > 0
        a1 = a1_astro;
    else
        a1 = 1;
    end
else
    zmax1 = z11
    zmax2 = z12
end



%z1_der = [0 1;0 0] * [z1(1); z1(2)] + [0;1] * F1 + [0;g1] * v1;



end

