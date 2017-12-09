function [ output_args ] = SOSM(m,J,Nt,Nt_der,Nr,Nr_der,ud,ud_1_der,ud_2_der,wd,wd_1_der,wd_2_der)
% m = [m m+ m-]
% J = [J J+ J-]
% Nt = [|Nt| f |Nt+|]
% Nt_der = [|Nt_der| f |Nt_der+|]
% Nr = [|Nr| f |Nr+|]
% Nr_der = [|Nr_der| f |Nr_der+|]
% m_average = m_minus
% Dm = Äm
% J_average = J_minus
% DJ = ÄJ

m_average = (m(2) + m(3))/2;
Dm_plus = m(2) - m_average;
F1 = - Nt_der/m(1) - ((Dm_plus*ud_2_der)/(m_average + Dm_plus));
g1 = 1/m(1);

J_average = (J(2) + J(3))/2;
DJ_plus = J(2) - J_average;
F2 = - c1*Nr/J(1) - Nr_der/J(1) - (DJ_plus*c1*wd_1_der)/(J_average + DJ_plus) - ((DJ_plus*wd_2_der)/(J_average + DJ_plus));
g2 = 1/J(1);

G11 = 1/m(2);
G21 = 1/m(3);
G12 = 1/J(2);
G22 = 1/J(3);
F1_plus = Nt(3)/m(3) + Dm_plus*ud_2_der_plus;
F2_plus = c1*Nr/J(1) - Nr_der/J(1) - (DJ_plus*c1*wd_1_der)/(J_average + DJ_plus) - ((DJ_plus*wd_2_der)/(J_average + DJ_plus));


z1_der = [0 1;0 0]*[z1(1); z1(2)] + [0;1]*F1 + [0;g1]*v1;


z1max = z1(1
end

