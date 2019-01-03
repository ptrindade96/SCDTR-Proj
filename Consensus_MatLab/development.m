
syms L1 o1 positive
syms c1 rho Q1 Q2 k1 k2 positive
syms y1 y2 d_av1 d_av2 d1 d2 real

ki = [k1;k2];
y = [y1;y2];
ci = [c1;0];
d_av = [d_av1;d_av2];
Qi = [Q1,0;0,0];
X = Qi/2 + rho/2*eye(2);
B = 2*X;
di = [d1;d2];



z = ci + y - rho*d_av;
d_u = -inv(B) * z


Ai = -ki';
u = o1-L1;
d_1 = -inv(B)*z + inv(B)*Ai'*inv(Ai*inv(B)*Ai')*(Ai*inv(B)*z+u);
d_1 = simplify(d_1)


Ai = -[1;0]';
u = 0;
d_2 = -inv(B)*z + inv(B)*Ai'*inv(Ai*inv(B)*Ai')*(Ai*inv(B)*z+u);
d_2 = simplify(d_2)

Ai = [1;0]';
u = 255;
d_3 = -inv(B)*z + inv(B)*Ai'*inv(Ai*inv(B)*Ai')*(Ai*inv(B)*z+u);
d_3 = simplify(d_3)

Ai = [-ki';-[1;0]'];
u = [o1-L1;0];
d_4 = -inv(B)*z + inv(B)*Ai'*inv(Ai*inv(B)*Ai')*(Ai*inv(B)*z+u);
d_4 = simplify(d_4)


Ai = [-ki';[1;0]'];
u = [o1-L1;255];
d_5 = -inv(B)*z + inv(B)*Ai'*inv(Ai*inv(B)*Ai')*(Ai*inv(B)*z+u);
d_5 = simplify(d_5)






