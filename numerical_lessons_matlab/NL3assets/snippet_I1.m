clear all;
I_m_=0.1; I_h_=0.04; K_=0.07; b_=0.1;
syms K b I_m I_h Mm s

A = [I_m*s^2+b*s+K, -b*s-K;
    -b*s-K, I_h*s^2+b*s+K];
B=[1;0];
G_symbolic = inv(A)*B
Gm_s = subs(G_symbolic(1), {K I_m I_h b}, {K_ I_m_ I_h_ b_});
Gh_s = subs(G_symbolic(2), {K I_m I_h b}, {K_ I_m_ I_h_ b_});
[num,den] = numden(Gm_s);
num = sym2poly(num);
den = sym2poly(den);
Gm = tf(num,den)
[num,den] = numden(Gh_s);
num = sym2poly(num);
den = sym2poly(den);
Gh = tf(num,den)