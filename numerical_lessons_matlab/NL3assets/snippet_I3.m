t = 0:0.001:5;
Kp = 3.079*0.22489;
Ki = 0;
Kd = 0.22489;
Gpid = pid(Kp, Ki, Kd)
% Gc =  0.01;
figure()
Gcl = feedback(Gpid*Gm, 1)
E = (1-Gcl)
[e,t] = step(E);
step(Gcl, t)
stepinfo(Gcl)