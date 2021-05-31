% controlSystemDesigner('rlocus', Gm)
t = 0:0.001:20;
Kp = .05;
Gc =  Kp;
Gclm = feedback(Gc*Gm, 1)
figure()
step(Gclm, t)
stepinfo(Gclm)
hold on 
step((1-Gclm)*Gc*Gh, t)