% controlSystemDesigner('rlocus', Gvcl/s)
Gc_p = 18;
Gpcl = feedback(Gc_p*Gvcl/s,1);
figure('Position', [0 0 900 450])
step(Gpcl)