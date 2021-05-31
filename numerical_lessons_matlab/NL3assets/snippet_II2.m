% controlSystemDesigner('rlocus', Gv)
% 1819.2 (s+0.05) (s+4.671)
%  -------------------------
%       s
Gc = 1819 * (s+0.05) * (s + 4.671) / s;
Gvcl = feedback(Gc*Gv,1);
figure()
step(Gvcl)