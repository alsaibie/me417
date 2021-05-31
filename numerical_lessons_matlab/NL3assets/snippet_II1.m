s = tf('s');
Kt = 1; Kb = 3; Ra = 10; La = 5; J = 10; Dm = 0.5;
Gv = Kt * s / ((J * s^2 + Dm * s) * (Ra + La * s) + Kt * Kb *s)