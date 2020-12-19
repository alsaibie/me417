# Exercise 8

Kb = 1; Kt = 2; Ra = 8; J = 2; D = 0.6; La = 5;
s = tf("s");
G_torque = Kt * ( J * s^2 + D * s) / ( (J * s^2 + D * s) * (Ra + La *  s) + Kt * Kb * s)

# Gains
Kp = 110; Ki = 100; Kd = 0;
# Reference Input
r = 2;

t=0:.01:5;

Gc = Kp + Ki/s + Kd * s;
Gcl_torque = feedback(Gc*G_torque,1);

p = stepplot(r*Gcl_speed,t, lw=3, label="\$τ\$")
plot!(p, framestyle=:origin, xguide="Time (s)", yguide="Response", linecolor=colors, title="", background_color=:transparent, foreground_color=:black, size=(800, 400); grid=true, minorgrid=true) #hide
savefig(joinpath(@__DIR__, "output", "ex8_plot.svg")) #hide
stepinfo(r*Gcl_speed)