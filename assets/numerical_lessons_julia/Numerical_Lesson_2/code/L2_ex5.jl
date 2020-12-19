# Exercise 5
s = tf("s")
Gp = 2 / (s^2+8*s+25);
Kp = 200; Ki = 150; Kd = 1;
Gc = (Kd*s^2 + Kp*s + Ki ) / s;

t = 0:0.01:10 
Gcl = feedback(Gc*Gp, 1);
p = stepplot(Gcl, t, lw=3);
plot!(p, framestyle=:origin, xguide="Time (s)", yguide="Response", linecolor=colors, title="", background_color=:transparent, foreground_color=:black, size=(800, 400); grid=true, minorgrid=true) #hide
savefig(joinpath(@__DIR__, "output", "ex5_plot.svg")) #hide