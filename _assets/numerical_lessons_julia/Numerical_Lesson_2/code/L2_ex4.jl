# Exercise 4
using CSV, DataFrames
data = CSV.read("../data/Part_I_Problem_B_Data_Step_Response.csv", DataFrame);
t = data[!,1];
x = data[!,2];

r = 10
tau = t[findfirst(a->a>=0.6*x[end], x)[1]]; 
a = 1 / tau
K= a * x[end] / r;
s = tf("s")
G = K / (s+a)
p = plot(t, x, lw=3,  label="Experiment")
stepplot!(p, r*G, t, lw=3, label="Estimate")
plot!(p, framestyle=:origin, xguide="Time (s)", yguide="Response", linecolor=colors, title="", background_color=:transparent, foreground_color=:black, size=(800, 400); grid=true, minorgrid=true) #hide
savefig(joinpath(@__DIR__, "output", "ex4_plot1.svg")) #hide


data = CSV.read("../data/Part_I_Problem_B_Data_Torque_Speed_10V.csv", DataFrame);
w = data[!,1];
T = data[!,2];
e = 10;
Ra = 8;
wnoload = w[end]
Tstall = T[1]
Kt = rd(Ra * Tstall / e)
Kb = rd(e / wnoload)
Jm = rd(Kt / (Ra * K))
Dm = rd(a*Jm - Kt*Kb/Ra)
println("Motor Parameters:\nR_a=$Ra, K_t=$Kt, K_b=$Kb, J_m=$Jm, D_m=$Dm")
