# Exercise 1

using ControlSystems; 
using Plots;
pyplot();
using LaTeXStrings #hide

M = 7;
K = 2.8;
fv = [1.5 5 20];
wn = sqrt(K/M);
zetas = fv ./ (M * 2 * wn);    
G = [ tf([wn^2], [1, 2*zeta*wn, wn^2]) for zeta in zetas ];
@show G
t = 0:0.01:40;

p = stepplot(G, t, lw=3,  label=[L"F_v="*"$(fv[1])" L"F_v="*"$(fv[2])" L"F_v="*"$(fv[3])"]);

plot!(p, framestyle=:origin, xguide="Time (s)", yguide="A", linecolor=colors, title="", background_color=:transparent, foreground_color=:black, size=(800, 400); grid=true, minorgrid=true) #hide

savefig(joinpath(@__DIR__, "output", "ex1_plot.svg")) #hide
