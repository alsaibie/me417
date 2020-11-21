# Exercise 2
using OrdinaryDiffEq
 
# Initial Conditions
x0 = [0; 0];
 
# funtion for xdot vector. You can also
# define a separate function in a separate file or at the end of the
# script
function dxdt(dx,x,p,t)
    M = 7; K = 2.8; f = 0.2;
    dx[1] = x[2];
    dx[2] = (sin(2*pi*f*t) - K * x[1] - 0.1 * x[2] * x[2]) / M;
end
 
# Time span
t = (0,40)
prob = ODEProblem(dxdt,x0,t)
# Simulate using ODE Solver
sol = solve(prob, Tsit5())
 
# And plot
p = plot(sol, lw=3, label = ["\$x\$" "\$\\dot{x}\$"])
plot!(p, framestyle=:origin, xguide="Time (s)", yguide="Response", linecolor=colors, title="", background_color=:transparent, foreground_color=:black, size=(800, 400); grid=true, minorgrid=true) #hide
savefig(joinpath(@__DIR__, "output", "ex2_plot.svg")) #hide
