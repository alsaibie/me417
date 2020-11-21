# Code Snippet 1

using OrdinaryDiffEq
# System Parameters
a = 2; b = 1; c = 2; f = 5;
 
# Initial Conditions
x0 = [0; 0];
 
# funtion for xdot vector. You can also
# define a separate function in a separate file or at the end of the
# script
function dxdt(dx,x,p,t)
    global a, b, c, f;
    dx[1] = x[2];
    dx[2] = (f - b * x[2] - c * x[1]) / a;
end
 
# Time span
t = (0,40)
 
prob = ODEProblem(dxdt,x0,t)

# Simulate using ODE Solver
sol = solve(prob, Tsit5())
 
# And plot
p = plot(sol, lw=3, label = ["\$x\$" "\$\\dot{x}\$"])
plot!(p, framestyle=:origin, xguide="Time (s)", yguide="Response", linecolor=colors, title="", background_color=:transparent, foreground_color=:black, size=(800, 400); grid=true, minorgrid=true) #hide
savefig(joinpath(@__DIR__, "output", "snp1_plot.svg")) #hide
