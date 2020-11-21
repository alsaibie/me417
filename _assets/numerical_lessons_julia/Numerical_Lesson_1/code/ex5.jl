# Exercise 5
function dxdt(x,u,t)
    mr = 3; l = .19; g = 9.81;
    I = 4/3*mr*l^2; 
    dx = [0.0; 0.0]
    dx[1] = x[2];
    dx[2] = (u - mr * g * l *sin(x[1])) / ( I + mr * l^2);
    return dx
end
 
# Initialize x
x0 = [0; 0]
dt = 0.01;
t = 0:dt:40;
x_sim = zeros(2,length(t)); # Empty 2xn array
x_sim[:,1] = x0;
 
function up(t)
    if t < 3
        return 5
    else
        return 0
    end
end

for ix = 1:length(t)-1
    xdot = dxdt(x_sim[:,ix], up(t[ix]), t[ix]); # Grab the derivative vector
    x_sim[:, ix+1] = x_sim[:, ix] + xdot * dt; # Integrate x
end
 
# And plot
p = plot(t,up.(t), lw=3, label = "M")
plot!(t,x_sim[1,:], lw=3, label = "\$θ\$")
plot!(p, t,x_sim[2,:], lw=3, label = "\$\\dot{θ}\$")
plot!(p, framestyle=:origin, xguide="Time (s)", yguide="Response", linecolor=colors, title="", background_color=:transparent, foreground_color=:black, size=(800, 400); grid=true, minorgrid=true) #hide
savefig(joinpath(@__DIR__, "output", "ex5_plot.svg")) #hide
