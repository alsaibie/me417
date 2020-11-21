# Exercise 4

# Initial Conditions
x0 = [0; 0]; 
function dxdt(x,u,t)
    I = 3.5; d = 2;
    dx = [0.0; 0.0]
    dx[1] = x[2];
    dx[2] = u*d/I;
    return dx
end
dt = 0.01;
#  Time vector
t = 0:dt:20;
 
# Initialize x
x_sim = zeros(2,length(t)); # Empty 2xn array

function us(t)
    if t > 2 && t <= 4
        return 2
    elseif t > 4 && t <= 6 
        return -2
    else
        return 0
    end
end

x_sim[:,1] = x0;
 
for ix = 1:length(t)-1
    xdot = dxdt(x_sim[:,ix], us(t[ix]), t[ix]); # Grab the derivative vector
    x_sim[:, ix+1] = x_sim[:, ix] + xdot * dt; # Integrate x
end
 
# And plot
p = plot(t,us.(t), lw=3, label = "M")
plot!(t,x_sim[1,:], lw=3, label = "\$θ\$")
plot!(p, t,x_sim[2,:], lw=3, label = "\$\\dot{θ}\$")
plot!(p, framestyle=:origin, xguide="Time (s)", yguide="Response", linecolor=colors, title="", background_color=:transparent, foreground_color=:black, size=(800, 400); grid=true, minorgrid=true) #hide
savefig(joinpath(@__DIR__, "output", "ex4_plot.svg")) #hide
