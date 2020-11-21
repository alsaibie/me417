# Code Snippet 2

a = 1; b = 1; c = 1;
# Initial Conditions
x0 = [0; 0];
 
# funtion for xdot vector - using an anonymous function. You can also
# define a separate function in a separate file or at the end of the
# script
 
function dxdt(x,u,t)
    global a, b, c;
    dx = [0.0; 0.0]
    dx[1] = x[2];
    dx[2] = (u - b * x[2] - c * x[1]) / a;
    return dx
end

# Integration Time step
dt = 0.1;
#  Time vector
t = 0:dt:20;
 
# Initialize x
x_sim = zeros(2,length(t)); # Empty 2xn array
ui = zeros(1,length(t)); # Empty 1xn vector
x_sim[:,1] = x0;
 
for ix = 1:length(t)-1
    ui[1,ix+1] = sin(t[ix]); # Calculate the next input value
    xdot = dxdt(x_sim[:,ix], ui[1,ix+1], t[ix]); # Grab the derivative vector
    x_sim[:, ix+1] = x_sim[:, ix] + xdot * dt; # Integrate x
end
 
# And plot
plot(t,ui[1,:])
p = plot(t,x_sim[1,:], lw=3, label = "\$x\$")
plot!(p, t,x_sim[2,:], lw=3, label = "\$\\dot{x}\$")
plot!(p, framestyle=:origin, xguide="Time (s)", yguide="Response", linecolor=colors, title="", background_color=:transparent, foreground_color=:black, size=(800, 400); grid=true, minorgrid=true) #hide
savefig(joinpath(@__DIR__, "output", "snp2_plot.svg")) #hide
