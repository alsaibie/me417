# Exercise 2
function dxdt(x,u,t)
    K = 0.07; b = 0.1; I_h = 0.04; I_m = 0.1;
    dx = [0.0; 0.0; 0.0; 0.0];
    dx[1] = x[2];
    dx[2] = (u  - K * x[1] - b * x[2] + K * x[3] + b * x[4]) / I_m;
    dx[3] = x[4];
    dx[4] = (- K * x[3] - b * x[4] + K * x[1] + b * x[2]) / I_h; 
    return dx
end

# Integration Time step
dt = 0.1;
#  Time vector
t = 0:dt:20;
    
# Initialize x
x_sim = zeros(6,length(t)); # Empty 2xn array
ui = zeros(1,length(t)); # Empty 1xn vector

x0 = [0, 0, 0, 0, 0, 0]
x_sim[:,1] = x0;
ui = ones(1, length(t))
for ix = 1:length(t)
    xdot = dxdt(x_sim[1:4,ix], ui[1,ix], t[ix]); # Grab the derivative vector
    x_sim[5:6, ix] = [xdot[2]; xdot[4]];
    if ix < length(t)
        x_sim[1:4, ix+1] = x_sim[1:4, ix] + xdot * dt; # Integrate x
    end
end
 
# And plot
plot(t,ui[1,:])
p = plot(t,x_sim[5,:], lw=3, label = "\$\\ddot{\\theta}_m\$")
plot!(p, t,x_sim[6,:], lw=3, label = "\$\\ddot{\\theta}_h\$")
plot!(p, framestyle=:origin, xguide="Time (s)", yguide="Response", linecolor=colors, title="", background_color=:transparent, foreground_color=:black, size=(800, 400); grid=true, minorgrid=true) #hide
savefig(joinpath(@__DIR__, "output", "ex2_plot.svg")) #hide