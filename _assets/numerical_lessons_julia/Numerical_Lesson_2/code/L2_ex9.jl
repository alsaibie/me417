# Exercise 9
# Funtion for xdot vector - using an anonymous function.
function dxdt(x,u,t)
    K_v = 0.07; b_v = 0.1; Ih_v = 0.04; Im_v = 0.1;
    return [x[2]; 
    (u - K_v * x[1] - b_v * x[2] + K_v * x[3] + b_v * x[4]) / Im_v; 
    x[4];
    (-K_v * x[3] - b_v * x[4] + K_v * x[1] + b_v * x[2]) / Ih_v 
    ];
end  

# Initialize simulation parameters
x0 = [0; 0; 0; 0]
dt = 0.001;
t_sim = 0:dt:2;
x_sim = zeros(4,length(t_sim)); # Empty 2xn array
u = zeros(length(t_sim)); # Empty 1xn vector
e = zeros(length(t_sim));
e_int = zeros(length(t_sim));
e_dot = zeros(length(t_sim)); 

# Reference input
r = 1; 
# Compute First Error
e[1] = 0;
# Compute First Control Output
u[1] = 0; 

x_sim[:,1] = x0;
Kp = 10; Ki = 8; Kd = 1;
 
for idx = 1:length(t_sim)-1
    # Grab the derivative vector
    xdot = dxdt(x_sim[:,idx], u[idx], t_sim[idx]);
    # Integrate 
    x_sim[:, idx+1] = x_sim[:, idx] + xdot * dt;

    # Calculate Errors
    e[idx+1] = r - x_sim[1, idx+1];
    e_int[idx+1] = e_int[idx] + e[idx]*dt;
    e_dot[idx+1] = (e[idx+1] - e[idx]) / dt;

    # PID Control Law
    u[idx+1] =  Kp*e[idx+1] + Ki*e_int[idx+1] + Kd*e_dot[idx+1];

end
 
# And plot
p = plot(t_sim,0.01*u, lw=3, label = "0.01u")
plot!(t_sim,x_sim[1,:], lw=3, label = "\$x_1\$")
plot!(p, t_sim,x_sim[2,:], lw=3, label = "\$x_2\$")
plot!(p, framestyle=:origin, xguide="Time (s)", yguide="Response", linecolor=colors, title="", background_color=:transparent, foreground_color=:black, size=(800, 400); grid=true, minorgrid=true) #hide
savefig(joinpath(@__DIR__, "output", "ex9_plot.svg")) #hide

stepinfo(t_sim[:,1], x_sim[1,:])