# Exercise 1.2
dt = 0.1;
t = 0:dt:100;

function u(t)
    if t > 1.5 && t <= 8
        return 3
    elseif t > 8 && t <= 14 
        return -3
    else
        return 0
    end
end

p = lsimplot(G,u.(t),t, lw=3, label=[L"F_v="*"$(fv[1])" L"F_v="*"$(fv[2])" L"F_v="*"$(fv[3])"]);
plot!(p, framestyle=:origin, xguide="Time (s)", yguide="A", linecolor=colors, title="", background_color=:transparent, foreground_color=:black, size=(800, 400); grid=true, minorgrid=true) #hide
savefig(joinpath(@__DIR__, "output", "ex1_2_plot.svg")) #hide
