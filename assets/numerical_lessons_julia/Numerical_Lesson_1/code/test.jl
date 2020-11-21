using Plots

f(x) = x^2
x = 0:10
plot(x, f.(x))
savefig(joinpath(@__DIR__, "output", "test.svg"))
