# Exercise 3
rd(x) = round(x, sigdigits=3) 

function stepinfo(t::Array, x::Array)
    xmax = maximum(x)   
    # Max compared to final value
    OS = rd(xmax/x[end] - 1)
    # From index of max value
    Tp = rd(t[findfirst(a->a==xmax, x)[1]])
    # From index of first time to reach 10% and 90% of final value
    Tr = rd(t[findfirst(t->t>=0.9*x[end], x)[1]] - t[findfirst(t->t>=0.1*x[end], x)[1]])
    # From last index when value is ≥ 2% of final value 
    Ts = rd(t[findlast(t->abs(t-x[end])>=0.02, x)[1]+1])

    println("Rise Time: $Tr s")
    println("Percent Overshoot: $(OS*100)%")
    println("Peak Time: $Tp s")
    println("Settling Time: $Ts s")
    return Tr, OS, Tp, Ts
end

function stepinfo(G::TransferFunction)
    x, t, _  = step(G)
    return stepinfo(t[:,1],x[:,1])
end

# Test 
Ts = 2.5;
Tp = 1;
ωd = π / Tp;
σ = 4 / Ts;
ζ = cos(atan(ωd, σ))
ωn = σ/ζ
OS = exp(-ζ*π/sqrt(1-ζ^2))
s = tf("s")
G = ωn^2 / (s^2+2*ζ*ωn*s+ωn^2)

println("Analytical:")
println("Percent Overshoot: $(rd(OS*100))%")
println("Peak Time: $Tp s")
println("Settling Time: $Ts s")
println("\nMeasured:")

stepinfo(G)