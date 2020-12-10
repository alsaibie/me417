# Exercise 1
using Plots;
pyplot();
using LaTeXStrings #hide

using ControlSystems
using SymPy

(s, K, b, I_h, I_m) = symbols("s K b I_h I_h")

K_ = 0.07; b_ = 0.1; I_h_ = 0.04; I_m_ = 0.1;
A = [I_m*s^2+b*s+K -b*s-K; -b*s-K I_h*s^2+b*s+K];
B = [1; 0];

Gx = inv(A)*B
G1s = subs(s^2*Gx[1], zip((K, I_m, I_h, b), (K_, I_m_, I_h_, b_))...);
G2s = subs(s^2*Gx[2], zip((K, I_m, I_h, b), (K_, I_m_, I_h_, b_))...);

# Takes a symbolic returns a ControlSystems::TransferFunction
function Sym_to_TF(G::Sym)::TransferFunction 
    s = symbols("s")
    num_s, den_s = simplify(G).as_numer_denom() # Grab num /den polynomials
    num = N(expand(num_s).as_poly(s).all_coeffs()) # Extract coefficient
    den = N(expand(den_s).as_poly(s).all_coeffs()) 
    num = round.(num; sigdigits = 3) # Round
    den = round.(den; sigdigits = 3)
    return tf(num, den) # Convert to ControlSystems::TransferFunction
end

@show G1  = Sym_to_TF(G1s)
@show G2  = Sym_to_TF(G2s)

t = 0:0.1:10;
p = stepplot(G2, t, lw=3)

plot!(p, framestyle=:origin, xguide="Time (s)", yguide="A", linecolor=colors, title="", background_color=:transparent, foreground_color=:black, size=(800, 400); grid=true, minorgrid=true) #hide

savefig(joinpath(@__DIR__, "output", "ex1_plot.svg")) #hide