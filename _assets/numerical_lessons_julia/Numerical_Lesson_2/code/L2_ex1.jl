# Exercise 1
using Plots;
pyplot();
using LaTeXStrings #hide

using ControlSystems
using SymPy

(s, K, b, Ih, Im) = symbols("s K b I_h I_h")

K_v = 0.07; b_v = 0.1; Ih_v = 0.04; Im_v = 0.1;
A = [Im*s^2+b*s+K -b*s-K; -b*s-K Ih*s^2+b*s+K];
B = [1; 0];

Gx = inv(A)*B
G1s = (s^2*Gx[1])(K=>K_v, Im=>Im_v, Ih=>Ih_v, b=>b_v);
G2s = (s^2*Gx[2])(K=>K_v, Im=>Im_v, Ih=>Ih_v, b=>b_v);

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

G1  = Sym_to_TF(G1s)
@show G1s
G2  = Sym_to_TF(G2s)
@show G2s

t = 0:0.1:10;
p = stepplot(G2, t, lw=3)
plot!(p, framestyle=:origin, xguide="Time (s)", yguide="A", linecolor=colors, title="", background_color=:transparent, foreground_color=:black, size=(800, 400); grid=true, minorgrid=true) #hide
savefig(joinpath(@__DIR__, "output", "ex1_plot.svg")) #hide