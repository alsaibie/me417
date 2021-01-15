cd(dirname(@__FILE__)) # hide
# # WGLMakie.activate!()
# using ControlSystems
# # using MakieLayout

b = board("brd", xlim=[-2,2], ylim=[-2,2])
b ++ point(0, 0, name="hello")
print("""~~~$(JSXGraph.standalone(b, preamble=false))~~~""") # hide


