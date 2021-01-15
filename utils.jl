function hfun_bar(vname)
  val = Meta.parse(vname[1])
  return round(sqrt(val), digits=2)
end

function hfun_m1fill(vname)
  var = vname[1]
  return pagevar("index", var)
end

function lx_baz(com, _)
  # keep this first line
  brace_content = Franklin.content(com.braces[1]) # input string
  # do whatever you want here
  return uppercase(brace_content)
end

function hfun_inserthtml(args)::String
    # check params
  if length(args) != 1
      throw(HTMLFunctionError("I found a {{inserthtml ...}} with more than one parameter. Verify."))
  end
  # apply
  repl   = ""
  fpath  = args[1]
  if isfile(fpath)
      repl = Franklin.convert_html(read(fpath, String))
  else
      Franklin.hfun_misc_warn(:insert, """
          Couldn't find the file '$fpath' to resolve the insertion.
          """)
  end
  return repl
end