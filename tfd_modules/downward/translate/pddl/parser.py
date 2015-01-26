__all__ = ["ParseError", "parse_nested_list"]

class ParseError(Exception):
  pass

# Basic functions for parsing PDDL (Lisp) files.
def parse_nested_list(input_file):
  tokens = tokenize(input_file)
  next_token = tokens.next()
  if next_token != "(":
    raise ParseError("Expected '(', got %s." % next_token)
  result = list(parse_list_aux(tokens))
  for tok in tokens:  # Check that generator is exhausted.
    raise ParseError("Unexpected token: %s." % tok)
  return result
  
def tokenize(input):
  # Here comes the ugly hack: :moduleoptions section must not be decapitalized!
  # therefore need to make sure, when the section ends.
  inModOpts = False
  modOptsOpenParen = 0
  for line in input:
    line = line.split(";", 1)[0]  # Strip comments.
    line = line.replace("(", " ( ").replace(")", " ) ").replace("?", " ?")
    # added to tokenize module calls
    line = line.replace("[", " [ ").replace("]", " ] ")
    for token in line.split():
      if token == ":moduleoptions" or token == ":moduleexitoptions":
        inModOpts = True
        modOptsOpenParen = 1  # the one right before :moduleoptions
      if token == "(" and inModOpts:
        modOptsOpenParen += 1
      if token == ")" and inModOpts:
        modOptsOpenParen -= 1
      if inModOpts and modOptsOpenParen <= 0:
        inModOpts = False
        
      if token.find("@") != -1 or inModOpts:   # keep library calls correct
        yield token
      else:
        yield token.lower()

def parse_list_aux(tokenstream):
  # Leading "(" has already been swallowed.
  while True:
    try:
      token = tokenstream.next()
    except StopIteration:
      raise ParseError()
    if token == ")":
      return
    elif token == "(":
      yield list(parse_list_aux(tokenstream))
    else:
      yield token

