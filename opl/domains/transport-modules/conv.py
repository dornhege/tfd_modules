from __future__ import with_statement
import sys

def multCap(line):
   line = line.strip()
   index = line.rfind(" ")
   if index == -1:
      return line
   start = line[:index + 1]
   end = line[index:-1].strip()
   cap = int(end) * 2.5
   print "\"", start, "\" - \"", end, "\""
   return "  " + start + str(cap) + ")\n"

print sys.argv[1:]
for prob in sys.argv[1:]:
   out = open(prob + ".new", "w")
   with open(prob, "r") as f:
      for line in f:
         if line.find("capacity") is not -1:
            line = multCap(line)
         print >> out, line,
   out.close()

