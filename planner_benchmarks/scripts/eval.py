#!/usr/bin/env python

# Run evaluation and compute absolute results, i.e. makespans, runtimes...

from __future__ import with_statement

from optparse import OptionParser
import os
import subprocess
import shutil
import collections
import datetime

class Problem(object):
    """ An evaluated problem with a plan and a times debug file. """
    def __init__(self, path, name):
        self.path = path
        self.name = name
        # valid problem has times file + plan.best - if none, no data, if no plan, is there times???
        self.planfile = os.path.join(self.path, "plan." + self.name + ".pddl.best")
        self.timesfile = os.path.join(self.path, "times." + self.name + ".pddl")
        self.makespan, self.runtime = self.parseTimes()
    def dump(self):
        print "Base path: %s Plan: %s, Times: %s" % (self.path, self.planfile, self.timesfile)
    def __repr__(self):
        return self.name
    def write(self, stream):
        """ Write out name + makespan + runtime """
        print >> stream, self.name, self.makespan, self.runtime
    def parseTimes(self):
        with open(self.timesfile) as f:
            lastLine = ""
            for line in f:
                if not line or line.startswith("#"):
                    continue
                lastLine = line
            if not lastLine:    # No data written -> no results/no plan
                yield -1
                yield -1
            else:
                entries = lastLine.split()
                assert len(entries) == 2
                yield (float)(entries[0])
                yield (float)(entries[1])

def make_dir_list(path):
    """ Create list from path where each directory is one list entry """
    front,back = os.path.split(path)
    if not back and not front:
        return []
    if not back:
        return make_dir_list(front)
    if not front:
        return [ back ]
    frontL = make_dir_list(front)
    backL = make_dir_list(back)
    if not frontL:
        return backL
    if not backL:
        return frontL
    theList = []
    theList.extend(frontL)
    theList.extend(backL)
    return theList

def evalDir(path, files):
    # The expected format for problem named X with problem file X.pddl is:
    # plan.X.pddl.best and times.X.pddl
    plannames = []
    for f in files:
        if f.endswith(".pddl.best"):
            assert(f.startswith("plan."))
            assert(len(f) > len("plan.") + len(".pddl.best"))
            name = f[len("plan."): - len(".pddl.best")]
            plannames.append(name)
    plannames.sort(key=str.lower)
    # valid names are those, where a times.X.pddl file exists
    validnames = [name for name in plannames if os.path.exists(os.path.join(path, "times." + name + ".pddl"))]
    problems = []
    for name in validnames:
        problems.append(Problem(path, name))
    
    # create return dictionary from path A/B/C/files -> {A: {B: {C: problems} } }
    theDict = {}
    dirList = make_dir_list(path)

    # seed dict with lowest level containing the problems
    assert len(dirList) > 0
    theDict[dirList[-1]] = problems
    dirList = dirList[:-1]

    # build the dict by pushing the current dict as val in the last entry in dirList
    while len(dirList) > 0:
        newDict = {}
        newDict[dirList[-1]] = theDict
        theDict = newDict
        dirList = dirList[:-1]

    return theDict

def quacks_like_dict(object):
    """Check if object is dict-like"""
    return isinstance(object, collections.Mapping)
    
# From: http://appdelegateinc.com/blog/2011/01/12/merge-deeply-nested-dicts-in-python/
def merge(a, b):
    """Merge two deep dicts non-destructively
    
    Uses a stack to avoid maximum recursion depth exceptions
    
    >>> a = {'a': 1, 'b': {1: 1, 2: 2}, 'd': 6}
    >>> b = {'c': 3, 'b': {2: 7}, 'd': {'z': [1, 2, 3]}}
    >>> c = merge(a, b)
    >>> from pprint import pprint; pprint(c)
    {'a': 1, 'b': {1: 1, 2: 7}, 'c': 3, 'd': {'z': [1, 2, 3]}}
    """
    assert quacks_like_dict(a), quacks_like_dict(b)
    dst = a.copy()
    
    stack = [(dst, b)]
    while stack:
        current_dst, current_src = stack.pop()
        for key in current_src:
            if key not in current_dst:
                current_dst[key] = current_src[key]
            else:
                if quacks_like_dict(current_src[key]) and quacks_like_dict(current_dst[key]) :
                    stack.append((current_dst[key], current_src[key]))
                else:
                    current_dst[key] = current_src[key]
    return dst

def parseResults(results_dir):
    """ Parse dir and all subdirectories to create results. 
        Returns a dictionary of the directory structure relative
        to results_dir with the evaluated problems """
    # List directories, a problem/domain directory contains one or more files with "pddl"

    allResults = {}
    for root, dirs, files in os.walk(results_dir):
        #print "Root: ", root
        #print "Dirs: ", dirs
        #print "Files: ", files
        #print ""
        # If files contains a file with "pddl" in it, it is a result dir
        pddls = [f for f in files if f.lower().find("pddl") >= 0]
        if pddls:
            dirEval = evalDir(root, files)
            #print "DIR EVAL for ", root, " : ", dirEval
            allResults = merge(allResults, dirEval)
            #print "ALL RESULTS: ", allResults
    return allResults

def writeEvalData(evaldict, path):
    for key, val in evaldict.iteritems():
        localpath = os.path.join(path, key)
        if quacks_like_dict(val):   # recurse
            writeEvalData(val, localpath)
        else:   # list of problems
            f = file(os.path.join(localpath, "eval.dat"), "w")
            for i in val:
                i.write(f)
            f.close()

def buildNameIdentifierDict(evaldict, name_entries_dict, curIdent):
    for key, val in evaldict.iteritems():
        if quacks_like_dict(val):   # recurse
            buildNameIdentifierDict(val, name_entries_dict, curIdent + "/" + key)
        else:           # found final dict that contains name -> problems
            if not key in name_entries_dict:
                name_entries_dict[key] = {}
            name_entries_dict[key][curIdent] = val

def writeTex(evaldict, filename):
    """ Write latex table for this dict.
        For dict: {X: {A: problems, B: problems}, Y: {A: problems, C: problems}}
        the output should be a table for A, B, C, where A has cols X/Y, B col X, C col Y
        """
    # First build the name -> {identifier -> entries} dict.
    nameEntriesDict = {}
    buildNameIdentifierDict(evaldict, nameEntriesDict, "")

    f = open(filename, "w")
    print >> f, '\\documentclass{article}'
    print >> f, "\\usepackage{caption}"
    print >> f, '\\begin{document}'

    #print nameEntriesDict
    for key, val in nameEntriesDict.iteritems():
        print "Domain: ", key
        print >> f, '\\begin{table}'
        print >> f, '  \\centering'
        print >> f, '  \\begin{tabular}{|l|c|c|}'
        print >> f, '  \\hline'
        print >> f, '  Problem & ',
        for num, (ident, probs) in enumerate(val.iteritems()):  # is the num ordering somewhat guaranteed?
            print >> f, num,
            if num < len(val) - 1:
                print >> f, "&",
            else:
                print >> f, ""
        print >> f, "\\\\"
        print >> f, '  \\hline'
        problems = set()
        # First collect all problems (some might not have entries)
        for num, (ident, probs) in enumerate(val.iteritems()):  # is the num ordering somewhat guaranteed?
            print "NUM", num
            print "RunIdnet: ", ident
            print "Problems: ", probs
            for i in probs:
                problems.add(repr(i))
        print "PROB SET ", problems
        probList = list(problems)
        probList.sort(key=str.lower)
        for i in probList:
            print "Problem: ", i
            print >> f, i, "    &",
            for num, (ident, probs) in enumerate(val.iteritems()):
                print "NUM", num
                print "RunIdnet: ", ident
                print "Problems: ", probs
                myProb = [j for j in probs if repr(j) == i]
                if myProb:
                    assert len(myProb) == 1
                    print >> f, myProb[0].makespan,
                else:
                    print >> f, "- ",
                if num < len(val) - 1:
                    print >> f, "&",
                else:
                    print >> f, ""
            print >> f, "\\\\"

        print >> f, '  \\hline'
        print >> f, '  \\end{tabular}'
        print >> f, '  \\caption{Domain:', key, #Date: %s, ' % datetime.datetime.now()
        print >> f, "\\\\"
        for num, (ident, probs) in enumerate(val.iteritems()):
            print >> f, str(num) + ": ", ident.replace("_", "\\_") + ",",
            print >> f, "\\\\"
        print >> f, ' }'
        print >> f, '\\end{table}'
        print >> f, ''

    print >> f, '\\end{document}'
    f.flush()
    f.close()

def main():
    parser = OptionParser("usage: %prog PROBLEMS")
    parser.add_option("-r", "--results-dir", dest="results_dir", type="string", action="store")
    opts, args = parser.parse_args()
    print "Results dir: %s" % opts.results_dir

    evalDict = parseResults(opts.results_dir)
    print "FINAL EVAL DICT: ", evalDict

    # write eval data
    writeEvalData(evalDict, ".")

    # create gnuplots/latex tables, etc. - all grouped by domain-name divided by settings/algos
    writeTex(evalDict, "output.tex")

if __name__ == "__main__":
    main()

