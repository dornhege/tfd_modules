#!/usr/bin/env python

# Evaluate results from a run and compute tables, e.g. makespans, runtimes...

from __future__ import with_statement

from optparse import OptionParser
import os
import subprocess
import shutil
import collections
import datetime

import data_tools

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
    def hasSamePlan(self, other):
        """ Compare my plan with the plan of other """
        retcode = subprocess.call(["diff", self.planfile, other.planfile], stdout=subprocess.PIPE)
        return retcode == 0

def evalDir(path, files):
    """ Evaluate the directory in path that contains a number of
        plan...pddl.best files.
        Returns a dict mapping from path to a list
        of Problems """
    # The expected format for problem named X with problem file X.pddl is:
    # plan.X.pddl.best and times.X.pddl
    plannames = []
    for f in files:
        if f.endswith(".pddl.best"):
            # plan.p03.pddl.best -> p03
            assert(f.startswith("plan."))
            assert(len(f) > len("plan.") + len(".pddl.best"))
            name = f[len("plan.") : - len(".pddl.best")]
            plannames.append(name)
    plannames.sort(key=str.lower)
    # valid names are those, where a times.X.pddl file exists
    validnames = [name for name in plannames if os.path.exists(os.path.join(path, "times." + name + ".pddl"))]
    problems = [Problem(path, name) for name in validnames]
 
    # create return dictionary from path A/B/C/files -> {A: {B: {C: problems} } }
    theDict = {}
    dirList = data_tools.make_dir_list(path)

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

def parseResults(eval_dir):
    """ Parse dir and all subdirectories to create results. 
        Returns a dictionary of the directory structure relative
        to eval_dir with the evaluated problems """
    # List directories, a problem/domain directory contains one or more files with "pddl"

    allResults = {}
    for root, dirs, files in os.walk(eval_dir):
        # If files contains a file with "pddl" in it, it is a result dir
        pddls = [f for f in files if f.lower().find("pddl") >= 0]
        if pddls:
            dirEval = evalDir(root, files)
            allResults = data_tools.merge(allResults, dirEval)
    return allResults

def writeEvalData(evaldict, path):
    """ write eval.dat with evaluation for all problems
        in each domain directory """
    for key, val in evaldict.iteritems():
        localpath = os.path.join(path, key)
        if data_tools.behaves_like_dict(val):   # recurse
            writeEvalData(val, localpath)
        else:   # list of problems
            f = file(os.path.join(localpath, "eval.dat"), "w")
            for i in val:
                i.write(f)
            f.close()

def buildNameIdentifierDict(evaldict, name_entries_dict, curIdent):
    """ From a dictionary like {K: {A: {X:1 Y:2}, B: {X:1 Z:3}}}
        build a dictionary: {X: {"K/A":1 "K/B": 1} Y: {"K/A": 2} Z: {"K/B" : 3}} """
    for key, val in evaldict.iteritems():
        if data_tools.behaves_like_dict(val):   # recurse
            buildNameIdentifierDict(val, name_entries_dict, curIdent + "/" + key)
        else:           # found final dict that contains name -> problems
            if not key in name_entries_dict:    # init
                name_entries_dict[key] = {}
            name_entries_dict[key][curIdent] = val

def writeTex(evaldict, filename, refDict):
    """ Write latex file for this dict.
        For dict: {X: {A: problems, B: problems}, Y: {A: problems, C: problems}}
        the output should be a table for A, B, C, where A has cols X/Y, B col X, C col Y
        """
    # First build the name -> {identifier -> entries} dict.
    nameEntriesDict = {}
    buildNameIdentifierDict(evaldict, nameEntriesDict, "")

    refEntriesDict = None
    if refDict:
        refEntriesDict = {}
        buildNameIdentifierDict(refDict, refEntriesDict, "")

    f = open(filename, "w")
    print >> f, '\\documentclass{article}'
    print >> f, "\\usepackage{caption}"
    print >> f, '\\begin{document}'

    if not refEntriesDict:      # runtime doesnt make sense for ref data unless we have some info
        writeTexTable(nameEntriesDict, f, "makespan", "<", refEntriesDict)
        writeTexTable(nameEntriesDict, f, "runtime", "<", refEntriesDict)
    else:
        writeTexTable(nameEntriesDict, f, "makespan", ">", refEntriesDict)

    print >> f, '\\end{document}'
    f.flush()
    f.close()

def writeTexTable(nameEntriesDict, f, target, better, refEntriesDict):
    """ Write latex table for this dict.
        Creates a table that has one row per problem.
        There is one column per Setting/Version.
        Target gives the target property of a problem to write in a column.
        If comparing the targets with better and item is equal to
        the best, the entry is marked bold.
        """
    for domain, val in nameEntriesDict.iteritems():    # domain -> {Run -> problems}
        refVals = None
        if refEntriesDict:
            try:
                refVals = refEntriesDict[domain]
                # don't really care about the ident for ref data, so just go down
                # in dict until we have the problem list
                while data_tools.behaves_like_dict(refVals):
                    assert len(refVals) == 1        # there should only be one ref data per domain
                    for rkey, rval in refVals.iteritems():
                        refVals = rval
                        break
                # print "REF vals for domain", domain, "is", refVals
            except:
                print "WARNING: No reference data for domain", domain, "- skipping domain!"
                continue
        print >> f, '\\begin{table}'
        print >> f, '  \\centering'
        print >> f, '  \\begin{tabular}{|l|c|c|}'
        print >> f, '  \\hline'
        
        # Runs contains:  Unique number, Descriptor Ident, Problem list
        runs = [ (num, ident, probs) for num, (ident,probs) in enumerate(val.iteritems()) ]

        # Write table header
        print >> f, '  Problem & ',
        for num, ident, probs in runs:
            print >> f, num,
            if num < len(val) - 1:
                print >> f, "&",
            else:
                print >> f, ""
        print >> f, "\\\\"
        print >> f, '  \\hline'
        
        # First collect all problems (some might not have entries)
        problems = set()
        for num, ident, probs in runs:
            for i in probs:
                problems.add(repr(i))
        probList = list(problems)
        probList.sort(key=str.lower)

        if refVals:
            probWithRefList = []
            for i in probList:
                refProbs = [p for p in refVals if repr(p) == i]
                if not refProbs:
                    print "No Ref data for domain", domain, " problem:", repr(i), "- skipping"
                    continue
                assert len(refProbs) == 1
                probWithRefList.append(i)
            probList = probWithRefList

        sums = dict( [ (num, 0) for num, ident, probs in runs ] )
        ref_sum = len(probList) # every ref prob scores 1.0 quality

        # Now for each problem, write a table line
        for i in probList:
            print >> f, i, "    &",
            best = None
            
            targetStr = "myProb[0]." + target
            refProbStr = "refProb[0]." + target
            # find best entry in line
            for num, ident, probs in runs:
                myProb = [j for j in probs if repr(j) == i]
                if refVals:
                    refProb = [j for j in refVals if repr(j) == i]
                    assert refProb
                    if myProb:
                        assert len(myProb) == 1
                        if not best:
                            best = eval(refProbStr + " / " + targetStr)
                        else:
                            if eval(refProbStr + " / " + targetStr + " " + better + " best"):
                                best = eval(refProbStr + " / " + targetStr)
                else:
                    if myProb:
                        assert len(myProb) == 1
                        if not best:
                            best = eval(targetStr)
                        else:
                            if eval(targetStr + " " + better + " best"):
                                best = eval(targetStr)

            # write actual entry for each ident
            for num, ident, probs in runs:
                myProb = [j for j in probs if repr(j) == i]
                if refVals:
                    refProb = [j for j in refVals if repr(j) == i]
                    assert refProb
                    if myProb:
                        assert len(myProb) == 1
                        if best and best == eval(refProbStr + " / " + targetStr):
                            print >> f, "\\textbf{",
                        print >> f, "%.2f" % eval(refProbStr + " / " + targetStr),
                        if best and best == eval(refProbStr + " / " + targetStr):
                            print >> f, "}",
                        sums[num] += eval(refProbStr + " / " + targetStr)
                    else:
                        print >> f, " - ",
                else:
                    if myProb:
                        assert len(myProb) == 1
                        if best and best == eval(targetStr):
                            print >> f, "\\textbf{",
                        print >> f, "%.2f" % eval(targetStr),
                        if best and best == eval(targetStr):
                            print >> f, "}",
                    else:                       # no entry for this run version -> "-"
                        print >> f, " - ",

                if num < len(val) - 1:
                    print >> f, "&",
                else:
                    print >> f, ""
            print >> f, "\\\\"

        print >> f, '  \\hline'
        print >> f, "Total &",
        for num, ident, probs in runs:
            print >> f, "%.2f" % sums[num], " / %.2f" % ref_sum,
            if num < len(val) - 1:
                print >> f, "&",
            else:
                print >> f, ""
        print >> f, "\\\\"

        print >> f, '  \\hline'
        print >> f, '  \\end{tabular}'
        print >> f, '  \\caption{\\textbf{', target, '} Domain: \\textbf{', domain, '}',
        print >> f, "\\\\"
        # write descriptor in caption matching run# to a description
        for num, ident, probs in runs:
            print >> f, str(num) + ": ", ident.replace("_", "\\_") + ",",
            print >> f, "\\\\"
        print >> f, ' }'
        print >> f, '\\end{table}'
        print >> f, ''

def checkPlans(evaldict, refDict):
    """ Check if plans in evaldict are equal to those in refDict
        """
    # First build the name -> {identifier -> entries} dict.
    nameEntriesDict = {}
    buildNameIdentifierDict(evaldict, nameEntriesDict, "")

    refEntriesDict = {}
    buildNameIdentifierDict(refDict, refEntriesDict, "")

    for domain, val in nameEntriesDict.iteritems():    # domain -> {Run -> problems}
        print "\nChecking Plans match ref data for domain:", domain
        refVals = None
        try:
            refVals = refEntriesDict[domain]
            # don't really care about the ident for ref data, so just go down
            # in dict until we have the problem list
            while data_tools.behaves_like_dict(refVals):
                assert len(refVals) == 1        # there should only be one ref data per domain
                for rkey, rval in refVals.iteritems():
                    refVals = rval                        
                    break
            #print "REF vals for domain", domain, "is", refVals
        except:
            print "WARNING: No reference data for domain", domain, "- skipping domain!"
            continue

        # Runs contains:  Unique number, Descriptor Ident, Problem list
        runs = [ (num, ident, probs) for num, (ident,probs) in enumerate(val.iteritems()) ]

        # First collect all problems (some might not have entries)
        problems = set()
        for num, ident, probs in runs:
            for i in probs:
                problems.add(repr(i))
        probList = list(problems)
        probList.sort(key=str.lower)

        probWithRefList = []
        for i in probList:
            refProbs = [p for p in refVals if repr(p) == i]
            if not refProbs:
                print "No Ref data for domain", domain, " problem:", repr(i), "- skipping"
                continue
            assert len(refProbs) == 1
            probWithRefList.append(i)
        probList = probWithRefList

        # Now for each problem, compare plan to ref data
        for i in probList:
            for num, ident, probs in runs:
                myProb = [j for j in probs if repr(j) == i]
                refProb = [j for j in refVals if repr(j) == i]
                samePlan = myProb[0].hasSamePlan(refProb[0])
                if samePlan:
                    print repr(myProb[0]), "OK"
                else:
                    print "Problem:", repr(myProb[0]), "for run", ident,
                    print "plan does NOT MATCH ref data"

def main():
    parser = OptionParser("usage: %prog PROBLEMS")
    parser.add_option("-e", "--eval-dir", dest="eval_dir", type="string", action="store")
    parser.add_option("-r", "--ref-data-dir", dest="ref_data_dir", type="string", action="store")
    parser.add_option("-c", "--check-plans", action="store_true", dest="check_plans", default=False)
    opts, args = parser.parse_args()
    print "Eval results dir: %s" % opts.eval_dir
    print "Ref data dir: %s" % opts.ref_data_dir
    print "Check plans against ref data: %s" % opts.check_plans

    evalDict = parseResults(opts.eval_dir)
    # print "FINAL EVAL DICT: ", evalDict

    ref_data = None
    if opts.ref_data_dir:
        ref_data = parseResults(opts.ref_data_dir)
        # print "REF DATA DICT: ", ref_data

    # write eval data
    writeEvalData(evalDict, ".")

    # create latex tables, all grouped by domain-name divided by settings/algos
    writeTex(evalDict, "output.tex", ref_data)
    
    if ref_data and opts.check_plans:
        checkPlans(evalDict, ref_data)

if __name__ == "__main__":
    main()

