#!/usr/bin/env python

# Evaluate results from a run and compute tables, e.g. makespans, runtimes...

from __future__ import with_statement

from optparse import OptionParser
import os
import subprocess
import shutil
import collections
import datetime
import re

import data_tools
from plan_file_parser import parsePlan
from plan_file_parser import makespanFromPlan

class Problem(object):
    """ An evaluated problem. """
    def __init__(self, path, name):
        """ Default init using a problem name, e.g. 02, and a path that contains a plan...best and times file """
        self.name = name
        if path and self.name:
            # valid problem has times file + plan.best - if none, no data, if no plan, is there times???
            self.planfiles = []
            self.planfiles.append(os.path.join(path, "plan.p" + self.name + ".pddl.best"))
            self.timesfile = os.path.join(path, "times.p" + self.name + ".pddl")
            self.makespan, self.runtime = self.parseTimes()
    def fromDirectory(path, name):
        """ Constructor for a directory at "path/name/" that contains
            multiple plan.soln.### files """
        p = Problem(None, None)
        p.name = name
        data_dir = os.path.join(path, name)
        assert os.path.isdir(data_dir)
        reg = "^plan\\.soln\\.[0-9]+$"
        plan_files = [x for x in os.listdir(data_dir) if re.match(reg, x)]
        # sort by integer number to put, e.g. 10 behind 1,2,...,9
        def plan_nr(planfile):
            assert planfile.startswith("plan.soln.")
            nrInt = int(planfile[len("plan.soln."):])
            return nrInt
        plan_files.sort(key = plan_nr)
        p.planfiles = [os.path.join(path, name, plan) for plan in plan_files]
        # Read in plan files in numbered order!!!
        if p.planfiles:
            p.makespan = makespanFromPlan(parsePlan(p.planfiles[-1]))   # makespan is the makespan of the last/best 
        p.runtime = -1
        return p
    fromDirectory = staticmethod(fromDirectory)
    def fromData(name, makespan):
        """ Constructor from raw data without file reference """
        p = Problem(None, None)
        p.name = name
        p.makespan = makespan
        p.runtime = -1
        return p
    fromData = staticmethod(fromData)
    def dump(self):
        print "Name:",self.name, "Makespan:", self.makespan, "Runtime:", self.runtime
        print "Plans: %s, Times: %s" % (self.planfiles, self.timesfile)
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
    def hasSameFirstPlan(self, other):
        """ Compare my plan with the plan of other """
        if not self.planfiles or not other.planfiles:
            return False
        retcode = subprocess.call(["diff", "-q", self.planfiles[0], other.planfiles[0]], stdout=subprocess.PIPE)
        return retcode == 0

def readRefDataFromFile(ref_file):
    """ Read reference data from a file and return a problem dict
        Containing {Domain: problem_list}
        Format from linewise ipc2008 data:
        tempo-sat  temporal-fast-downward  transport-numeric  12        OK       982                433                 0.440936863544
        track       planner                 domain          problem#  solved?   planner-quality reference-quality       score=reference/planner-quality
        We are interested in: tempo-sat track, any planner (don't care), any domain, any problem -> read the reference-quality
    """
    theDict = {}
    with open(ref_file) as f:
        for line in f:
            line = line.strip()
            if not line.startswith("tempo-sat"):
                continue
            if not line:
                continue
            entries = line.split()
            assert len(entries) == 8, "Wrong number of entries in ref data line"
            domain = entries[2]
            problemNr = int(entries[3])
            problem = "%02d" % problemNr
            ref_quality_str = entries[6]
            ref_quality = None
            if not ref_quality_str == "n/a":
                ref_quality = float(entries[6])
            if not ref_quality:
                continue
            p = Problem.fromData(problem, ref_quality)
            if not domain in theDict:
                theDict[domain] = []
            matches = [x for x in theDict[domain] if repr(p) == repr(x)]
            if matches:
                for i in matches:
                    assert i.makespan == ref_quality
            else:
                theDict[domain].append(p)
    for key, val in theDict.iteritems():
        val.sort(key=lambda x: repr(x))
    return theDict

def evalDir(path, files, files_are_condensed):
    """ Evaluate the directory in path that contains a number of
        plan...pddl.best files.
        Returns a dict mapping from path to a list of Problems 
        
        If files_are_condensed is False the files list is a list of
        directories that contain one problem each with plan.soln.XXX files.
        """
    # First create a list of problems
    problems = []
    if files_are_condensed:
        # The expected format for problem named X with problem file X.pddl is:
        # plan.pX.pddl.best and times.pX.pddl
        plannames = []
        for f in files:
            if f.endswith(".pddl.best"):
                # plan.p03.pddl.best -> p03
                assert(f.startswith("plan.p"))
                assert(len(f) > len("plan.p") + len(".pddl.best"))
                name = f[len("plan.p") : - len(".pddl.best")]
                plannames.append(name)
        plannames.sort(key=str.lower)
        # valid names are those, where a times.X.pddl file exists
        validnames = [name for name in plannames if os.path.exists(os.path.join(path, "times.p" + name + ".pddl"))]
        problems.extend([Problem(path, name) for name in validnames])
    else:
        for i in files:
            p = Problem.fromDirectory(path, i)
            problems.append(p)

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
        # If files contains a file with "plan...pddl" in it, it is a result dir
        pddls = [f for f in files if f.lower().find("pddl") >= 0 and f.lower().find("plan.") >= 0]
        if pddls:
            dirEval = evalDir(root, files, True)
            allResults = data_tools.merge(allResults, dirEval)
        
        reg = "^[0-9]+$"    # only numbers
        probs = [f for f in dirs if re.match(reg, f)]
        probs.sort()
        if probs:
            dirEval = evalDir(root, probs, False)
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
        writeTexTable(nameEntriesDict, f, "runtime", ">", refEntriesDict)

    print >> f, '\\end{document}'
    f.flush()
    f.close()

def evaluateProblem(problem, referenceProblem, target):
    """ Evaluate problem's target property.
        If a referenceProblem is given it is evaluated with respect to that.
        In that case problem might be None if there was no data and None is returned. """

    if problem is None:
        return None

    targetStr = "problem." + target
    try:
        eval(targetStr)
    except AttributeError:
        return None

    refProbStr = "referenceProblem." + target
    if referenceProblem:
        try:
            return eval(refProbStr) / eval(targetStr)     # TODO generalize?
        except ZeroDivisionError:       # targetStr was incredibly small -> result in huge eval + debug
            return 999999.99
    else:
        return eval(targetStr)

def writeTexTableEntry(f, problem, referenceProblem, target, best, num, sums):
    """ Write one entry for an output table referring to the target property of problem.
        If referenceProblem is given the problem is compared to the referenceProblem
        and relative values are printed.
        Comparison in done with respect to best.
        In that case problem might be None if there was no result/data.
        Sums is a dictionary of num -> accumulated sum that should be updated. """

    targetStr = "problem." + target
    refProbStr = "referenceProblem." + target
    
    # write actual entry for each ident
    probVal = evaluateProblem(problem, referenceProblem, target)
    if probVal:
        print >> f, "{\\tiny ", "%.0f" % eval(targetStr), "} ",
        if best and best == probVal:   # this is the best entry
            print >> f, "\\textbf{",
        print >> f, "%.2f" % probVal,
        if best and best == probVal:
            print >> f, "}",
            sums[num] += probVal   # count for score
    else:
        print >> f, "-",

def writeTexTableLine(f, problem_id, runs, refVals, target, better, numEntries, sums):
    """ Write one line for problem_id in the output table referring to the target property of the problem.
        If refVals is given the problem is compared to the reference
        and relative values are printed.
        Comparison in done with the better property of the division of the properties.
        In that case runs might not contain a problem for problem_id if there was no result/data.
        numEntries is only used to decide when the line ends.
        Sums is a dictionary of run_num -> accumulated sum """
    
    print >> f, "   ", problem_id, "    &",
 
    # First find the referenceProblem for problem_id
    refProb = None
    if refVals:
        refProbSearch = [j for j in refVals if repr(j) == problem_id]
        assert refProbSearch
        refProb = refProbSearch[0]
   
    # find best entry in line
    best = None
    for num, ident, probs in runs:
        # First find the current problem for problem_id
        myProb = None
        myProbSearch = [j for j in probs if repr(j) == problem_id]
        if myProbSearch:
            myProb = myProbSearch[0]

        probVal = evaluateProblem(myProb, refProb, target) 
        if probVal is None:
            continue
        if not best:
            best = probVal
        else:
            if eval("probVal" + better + "best"):
                best = probVal

    # write actual entry for each ident
    for num, ident, probs in runs:
         # First find the current problem for problem_id
        myProb = None
        myProbSearch = [j for j in probs if repr(j) == problem_id]
        if myProbSearch:
            myProb = myProbSearch[0]

        writeTexTableEntry(f, myProb, refProb, target, best, num, sums)
  
        if num < numEntries - 1:
            print >> f, "&",
        else:
            print >> f, "",
    print >> f, "\\\\"

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

        # Runs contains:  Unique number, Descriptor Ident, Problem list
        runs = [ (num, ident, probs) for num, (ident,probs) in enumerate(val.iteritems()) ]

        # Write table header
        print "Writing table for", domain
        print >> f, '\\begin{table}'
        print >> f, '  \\centering'
        print >> f, '  \\begin{tabular}{|l',
        for num, ident, probs in runs:
            print >> f, "|c",
        print >> f, "|}"
        print >> f, '  \\hline'

        print >> f, '  Problem & ',
        for num, ident, probs in runs:
            print >> f, num,
            if num < len(val) - 1:
                print >> f, "&",
            else:
                print >> f, "",
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
            # verify check that all problems have ref-data
            for i in probList:
                refProbs = [p for p in refVals if repr(p) == i]
                if not refProbs:
                    print "No Ref data for domain", domain, " problem:", repr(i), "- skipping"
                    continue
            # probList = all refvals
            # i.e. ignore original probList (might get some with no problem, which is OK, empty entries)
            probList = [repr(i) for i in refVals]

        sums = dict( [ (num, 0) for num, ident, probs in runs ] )   # run# -> accumulated score
        ref_sum = len(probList) # every ref prob scores 1.0 quality

        # Now for each problem, write a table line
        for i in probList:
            writeTexTableLine(f, i, runs, refVals, target, better, len(val), sums)

        if refVals and target == "makespan":    # this only makes sense for makespans
            print >> f, '  \\hline'
            print >> f, "Total &",
            for num, ident, probs in runs:
                print >> f, "%.2f" % sums[num], " / %.2f" % ref_sum,
                if num < len(val) - 1:
                    print >> f, "&",
                else:
                    print >> f, "",
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
        print >> f, '\\clearpage'
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
                samePlan = myProb[0].hasSameFirstPlan(refProb[0])
                if samePlan:
                    print repr(myProb[0]), "OK"
                else:
                    print "Problem:", repr(myProb[0]), "for run", ident,
                    print "plan does NOT MATCH ref data"

def main():
    parser = OptionParser("usage: %prog ")
    parser.add_option("-e", "--eval-dir", dest="eval_dir", type="string", action="store")
    parser.add_option("-r", "--ref-data", dest="ref_data", type="string", action="store")
    parser.add_option("-c", "--check-plans", action="store_true", dest="check_plans", default=False)
    opts, args = parser.parse_args()
    print "Eval results dir: %s" % opts.eval_dir
    print "Ref data: %s" % opts.ref_data
    print "Check plans against ref data: %s" % opts.check_plans

    evalDict = parseResults(opts.eval_dir)
    # print "FINAL EVAL DICT: ", evalDict

    ref_data = None
    if opts.ref_data:
        if os.path.isdir(opts.ref_data):
            ref_data = parseResults(opts.ref_data)
        elif os.path.isfile(opts.ref_data):
            ref_data = readRefDataFromFile(opts.ref_data)
        else:
            assert False, "ref data is neither dir nor file"
        assert ref_data, "No ref_data read."
        print "Ref-Domains:", ", ".join(ref_data.keys())
        #print "REF DATA DICT: ", ref_data

    # write eval data
    #writeEvalData(evalDict, ".")

    # create latex tables, all grouped by domain-name divided by settings/algos
    writeTex(evalDict, "output.tex", ref_data)
    
    if ref_data and opts.check_plans:
        checkPlans(evalDict, ref_data)

if __name__ == "__main__":
    main()

