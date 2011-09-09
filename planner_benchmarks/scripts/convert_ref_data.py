#!/usr/bin/env python

# Take ref data in the format of 
# some-dirs: domain-name: 01/ 02/ 03/...
# where each numbered dir will contain a file plan.soln.1
# copy the file 04/plan.soln.1 to plan.p04.best 
# and create a times.p04.pddl with the makespan of plan.soln.1 and invalid timestamp

from __future__ import with_statement

from optparse import OptionParser
import os
import subprocess
import shutil
import collections
import datetime
import re

def parsePlan(file):
    # Matches: start_time: (some action stuff) [duration]
    exp = "([0-9\\.]+) *: *\\(([a-zA-Z0-9\\-_ ]+)\\) *\\[([0-9\\.]+)\\]"
    with open(file, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith(";"):    # empty lines, comments start with ;
                continue
            m = re.match(exp, line)
            if not m:
                print "NO MATCH FOR", line
            start_time = (float)(m.group(1))
            op = m.group(2)
            duration = (float)(m.group(3))
            yield (start_time, op, duration)


def convertDomainDir(dir, probdirs):
    """ Each probdir should contain:
        domain.pddl, problem.pddl, plan.soln.1 """
    print "Converting ", dir
    
    # first figure out, if we can just take the same domain
    allDomainsEqual = True
    for p in probdirs:
        domainx = os.path.join(dir, probdirs[0], "domain.pddl")
        domainy = os.path.join(dir, p, "domain.pddl")
        if not os.path.exists(domainx):
            continue
        if not os.path.exists(domainy):
            continue
        retcode = subprocess.call(["diff", "-q", domainx, domainy], stdout=subprocess.PIPE)
        if retcode != 0:
            allDomainsEqual = False
            break
    print "All domains equal:", allDomainsEqual
    if allDomainsEqual:
        # convert domain
        domain = os.path.join(dir, probdirs[0], "domain.pddl")
        newDomain = os.path.join(dir, "domain.pddl")
        shutil.copyfile(domain, newDomain)

    for d in probdirs:
        #print "Problem:", d
        # Convert plan
        plan = os.path.join(dir, d, "plan.soln.1")
        if not os.path.exists(plan):
            print "WARNING: No plan file at: " + plan
            continue
        newPlan = os.path.join(dir, "plan.p" + d + ".pddl.best")
        shutil.copyfile(plan, newPlan)
        # read makespan and create times file
        plan = parsePlan(newPlan)
        latest_timestamp = -1
        for (st, op, dur) in plan:
            ts = st + dur
            if ts > latest_timestamp:
                latest_timestamp = ts
        assert latest_timestamp >= 0
        timesfile = os.path.join(dir, "times.p" + d + ".pddl")
        with open(timesfile, "w") as f:
            print >> f, "# makespan search_time(s)"
            print >> f, latest_timestamp, "-1"
        # Convert domain/problem
        if not allDomainsEqual:
            # convert domain
            domain = os.path.join(dir, d, "domain.pddl")
            newDomain = os.path.join(dir, "domain.p" + d + ".pddl")
            shutil.copyfile(domain, newDomain)
        # convert problem
        problem = os.path.join(dir, d, "problem.pddl")
        newProblem = os.path.join(dir, "problem.p" + d + ".pddl")
        shutil.copyfile(problem, newProblem)

def convertRefData(ref_dir):
    """ Parse dir and all subdirectories to find domain dirs. 
        That is a directory that contains directories that only contain numbers,
        e.g. 01/ 02/ etc.
        Convert that dir to the eval format. """
    # List directories, a problem/domain directory contains one or more files with "pddl"

    allResults = {}
    for root, dirs, files in os.walk(ref_dir):
        #print "Root: ", root
        #print "Dirs: ", dirs
        #print "Files: ", files
        #print ""
        # If dirs contains a dir that is named "01" this is a domain dir
        reg = "^[0-9]+$"
        probs = [f for f in dirs if re.match(reg, f)]
        probs.sort()
        if probs:
            convertDomainDir(root, probs)

def main():
    parser = OptionParser("usage: %prog PROBLEMS")
    parser.add_option("-r", "--ref-dir", dest="ref_dir", type="string", action="store")
    opts, args = parser.parse_args()
    print "Ref-data dir: %s" % opts.ref_dir

    convertRefData(opts.ref_dir)

if __name__ == "__main__":
    main()

