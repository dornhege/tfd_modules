#!/usr/bin/env python

from optparse import OptionParser
import os
import subprocess
import shutil

# Perform an evaluation run

class Configuration(object):
    def __init__(self, configfile):
        assert(len(configfile) > 0)
        self.config = configfile
        # for file /bla/myconf.dat the name is myconf
        root,ext = os.path.splitext(configfile)
        assert(len(root) > 0)
        self.name = os.path.basename(root)
        assert(len(self.name) > 0)
    def dump(self):
        print "Config \"%s\" at %s" % (self.name, self.config)
    def set(self):
        """ Set config in ROS for planner call """
        cmd = "rosparam load %s /tfd_modules" % self.config
        code = os.system(cmd)
        assert(code == 0)

class Problem(object):
    def __init__(self, path, domain, problem):
        self.domain = domain
        self.problem = problem
        self.path = path
    def dump(self):
        print "Base path: %s Domain: %s, Problem: %s" % (self.path, self.domain, self.problem)
    def run(self, outdir):
        # create dir for results as outdir + relative path
        probDir = os.path.join(outdir, self.path)
        if not os.path.exists(probDir):
            os.makedirs(probDir)
        # create cmd
        # get problem base path from rospack find planner_benchmarks
        pipe = subprocess.Popen(["rospack", "find", "planner_benchmarks"], stdout=subprocess.PIPE)
        base_path = pipe.communicate()[0].strip()

        domain_path = os.path.join(base_path, self.path, self.domain)
        problem_path = os.path.join(base_path, self.path, self.problem)

        # set the tfd_modules/plan param for output
        cmd = "rosparam set /tfd_modules/plan_name %s" % (os.path.join(outdir, self.path, "plan." + self.problem))
        code = os.system(cmd)
        assert(code == 0)
        cmd = "rosparam set /tfd_modules/time_debug_file %s" % (os.path.join(outdir, self.path, "times." + self.problem))
        code = os.system(cmd)
        assert(code == 0)

        print "Exec run for %s, %s" % (domain_path, problem_path)
        #cmd = "rosrun tfd_modules tfd_plan_eval %s %s plan.%s 3600 %s" % \
         #   (domain_path, problem_path, self.problem, probDir)
        cmd = "rosrun tfd_modules tfd_plan %s %s" % \
            (domain_path, problem_path)

        # run it
        code = os.system(cmd)
        print "%s run with %d" % (self.problem, code)
    def parse(line):
        dp = line.split()
        assert(len(dp) == 3)
        return Problem(dp[0], dp[1], dp[2])
    parse = staticmethod(parse)


def main():
    parser = OptionParser("usage: %prog PROBLEMS")
    parser.add_option("-p", "--problems", dest="problems", type="string", action="store")
    # format: path domain problem [per line]
    parser.add_option("-r", "--results-dir", dest="results_dir", type="string", action="store")
    parser.add_option("-c", "--config", dest="config", type="string", action="store")
    opts, args = parser.parse_args()
    print "Problems file: %s" % opts.problems
    print "Results dir: %s" % opts.results_dir

    config = Configuration(opts.config)
    config.dump()
    config.set()

    problems = []
    with open(opts.problems) as f:
        for line in f:
            prob = Problem.parse(line)
            problems.append(prob)
    for i in problems:
        i.dump()

    if not os.path.exists(opts.results_dir):
        os.mkdir(opts.results_dir)
    outdir = os.path.join(opts.results_dir, config.name)
    if os.path.exists(outdir):
        print "WARNING outdir %s already exists!" % outdir
    else:
        os.mkdir(outdir)

    # save the config
    shutil.copy(config.config, outdir)
    # save the run script
    pipe = subprocess.Popen(["rospack", "find", "tfd_modules"], stdout=subprocess.PIPE)
    tfd_path = pipe.communicate()[0].strip()
    shutil.copy(os.path.join(tfd_path, "scripts/tfd_plan"), outdir)

    # run all problems
    for num, i in enumerate(problems):
        print "Running: %s: %s - %s (%.1f %%)" % (i.path, i.domain, i.problem, 100.0*num/len(problems))
        i.run(outdir)

if __name__ == "__main__":
    main()

