#!/bin/bash

LATEST_REVISION=$(svn info | grep Revision: | cut -c11-)
TODAY=$(date +%Y-%m-%d)

echo REVISION: $LATEST_REVISION TODAY: $TODAY
echo Warning: Make sure that all directories are up2date with this revision and there are no outgoings

CONFIGS="tfd_modules_params_no_anytime_60s.yaml"
#CONFIGS="tfd_modules_params_no_anytime_60s.yaml tfd_modules_params_timeout_10_30.yaml"

#PROBLEMS="problem_sets/crewplanning.dat problem_sets/elevators-strips.dat problem_sets/openstacks-adl.dat problem_sets/pegsol-strips.dat problem_sets/transport.dat"
#PROBLEMS="problem_sets/crewplanning-strips.dat problem_sets/elevators-numeric.dat problem_sets/elevators-strips.dat problem_sets/openstacks-adl.dat problem_sets/openstacks-numericadl.dat problem_sets/openstacks-time.dat problem_sets/parcprinter-strips.dat problem_sets/pegsol-strips.dat problem_sets/sokoban-strips.dat problem_sets/transport-numeric.dat problem_sets/woodworking-numeric.dat"
PROBLEMS="problem_sets/crewplanning-strips.dat problem_sets/elevators-numeric.dat problem_sets/elevators-strips.dat problem_sets/openstacks-adl.dat problem_sets/openstacks-numericadl.dat problem_sets/parcprinter-strips.dat problem_sets/pegsol-strips.dat problem_sets/sokoban-strips.dat problem_sets/transport-numeric.dat problem_sets/woodworking-numeric.dat"

CONFIG_DIR=$(rospack find tfd_modules)/config/

OUTDIR="results_r${LATEST_REVISION}_$TODAY"
mkdir -p $OUTDIR

echo $TODAY >> $OUTDIR/runs

for c in $CONFIGS; do
  echo Running Config: $CONFIG_DIR$c
  for p in $PROBLEMS; do
    ./run_eval.py --problems $p --config $CONFIG_DIR$c --results-dir $OUTDIR
  done
done 

