#!/bin/bash

# generate a fake times file for problem named $1 with makespan $2

echo ARGS: $*

NR=$1
MS=$2

echo \# makespan search_time > times.p$1.pddl
echo $2 -1 > times.p$1.pddl
touch plan.p$1.pddl.best

