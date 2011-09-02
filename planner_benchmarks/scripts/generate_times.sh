#!/bin/bash

echo ARGS: $*

NR=$1
MS=$2

echo 0: $0 1: $1 2: $2 3: $3

echo \# makespan search_time > times.p$1.pddl
echo $2 -1 > times.p$1.pddl
touch plan.p$1.pddl.best

