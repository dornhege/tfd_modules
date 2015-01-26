#!/bin/bash

# Create a problems input from a directory
# Assumption there is exaclty one file domain...pddl and N x p...pddl files

DIR=$1

DOMAIN=$(ls $DIR/domain*.pddl)

for i in $(ls $DIR/p*.pddl); do
  echo $DIR `basename $DOMAIN` `basename $i`
done

