#!/bin/bash

# Create a problems input from a directory
# Assumption there is exaclty N x p01.pddl files and d01.pddl files with N from 01 ... 30

DIR=$1

for i in $(seq --format "%02.0f" 1 30); do
  DOMAIN=$DIR/d$i.pddl
  PROBLEM=$DIR/d$i.pddl
  if [[ ! -e $DOMAIN ]]; then
      echo WARNING Domain $DOMAIN does not exist.
      continue
  fi
  if [[ ! -e $PROBLEM ]]; then
      echo WARNING Problem $PROBLEM does not exist.
      continue
  fi

  echo $DIR `basename $DOMAIN` `basename $PROBLEM`
done

