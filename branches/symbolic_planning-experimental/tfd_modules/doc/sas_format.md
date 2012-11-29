SAS+ format description
=======================

Description of the intermediate data passed from translation to preprocess
and from preprocess to search. PDDL/M is converted to SAS+ and preprocessed.
This file describes the file format that is used. Files are organized in sections.
Sections must appear in the order given.

The following sections are defined:

 1. Variables
 1. OPL Inits
 1. Objects
 1. PDDL Translation
 1. Constant Facts
 1. Modules
 1. Initial State
 1. Goal
 1. Operators
 1. Axioms

Preprocess adds the following additional sections:
 
 11. SG?
 12. Causal Graph (CG)
 13. Domain Transition Graph(s) (DTG)

Generic Sections
================

Variables
---------

    begin_variables
    117
    var0 2 -1
    var1 2 -1
    var2 2 -1
    var3 2 1
    ...
    var28 -1 -1
    var29 -2 -1
    ...
    var98 -3 -1
    ...
    end_variables

A section contains N variable definitions:

```Number of Variables  
(Variable Definition)+```

A variable definition is:
```VarName Domain DefaultValue``` FIXME: Is that default?

There are some special domain sizes:  
-1: Numerical fluent (PNE)  
-2: Condition Checker Module FIXME  
-3: Cost Module FIXME  

OPL Inits
---------

Objects
---------

PDDL Translation
---------

Constant Facts
---------

Modules
---------

    Number of Init Modules
    (Init Modules)*
    Number of Exit Modules
    (Exit Modules)*
    Number of Condition Checkers
    (Condition Checkers)*
    Number of Effect Applicators
    (Effect Applicators)*
    Number of Cost Modules
    (Cost Modules)*

Initial State
---------

Goal
---------

Operators
---------

    Number of Operators
    (Operator)+

Operator definitions differ, see program specific sections.

Axioms
------
Axiom definitions differ, see program specific sections.

Translator Output (output.sas)
==============================

Operator
--------

    begin_operator
    name ground_param*
    = DurationVar
    Number of at start conditions
    (Condition)*
    Number of over all conditions
    (Condition)*
    Number of at end conditions
    (Condition)*
    Number of at start effects
    (Effect)*
    Number of at end effects
    (Effect)*
    end_operator

A `Condition` is a pair definition that variable `Var` should have `Value`.

    Var Value

An `Effect` is one line defining a conditional effect:

    NumberAtStartConds (Condition)* NumberOverAllConds (Condition)* NumberAtEndConds (Condition)* Var PreValue PostValue

Given that all effect conditions hold `Var` is changed from `PreValue` to `PostValue`. That is, it is only changed if `Var` is `PreValue` in the state. A `PreValue` of -1 means always change.
Instead of `Var PreValue PostValue` an effect can have the special value `me-NUM` for effect applicator modules.

Example:

    begin_operator
    pick-up beer008 crate_008 side
    = 95
    3
    2 0
    51 0
    4 0
    0
    0
    0
    4
    0 0 0 0 -1 0
    0 0 1 2 0 2 -1 1
    0 0 1 4 0 4 -1 1
    0 0 0 me-22
    end_operator


Axioms
------
Rel
Comp
Func

Preprocess Output (output)
==========================

The preprocess output file starts with a single line
to signal if the problem is solvable in poly time.

`0` or `1`

Following are the generic sections (see above) and then the 
three preprocess specific sections.

Operator
--------

    begin_operator
    name ground_param*
    = DurationVar
    Number of at start conditions
    (Condition)*
    Number of over all conditions
    (Condition)*
    Number of at end conditions
    (Condition)*
    Number of symbolic at start effects
    (PreEffect)*
    Number of symbolic at end effects
    (PreEffect)*
    Number of numeric at start effects
    (PreEffect)*
    Number of numeric at end effects
    (PreEffect)*
    Number of module at start effects
    (PreEffect)*
    Number of module at end effects
    (PreEffect)*
    end_operator

A `Condition` is a pair definition that variable `Var` should have `Value`.

    Var Value

An `PreEffect` defines a conditional effect:

    NumberAtStartConds
    (Condition)*
    NumberOverAllConds
    (Condition)*
    NumberAtEndConds
    (Condition)*
    Var PreValue PostValue

Given that all effect conditions hold `Var` is changed from `PreValue` to `PostValue`. That is, it is only changed if `Var` is `PreValue` in the state. A `PreValue` of -1 means always change.
FIXME: numerical effects?
 
Instead of `Var PreValue PostValue` an effect can have the special value `me-NUM` for effect applicator modules.

Example:

    begin_operator
    pick-up beer008 crate_008 side
    = 95
    3
    2 0
    51 0
    4 0
    0
    0
    0
    3
    0
    0
    0
    0 -1 0
    0
    0
    1
    2 0
    2 -1 1
    0
    0
    1
    4 0
    4 -1 1
    0
    0
    0
    1
    0
    0
    0
    me-22
    end_operator


Axioms
------
7
0

SG?
-------

Causal Graph (CG)
-------

Domain Transition Graph(s) (DTG)
-------

