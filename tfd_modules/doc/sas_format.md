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
 
 11. SG - Successor Generator?
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

    begin_oplinits
    Number of OPL Inits
    (OplInit)*
    end_oplinits

Objects
---------

    begin_objects
    Number of Objects
    (Object)*
    end_objects

where Object is `type object-id`, e.g. `movable beer008`.

PDDL Translation
---------

    begin_pddl_translation
    Number of Predicate Translations
    (PredicateTranslation)*
    Number of Numerical Fluent Translations
    (NumericalFluentTranslation)*
    end_pddl_translation

where a PredicateTranslation is:

    Name NumParameters (ParameterName)* Var Val

This declares that the ground predicate build by `Name` and the `ParameterName`s is true, iff `Var` is `Val`.

A NumericalFluentTranslation is defined as:

    Name NumParameters (ParameterName)* Var

The grounded numerical fluent `Name` `ParameterName`* is represented by state variable `Var`.

Constant Facts
---------

    begin_constant_facts
    Number of True Predicates
    (ConstPredicate)*
    Number of Numerical Constants
    (NumericalConstant)*
    end_constant_facts

where a `ConstPredicate` is

    Name NumParameters (ParameterName)*

and states that this fact is always true and a `NumericalConstant`

    Name NumParameters (ParameterName)* Value

state that `Name` `ParameterName`s is always `Value`, where value in this case in an
actual numerical value and not a state variable.

Modules
---------

    begin_modules
    Number of Init Modules
    (InitModule)*
    Number of Exit Modules
    (ExitModule)*
    Number of Condition Checkers
    (ConditionChecker)*
    Number of Effect Applicators
    (EffectApplicator)*
    Number of Cost Modules
    (CostModule)*
    Number of Grounding Modules
    (GroundingModule)*
    end_modules

An `InitModule` and `ExitModule` is given as:

    LibraryCall NumParameters (ParameterName)*

A `ConditionChecker` and `CostModule` are given as:

    LibraryCall NumParameters (Parameter)* Var

where a `Parameter` is `SchematicName Type Name`. SchematicName is the name in the schematic operator, e.g. `?x`.

An `EffectApplicator` is:

    LibraryCall NumParameters (Parameter)* EffectName EffectVar*

The `EffectName` is an identifier for a module effect and to be used as an effect
in operator declarations. `EffectVar` states the variables to be set.

An `GroundingModule` is:

    LibraryCall NumParameters (Parameter)* ModuleGrounding

The `ModuleGrounding` is an identifier for a module grounding and will be used in
in partially grounded operator declarations.

Initial State
---------

    begin_state
    Value*
    end_state

This just states the initial values in the state in the same order that
the variables are defined in the Variables section.

Example:

    begin_state
    1
    1
    0
    1
    0
    ...
    1.570796327
    ....
    end_state

Goal
---------

    begin_goal
    Number Of Goal Conditions
    (Var Val)*
    end_goal

This just states that `Var` should be `Val` for all entries in the goal.

Example:

    begin_goal
    2
    3 0
    2 1
    end_goal

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
    Number of grounding modules
    (ModuleGrounding)*
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

Currently only 0 or 1 grounding modules are supported.
A `ModuleGrounding` is an identifier `gm-ID` as declared in the modules section.

A `Condition` is a pair definition that variable `Var` should have `Value`.

    Var Value

An `Effect` is one line defining a conditional effect:

    NumberAtStartConds (Condition)* NumberOverAllConds (Condition)* NumberAtEndConds (Condition)* Var PreValue PostValue

Given that all effect conditions hold `Var` is changed from `PreValue` to `PostValue`. That is, it is only changed if `Var` is `PreValue` in the state. A `PreValue` of -1 means always change.
Instead of `Var PreValue PostValue` an effect can have the special value `me-NUM` for effect applicator modules.

Example:

    begin_operator
    pick-up beer008 crate_008 side
    0
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
    Number of grounding modules
    (ModuleGrounding)*
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

Currently only 0 or 1 grounding modules are supported.
A `ModuleGrounding` is an identifier `gm-ID` as declared in the modules section.

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
    0
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

