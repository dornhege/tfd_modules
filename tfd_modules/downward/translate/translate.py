#! /usr/bin/env python
# -*- coding: latin-1 -*-
import sys

import axiom_rules
import fact_groups
import instantiate
import numeric_axiom_rules
import pddl
import sas_tasks
import simplify

# The translator may generate trivial derived variables which are always true,
# for example if there ia a derived predicate in the input that only depends on
# (non-derived) variables which are detected as always true.
# Such a situation was encountered in the PSR-STRIPS-DerivedPredicates domain.
# Such "always-true" variables should best be compiled away, but it is
# not clear what the best place to do this should be. Similar
# simplifications might be possible elsewhere, for example if a
# derived variable is synonymous with another variable (derived or
# non-derived).

ALLOW_CONFLICTING_EFFECTS = False
USE_PARTIAL_ENCODING = True
WRITE_ALL_MUTEXES = True

def strips_to_sas_dictionary(groups, num_axioms, num_axiom_map, num_fluents, modules):
    dictionary = {}
    mod_effects_dict = {}

    # sort groups to get a deterministic output
    map(lambda g: g.sort(lambda x, y: cmp(str(x),str(y))),groups)
    groups.sort(lambda x, y: cmp((-len(x),str(x[0])),(-len(y),str(y[0]))))
 
    for var_no, group in enumerate(groups):
        for val_no, atom in enumerate(group):
            dictionary.setdefault(atom, []).append((var_no, val_no))
    if USE_PARTIAL_ENCODING:
        assert all(len(sas_pairs) == 1
                   for sas_pairs in dictionary.itervalues())

    redundant_axioms = []
    num_ax_count = 0
    for axiom in num_axioms:
        if axiom.effect in num_axiom_map:
            redundant_axioms.append(axiom.effect)
        else:
            dictionary.setdefault(axiom.effect,[]).append((num_ax_count + len(groups), -2))
            num_ax_count += 1
    for axiom_effect in redundant_axioms:
            dictionary[axiom_effect] = dictionary[num_axiom_map[axiom_effect].effect]

    ranges = [len(group) + 1 for group in groups] + [-1]*num_ax_count

    var_no = len(groups) + num_ax_count
    fluent_list = list(num_fluents)
    fluent_list.sort(lambda x,y: cmp(str(x), str(y)))
    for fluent in fluent_list: # are partially contained in num_axiom
        if fluent not in dictionary:
            dictionary.setdefault(fluent,[]).append((var_no, -2))
            var_no += 1
            ranges.append(-1)
    
    module_list = list(modules)
    module_list.sort(lambda x,y: cmp(str(x), str(y)))
    mod_eff_no = 0
    for module in module_list:
      moduleCall = module.toModuleCall()
      if module.type == "effect":
        if moduleCall not in mod_effects_dict:
          mod_effects_dict.setdefault(moduleCall, []).append(mod_eff_no)
          # might be enough just to set that to mod_eff_no
          mod_eff_no += 1
      else:
        if moduleCall not in dictionary:
          # hm, if we use 1 here, we could even handle negated effects
          # see if this happens, when effects are not at toplevel in a condition
          dictionary.setdefault(moduleCall, []).append((var_no, 0))
          var_no += 1
          if module.type == "conditionchecker":
            ranges.append(-2)
          elif module.type == "cost":
            ranges.append(-3)
          else:
            assert False, "Unknown module type in dictionary"
        
    return ranges, dictionary, mod_effects_dict

def translate_strips_conditions(conditions, dictionary, ranges, comp_axioms, 
                                temporal=False):
    if temporal:
        condition = [translate_strips_conditions_aux(conds, dictionary, ranges, comp_axioms) 
                     for conds in conditions] 
        if None in condition:
            return None
        else:
            return condition
    else:
        return translate_strips_conditions_aux(conditions, dictionary, ranges, comp_axioms)


def translate_strips_conditions_aux(conditions, dictionary, ranges, comparison_axioms):
    if not conditions:
        return {} # Quick exit for common case.

    condition = {}
    comp_axiom_dict = comparison_axioms[0]
    sas_comp_axioms = comparison_axioms[1]
    for fact in conditions:
        if (isinstance(fact,pddl.FunctionComparison) or 
            isinstance(fact,pddl.NegatedFunctionComparison)):
            if fact not in dictionary:
                parts = [dictionary[part][0][0] for part in fact.parts]
                key = (fact.comparator, tuple(parts))
                negated = fact.negated
                if key in comp_axiom_dict:
                    fact = comp_axiom_dict[key]
                    if negated:
                        fact = fact.negate()
                else:
                    axiom = sas_tasks.SASCompareAxiom(fact.comparator, parts, len(ranges)) 
                    sas_comp_axioms.append(axiom)
                    if negated:
                        negfact = fact
                        posfact = fact.negate()
                    else:
                        posfact = fact
                        negfact = fact.negate()
                    comp_axiom_dict[key] = posfact
                    dictionary.setdefault(posfact,[]).append((len(ranges), 0))
                    dictionary.setdefault(negfact,[]).append((len(ranges), 1))
                    ranges.append(3)
            var, val = dictionary[fact][0]
            if condition.get(var) not in (None, val):
                # Conflicting conditions on this variable: Operator invalid.
                return None
            condition[var] = val 
        elif isinstance(fact, pddl.ModuleCall):
            assert fact in dictionary
            # warum braucht der else zweig hier ein Atom als zwischekonstrukt?
            for var, val in dictionary[fact]:
              if condition.get(var) not in (None, val):
                    # Conflicting conditions on this variable: Operator invalid.
                    return None
              # Warum koennen das mehr als 1 eintrag sein?
              condition[var] = val
        else:
            atom = pddl.Atom(fact.predicate, fact.args) # force positive
            try:
                for var, val in dictionary[atom]:
                    if fact.negated:
                        ## BUG: Here we take a shortcut compared to Sec. 10.6.4
                        ##      of the thesis and do something that doesn't appear
                        ##      to make sense if this is part of a proper fact group.
                        ##      Compare the last sentences of the third paragraph of
                        ##      the section.
                        ##      We need to do what is written there. As a test case,
                        ##      consider Airport ADL tasks with only one airport, where
                        ##      (occupied ?x) variables are encoded in a single variable,
                        ##      and conditions like (not (occupied ?x)) do occur in
                        ##      preconditions.
                        ##      However, *do* what we do here if this is a binary
                        ##      variable, because this happens to be the most
                        ##      common case.
                        val = ranges[var] - 1
                    if condition.get(var) not in (None, val):
                        # Conflicting conditions on this variable: Operator invalid.
                        return None
                    condition[var] = val
            except KeyError as e:
                print "Atom not in dictionary: ", atom.dump()
                raise
    return condition

def translate_operator_duration(duration, dictionary):
    sas_durations = []
    for timed_duration in duration:
        timed_sas_durations = []
        for dur in timed_duration:
            var, val = dictionary.get(dur[1])[0]
            timed_sas_durations.append(sas_tasks.SASDuration(dur[0],var))
        sas_durations.append(timed_sas_durations)
    return sas_durations

def mutex_conditions(cond_dict, condition, temporal):
    # return value True means that the conditions are mutex
    # return value False means that we don't know whether they are mutex
    if temporal:
        for time in range(3):
            for var,val in condition[time]:
                if var in cond_dict[time]:
                    if cond_dict[time][var] != val:
                        return True
    else:
        for var,val in condition:
            if var in cond_dict:
                if cond_dict[var] != val:
                    return True
    return False

def implies(condition, condition_list, global_cond, temporal):
    # True: whenever condition is true also at least one condition 
    # from condition_list is true (given global_cond)
    if temporal:
        if [[],[],[]] in condition_list:
            return True
        for cond in condition_list:
            triggers = True
            for time in range(3):
                for (var,val) in cond[time]:
                    if (var,val) not in condition[time] and global_cond[time].get(var)!=val:
                        triggers=False
                        break
                if not triggers:
                    break
            if triggers:
                return True
    else:
        if [] in condition_list:
            return True
        for cond in condition_list:
            triggers = True
            for (var,val) in cond:
                if (var,val) not in condition and global_cond.get(var)!=val:
                    triggers=False
                    break
            if triggers:
                return True
    return False

def translate_add_effects(add_effects, dictionary, mod_effects_dict, ranges, comp_axioms, temporal=False):
    effect = {}
    mod_effect = {}
    possible_add_conflict = False

    for conditions, fact in add_effects:
        eff_condition_dict = translate_strips_conditions(conditions, dictionary, 
                                         ranges, comp_axioms, temporal)
        if eff_condition_dict is None: # Impossible condition for this effect.
            continue

        if temporal:
            eff_condition = [eff_cond.items() for eff_cond in eff_condition_dict]
        else:
            eff_condition = eff_condition_dict.items()
        if isinstance(fact, pddl.ModuleCall):
          for eff_num in mod_effects_dict[fact]:
            assert eff_num not in mod_effect
            mod_effect.setdefault(eff_num, eff_condition)
        # wie geht hier das: 3 1 2 (3 von 1 auf 2), da unten steht ja nur die eff_con
        # die ist doch fuer den ganzen, oder?!?
        # PRE_POST gebastel eins drueber ansehen
        else:
          for var, val in dictionary[fact]:
            hitherto_effect = effect.setdefault(var,{})
            for other_val in hitherto_effect:
                if other_val != val:
                    for other_cond in hitherto_effect[other_val]:
                        if not mutex_conditions(eff_condition_dict, 
                                                other_cond, temporal):
                            possible_add_conflict = True
            hitherto_effect.setdefault(val,[]).append(eff_condition)
    return effect, possible_add_conflict, mod_effect

def translate_del_effects(del_effects,dictionary,ranges,effect,condition,
                          comp_axioms, temporal=False, time=None):
    if temporal:
        assert time is not None

    for conditions, fact in del_effects:
        eff_condition_dict = translate_strips_conditions(conditions, dictionary,
                                                  ranges, comp_axioms, temporal)
        if eff_condition_dict is None:
            continue

        if temporal:
            eff_condition = [eff_cond.items() for eff_cond in eff_condition_dict]
        else:
            eff_condition = eff_condition_dict.items()
        
        for var, val in dictionary[fact]:
            none_of_those = ranges[var] - 1
            hitherto_effects = effect.setdefault(var,{})
            
            # Look for matching add effect; ignore this del effect if found
            found_matching_add_effect = False
            uncertain_conflict = False

            for other_val, eff_conditions in hitherto_effects.items():
                if other_val!=none_of_those:
                    if implies(eff_condition, eff_conditions, condition, temporal):
                        found_matching_add_effect = True
                        break
                    for cond in eff_conditions:
                        if not mutex_conditions(eff_condition_dict, 
                                                cond, temporal): 
                            uncertain_conflict = True
            if found_matching_add_effect:
                continue
            else:
                assert not uncertain_conflict, "Uncertain conflict"
                if temporal:
                    if (condition[time].get(var) != val and 
                       eff_condition_dict[time].get(var) != val):
                        # Need a guard for this delete effect.
                        assert (var not in condition[time] and 
                               var not in eff_condition[time]), "Oops?"
                        eff_condition[time].append((var, val))
                else:
                    if condition.get(var) != val and eff_condition_dict.get(var) != val:
                        # Need a guard for this delete effect.
                        assert var not in condition and var not in eff_condition, "Oops?"
                        eff_condition.append((var, val))
                eff_conditions = hitherto_effects.setdefault(none_of_those,[])
                eff_conditions.append(eff_condition)

def translate_assignment_effects(assign_effects, dictionary, ranges, comp_axioms, 
                                 temporal=False):
    effect = {}
    possible_assign_conflict = False

    for conditions, assignment in assign_effects:
        eff_condition_dict = translate_strips_conditions(conditions, dictionary, 
                                         ranges, comp_axioms, temporal)
        if eff_condition_dict is None: # Impossible condition for this effect.
            continue

        if temporal:
            eff_condition = [eff_cond.items() for eff_cond in eff_condition_dict]
        else:
            eff_condition = eff_condition_dict.items()
        for var, dummy in dictionary[assignment.fluent]:
            for expvar, dummy in dictionary[assignment.expression]:
                val = (assignment.symbol, expvar)
                hitherto_effect = effect.setdefault(var,{})
                for other_val in hitherto_effect:
                    if other_val != val:
                        for other_cond in hitherto_effect[other_val]:
                            if not mutex_conditions(eff_condition_dict, 
                                                    other_cond, temporal):
                                possible_assign_conflict = True
                hitherto_effect.setdefault(val,[]).append(eff_condition)
    return effect, possible_assign_conflict

def translate_strips_operator(operator, dictionary, mod_effects_dict, ranges, comp_axioms):
    # NOTE: This function does not really deal with the intricacies of properly
    # encoding delete effects for grouped propositions in the presence of
    # conditional effects. It should work ok but will bail out in more
    # complicated cases even though a conflict does not necessarily exist.

    condition = translate_strips_conditions(operator.condition, dictionary, ranges, comp_axioms)
    if condition is None:
        return None

    effect, possible_add_conflict, mod_eff = translate_add_effects(operator.add_effects, 
                                                          dictionary, mod_effects_dict, ranges, comp_axioms)
    translate_del_effects(operator.del_effects,dictionary,ranges,effect,condition, comp_axioms)

    if possible_add_conflict:
        print operator.name
    assert not possible_add_conflict, "Conflicting add effects?"

    assign_effect, possible_assign_conflict = \
        translate_assignment_effects(operator.assign_effects, dictionary, ranges, comp_axioms)
    
    if possible_assign_conflict:
        print operator.name
    assert not possible_assign_conflict, "Conflicting assign effects?"

    pre_post = []
    for var in effect:
        for (post, eff_condition_lists) in effect[var].iteritems():
            pre = condition.get(var, -1)
            if pre != -1:
                del condition[var]
            for eff_condition in eff_condition_lists:
                pre_post.append((var, pre, post, eff_condition))
    prevail = condition.items()

    assign_effects = []
    for var in assign_effect:
        for ((op, valvar), eff_condition_lists) in assign_effect[var].iteritems():
            for eff_condition in eff_condition_lists:
                sas_effect = sas_tasks.SASAssignmentEffect(var, op, valvar, 
                                                       eff_condition)
                assign_effects.append(sas_effect)
    # add mod_eff to a SASOperator
    return sas_tasks.SASOperator(operator.name, prevail, pre_post, assign_effects)
    
def translate_temporal_strips_operator(operator, dictionary, mod_effects_dict, ranges, comp_axioms):
    # NOTE: This function does not really deal with the intricacies of properly
    # encoding delete effects for grouped propositions in the presence of
    # conditional effects. It should work ok but will bail out in more
    # complicated cases even though a conflict does not necessarily exist.

    duration = translate_operator_duration(operator.duration, dictionary)
    condition = translate_strips_conditions(operator.conditions, 
                                dictionary, ranges, comp_axioms, True)
    if condition is None:
        print "condition is None"
        return None

    effect = []
    mod_effects = []
    possible_add_conflict = False
    for time in range(2):
        eff, poss_conflict, mod_eff = translate_add_effects(operator.add_effects[time], 
                                          dictionary, mod_effects_dict, ranges, comp_axioms, True)
        translate_del_effects(operator.del_effects[time], dictionary, ranges, 
                              eff, condition, comp_axioms, True, time)
        effect.append(eff)
        mod_effects.append(mod_eff)
        possible_add_conflict |= poss_conflict

    if possible_add_conflict:
        print operator.name
    assert not possible_add_conflict

    assign_effect = []
    possible_assign_conflict = False
    for time in range(2):
        eff, conflict = translate_assignment_effects(operator.assign_effects[time], 
                                                     dictionary, ranges, comp_axioms, True)
        assign_effect.append(eff)
        possible_assign_conflict |= conflict
    
    if possible_assign_conflict:
        print operator.name
    assert not possible_assign_conflict

    pre_post = [[],[]]
    for time in range(2):
        cond_time = time*2 # start -> start condition, end -> end_condition
        for var in effect[time]:
            for (post, eff_condition_lists) in effect[time][var].iteritems():
                pre = condition[cond_time].get(var, -1)
                if pre != -1:
                    del condition[cond_time][var]

                # substitute normal effect for conditional effects if it has only
                # one at-start condition on a binary variable which is equivalent
                # to the effect variable
                # example: temporal effect 1 6 0 0 0 6 -1 1
                #          becomes 0 0 0 6 0 1 
                if len(eff_condition_lists) == 1: # only one conditon
                    eff_condition = eff_condition_lists[0]
                    if (eff_condition[1] == [] and eff_condition[2] == [] and
                        len(eff_condition[0]) == 1):
                        ecvar, ecval = eff_condition[0][0]
                        if ecvar == var and ranges[var] == 2:
                            eff_condition[0] = []
                for eff_condition in eff_condition_lists:
                    pre_post[time].append((var, pre, post, eff_condition))
    prevail = [cond.items() for cond in condition]

    assign_effects = [[],[]]
    for time in range(2):
        for var in assign_effect[time]:
            for ((op, valvar), eff_condition_lists) \
                in assign_effect[time][var].iteritems():
                for eff_condition in eff_condition_lists:
                    sas_effect = sas_tasks.SASAssignmentEffect(var, op, valvar, 
                                                           eff_condition, True)
                    assign_effects[time].append(sas_effect)

    return sas_tasks.SASTemporalOperator(operator.name, duration, 
                prevail, pre_post, assign_effects, mod_effects)
    
def translate_strips_axiom(axiom, dictionary, ranges, comp_axioms):
    condition = translate_strips_conditions(axiom.condition, dictionary, ranges, comp_axioms)
    if condition is None:
        return None
    if axiom.effect.negated:
        [(var, _)] = dictionary[axiom.effect.positive()]
        effect = (var, ranges[var] - 1)
    else:
        [effect] = dictionary[axiom.effect]
    return sas_tasks.SASAxiom(condition.items(), effect)

def translate_numeric_axiom(axiom, dictionary):
    effect = dictionary.get(axiom.effect)[0][0]
    op = axiom.op
    parts = []
    for part in axiom.parts:
        if isinstance(part, pddl.PrimitiveNumericExpression):
            parts.append(dictionary.get(part)[0][0])
        else: # part is PropositionalNumericAxiom
            parts.append(dictionary.get(part.effect)[0][0])
    return sas_tasks.SASNumericAxiom(op, parts, effect)

def translate_strips_operators(actions, strips_to_sas, module_effects_to_sas, ranges, comp_axioms):
    result = []
    actions.sort(lambda x,y: cmp(x.name,y.name))
    for action in actions:
        sas_op = translate_strips_operator(action, strips_to_sas, module_effects_to_sas, ranges, comp_axioms)
        if sas_op:
            result.append(sas_op)
    return result

def translate_temporal_strips_operators(actions, strips_to_sas, module_effects_to_sas, ranges, comp_axioms):
    result = []
    actions.sort(lambda x,y: cmp(x.name,y.name))
    for action in actions:
        sas_op = translate_temporal_strips_operator(action, strips_to_sas, module_effects_to_sas, ranges, comp_axioms)
        if sas_op:
            result.append(sas_op)
    return result

def translate_strips_axioms(axioms, strips_to_sas, ranges, comp_axioms):
    result = []
    axioms.sort(lambda x,y: cmp(x.name,y.name))
    for axiom in axioms:
        sas_axiom = translate_strips_axiom(axiom, strips_to_sas, ranges, comp_axioms)
        if sas_axiom:
            result.append(sas_axiom)
    return result

def translate_task(strips_to_sas, module_effects_to_sas, ranges, init, goals, actions, 
                   durative_actions, axioms, num_axioms, num_axioms_by_layer, 
                   max_num_layer, num_axiom_map, const_num_axioms,
                   modules, module_inits, subplan_generators, init_constant_predicates, init_constant_numerics):

    axioms, axiom_init, axiom_layer_dict = axiom_rules.handle_axioms(
      actions, durative_actions, axioms, goals)

    init = init + axiom_init

    comp_axioms = [{},[]]
    goal_pairs = translate_strips_conditions(goals, strips_to_sas, ranges, comp_axioms).items()
    goal = sas_tasks.SASGoal(goal_pairs)

    operators = translate_strips_operators(actions,
                                        strips_to_sas, module_effects_to_sas, ranges, comp_axioms)
    temp_operators = translate_temporal_strips_operators(durative_actions, 
                                        strips_to_sas, module_effects_to_sas, ranges, comp_axioms)
    
    axioms = translate_strips_axioms(axioms, strips_to_sas, ranges, comp_axioms)
    sas_num_axioms = [translate_numeric_axiom(axiom,strips_to_sas) for axiom in num_axioms 
                      if axiom not in const_num_axioms and
                      axiom.effect not in num_axiom_map]


    axiom_layers = [-1] * len(ranges)
    
    ## each numeric axiom gets its own layer (a wish of a colleague for 
    ## knowledge compilation or search. If you use only the translator,
    ## you can change this)
    num_axiom_layer = 0
    for layer in num_axioms_by_layer:
        num_axioms_by_layer[layer].sort(lambda x,y: cmp(x.name,y.name))
        for axiom in num_axioms_by_layer[layer]:
            if axiom.effect not in num_axiom_map:
                [(var,val)] = strips_to_sas[axiom.effect]
                if layer == -1:
                    axiom_layers[var] = -1
                else:
                    axiom_layers[var] = num_axiom_layer
                    num_axiom_layer += 1
    for axiom in comp_axioms[1]:
        axiom_layers[axiom.effect] = num_axiom_layer
    for atom, layer in axiom_layer_dict.iteritems():
        assert layer >= 0
        [(var, val)] = strips_to_sas[atom]
        axiom_layers[var] = layer + num_axiom_layer + 1
    variables = sas_tasks.SASVariables(ranges, axiom_layers)

    init_values = [rang - 1 for rang in ranges]
    # Closed World Assumption: Initialize to "range - 1" == Nothing.
    for fact in init:
        if isinstance(fact,pddl.Atom):
            pairs = strips_to_sas.get(fact, [])  # empty for static init facts
            for var, val in pairs:
                assert init_values[var] == ranges[var] - 1, "Inconsistent init facts!"
                init_values[var] = val
        else: # isinstance(fact,pddl.FunctionAssignment)
            pairs = strips_to_sas.get(fact.fluent,[]) #empty for constant functions 
            for (var,dummy) in pairs:
                val = fact.expression.value
                assert init_values[var] == ranges[var] - 1, "Inconsistent init facts!"
                init_values[var]=val
    for axiom in const_num_axioms:
        var = strips_to_sas.get(axiom.effect)[0][0]
        val = axiom.parts[0].value
        init_values[var]=val
    init = sas_tasks.SASInit(init_values)
    
    strips_condition_modules = [module for module in modules if module.type == "conditionchecker"]
    strips_effect_modules = [module for module in modules if module.type == "effect"]
    strips_cost_modules = [module for module in modules if module.type == "cost"]
    strips_condition_modules.sort(lambda x,y : cmp(str(x), str(y)))
    strips_effect_modules.sort(lambda x,y : cmp(str(x), str(y)))
    strips_cost_modules.sort(lambda x,y : cmp(str(x), str(y)))
    condition_modules = []
    effect_modules = []
    cost_modules = []

    for mod in strips_condition_modules:
      assert mod.parent is not None
      sas_params = []
      for (ground_param, pddl_param) in zip(mod.parameters, mod.parent.parameters):
        sas_params.append((pddl_param.name, ground_param.type, ground_param.name))
      # strips_to_sas call is very hacky
      assert len(strips_to_sas[mod.toModuleCall()]) == 1
      assert len(strips_to_sas[mod.toModuleCall()][0]) == 2
      mod_var = strips_to_sas[mod.toModuleCall()][0][0]
      condition_modules.append(sas_tasks.SASConditionModule(mod.modulecall, sas_params, mod_var))
    for mod in strips_effect_modules:
      assert mod.parent is not None
      sas_params = []
      for (ground_param, pddl_param) in zip(mod.parameters, mod.parent.parameters):
        sas_params.append((pddl_param.name, ground_param.type, ground_param.name))
      sas_effs = []
      for eff in mod.effects:
        assert len(strips_to_sas[eff]) == 1
        assert len(strips_to_sas[eff][0]) == 2
        sas_effs.append(strips_to_sas[eff][0][0])
      # missing eff num + eff vars
      effect_modules.append(sas_tasks.SASEffectModule(mod.modulecall, sas_params, module_effects_to_sas[mod.toModuleCall()][0], sas_effs))
    for mod in strips_cost_modules:
      assert mod.parent is not None # ?
      sas_params = []
      for (ground_param, pddl_param) in zip(mod.parameters, mod.parent.parameters):
        sas_params.append((pddl_param.name, ground_param.type, ground_param.name))
      # make sure strips_to_sas is not mixed badly with condition modules
      assert len(strips_to_sas[mod.toModuleCall()]) == 1
      assert len(strips_to_sas[mod.toModuleCall()][0]) == 2
      mod_var = strips_to_sas[mod.toModuleCall()][0][0]
      cost_modules.append(sas_tasks.SASConditionModule(mod.modulecall, sas_params, mod_var))

    return sas_tasks.SASTask(variables, init, goal, operators, 
                             temp_operators, axioms, sas_num_axioms, comp_axioms[1], condition_modules, effect_modules, cost_modules, sas_tasks.SASTranslation(strips_to_sas), module_inits, subplan_generators, 
                             init_constant_predicates, init_constant_numerics)

def unsolvable_sas_task(msg):
    print "%s! Generating unsolvable task..." % msg
    variables = sas_tasks.SASVariables([2], [-1])
    init = sas_tasks.SASInit([0])
    goal = sas_tasks.SASGoal([(0, 1)])
    operators = []
    temp_operators = []
    axioms = []
    num_axioms = []
    comp_axioms = []
    condition_modules = []
    effect_modules = []
    cost_modules = []
    strips_to_sas = {}
    module_inits = []
    subplan_generators = []
    init_cons_pred = []
    init_cons_numer = []
    return sas_tasks.SASTask(variables, init, goal, operators,
            temp_operators, axioms, num_axioms, comp_axioms, condition_modules, effect_modules, cost_modules, sas_tasks.SASTranslation(strips_to_sas), module_inits, subplan_generators,
            init_cons_pred, init_cons_numer)

def pddl_to_sas(task):
    print "Instantiating..."
    (relaxed_reachable, atoms, num_fluents, actions, 
        durative_actions, axioms, num_axioms, modules, 
        init_constant_predicates, init_constant_numerics) = instantiate.explore(task)

    if not relaxed_reachable:
        return unsolvable_sas_task("No relaxed solution")

    num_axioms = list(num_axioms)
    num_axioms.sort(lambda x,y: cmp(x.name,y.name))

    # HACK! Goals should be treated differently.
    # Update: This is now done during normalization. The assertions
    # are only left here to be on the safe side. Can be removed eventually
    if isinstance(task.goal, pddl.Conjunction):
        goal_list = task.goal.parts
    else:
        goal_list = [task.goal]
    for item in goal_list:
        assert isinstance(item, pddl.Literal)

    groups, mutex_groups, translation_key = fact_groups.compute_groups(
        task, atoms, return_mutex_groups=WRITE_ALL_MUTEXES,
        partial_encoding=USE_PARTIAL_ENCODING)

    num_axioms_by_layer, max_num_layer, num_axiom_map, const_num_axioms = \
        numeric_axiom_rules.handle_axioms(num_axioms)

    print "Building STRIPS to SAS dictionary..."
    ranges, strips_to_sas, module_effects_to_sas = strips_to_sas_dictionary(groups, num_axioms, num_axiom_map, num_fluents, modules)
    print "Translating task..."
    sas_task = translate_task(strips_to_sas, module_effects_to_sas, ranges, task.init, goal_list,
                              actions, durative_actions, axioms, num_axioms,
                              num_axioms_by_layer, max_num_layer, num_axiom_map,
                              const_num_axioms, modules, task.module_inits, task.subplan_generators, init_constant_predicates, init_constant_numerics)

    mutex_key = build_mutex_key(strips_to_sas, mutex_groups)

#    try:
#        simplify.filter_unreachable_propositions(
#            sas_task, mutex_key, translation_key)
#    except simplify.Impossible:
#        return unsolvable_sas_task("Simplified to trivially false goal")

    write_translation_key(strips_to_sas)
    if WRITE_ALL_MUTEXES:
        write_mutex_key(mutex_key)
    return sas_task

def build_mutex_key(strips_to_sas, groups):
    group_keys = []
    for group in groups:
        group_key = []
        for fact in group:
            if strips_to_sas.get(fact):
                for var, val in strips_to_sas[fact]:
                    group_key.append((var, val, str(fact)))
            else:
                print "not in strips_to_sas, left out:", fact
        group_keys.append(group_key)
    return group_keys

def write_translation_key(strips_to_sas):
    var_file = file("variables.groups", "w")
    vars = dict()
    for exp,[(var, val)] in strips_to_sas.iteritems():
        vars.setdefault(var, []).append((val, exp))
    for var in range(len(vars)):
        print >> var_file, "var%d" % var
        vals = sorted(vars[var]) 
        for (val, exp) in vals:
            print >> var_file, "   %d: %s" % (val, exp)
        if val != -2:
            print >> var_file, "   %d: <none of those>" % (val + 1)

def write_mutex_key(mutex_key):
    invariants_file = file("all.groups", "w")
    print >> invariants_file, "begin_groups"
    print >> invariants_file, len(mutex_key)
    for group in mutex_key:
        #print map(str, group)
        no_facts = len(group)
        print >> invariants_file, "group"
        print >> invariants_file, no_facts
        for var, val, fact in group:
            #print fact
            assert str(fact).startswith("Atom ")
            predicate = str(fact)[5:].split("(")[0]
            #print predicate
            rest = str(fact).split("(")[1]
            rest = rest.strip(")").strip()
            if not rest == "":
                #print "there are args" , rest
                args = rest.split(",")
            else:
                args = []
            print_line = "%d %d %s %d " % (var, val, predicate, len(args))
            for arg in args:
                print_line += str(arg).strip() + " "
            #print fact
            #print print_line
            print >> invariants_file, print_line
    print >> invariants_file, "end_groups"
    invariants_file.close()


if __name__ == "__main__":
    import pddl
    sys.stdout = sys.__stderr__
    print "Parsing..."
    task = pddl.open()
    if task.domain_name in ["protocol", "rover"]:
        # This is, of course, a HACK HACK HACK!
        # The real issue is that ALLOW_CONFLICTING_EFFECTS = True
        # is actually the correct semantics, but then we don't get to filter
        # out operators that are impossible to apply due to mutexes between
        # different SAS+ variables. For example,
        # ALLOW_CONFLICTING_EFFECTS = True does not filter on(a,a) in
        # blocksworld/4-0.
        ALLOW_CONFLICTING_EFFECTS = True

    # EXPERIMENTAL!
    # import psyco
    # psyco.full()

    sas_task = pddl_to_sas(task)
    print "Writing output..."
    #sas_task.output(file("output.sas", "w"))
    sas_task.output(sys.__stdout__)
    print "Done!"
