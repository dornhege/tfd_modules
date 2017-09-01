import sys

import pddl_types
import functions
import parser
import actions
import predicates
import conditions
import effects

class TimedInitial(object):
    def __init__(self, time, init_fact):
      self.time = time
      self.fact = init_fact

    def __str__(self):
        return "%s (at %s %s)" % (self.__class__.__name__, self.time, self.fact)

    def dump(self, indent="  "):
        print "%s%s (at %s %s)" % (indent, self.__class__.__name__, self.time, self.fact)

def compile_away(predicates_, actions_, init_, goal_, timed_initials_):
  initial_state_predicate = predicates.Predicate('initial_state', [])
  predicates_.append(initial_state_predicate)
  # add init
  init_.append(conditions.Atom('initial_state', []))
  disable_initial_state = '(at start (not (initial_state)))'
  goal_list = []
  ti_actions = []
  for i, ti in enumerate(timed_initials_):
    name = 'timed_initial_{}__'.format(i)
    # add predicates
    scheduled_predicate = predicates.Predicate(name+'scheduled', [])
    predicates_.append(scheduled_predicate)
    # add goal
    goal_list.append(conditions.Atom(scheduled_predicate.name, []))
    # add action
    #fact_string =  effect_string_from_initial(ti.fact)
    fact_string =  effect_list_from_conjunction(ti.fact)
    action_string = ['''
    (:durative-action {name}action
      :parameters ()
      :duration (= ?duration {time})
      :condition (at start (initial_state))
      :effect (and
        {disable_initial_state}
        (at end ({predicate}))
        {fact}
      )
    )'''.format(name=name, time=ti.time, predicate=scheduled_predicate.name,
                disable_initial_state=disable_initial_state, fact=fact_string)]
    print(action_string[0])
    action = actions.DurativeAction.parse(parser.parse_nested_list(action_string))
    ti_actions.append(action)
    disable_initial_state = ''
  # add goals
  goal_.parts += tuple(goal_list)
  goal_.dump()
  # add negated precontition to all other actions
  [at_start, overall, at_end] = conditions.parse_durative_condition(parser.parse_nested_list(['(at start (not (initial_state)))']))
  for da in actions_:
    da.condition[0].parts += (at_start, )
  actions_.extend(ti_actions)
  
def effect_list_from_conjunction(fact_list):
  facts = []
  fact_template = '(at end {})'
  if type(fact_list) == list and len(fact_list) > 1 and fact_list[0] == 'and':
    for item in fact_list[1:]:
      facts.append('(at end {})'.format(effect_string_from_nested_list(item)))
  else:
    facts.append('(at end {})'.format(effect_string_from_nested_list(fact_list)))
  return '\n        '.join(facts)
  
def effect_string_from_nested_list(fact):
  parts = []
  for item in fact:
    if type(item) == list:
      parts.append(effect_string_from_nested_list(item))
    else:
      parts.append(item)
  return '({})'.format(' '.join(parts))
  