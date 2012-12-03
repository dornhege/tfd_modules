import itertools

import actions
import axioms
import conditions
import effects
import f_expression
import functions
import modules
import predicates
import pddl_types


class Task(object):
  FUNCTION_SYMBOLS = dict()
  CONSTANT_MAPPING = dict() # yes, this is a hack and should be removed

  def __init__(self, domain_name, task_name, requirements, oplinit,
               types, objects, modules, predicates, init, goal, actions, durative_actions, axioms, function_symbols, subplan_generators, module_inits, module_exits):
    self.domain_name = domain_name
    self.task_name = task_name
    self.requirements = requirements
    self.oplinit = oplinit
    self.types = types
    self.objects = objects
    self.modules = modules
    self.predicates = predicates
    self.init = init
    self.goal = goal
    self.actions = actions
    self.durative_actions = durative_actions
    self.axioms = axioms
    self.axiom_counter = 0
    self.function_symbols = function_symbols
    self.function_administrator = DerivedFunctionAdministrator()
    self.subplan_generators = subplan_generators
    self.module_inits = module_inits
    self.module_exits = module_exits

  def add_axiom(self, parameters, condition):
    name = "new-axiom@%d" % self.axiom_counter
    self.axiom_counter += 1
    axiom = axioms.Axiom(name, parameters, condition)
    self.predicates.append(predicates.Predicate(name, parameters))
    self.axioms.append(axiom)
    return axiom

  def parse(domain_pddl, task_pddl):
    domain_name, requirements, oplinit, constants, predicates, types, functions, actions, durative_actions, axioms, modules, subplan_generators \
                 = parse_domain(domain_pddl)
    task_name, task_domain_name, module_inits, module_exits, objects, init, goal = parse_task(task_pddl)

    assert domain_name == task_domain_name
    objects = constants + objects
    init += [conditions.Atom("=", (conditions.parse_term(obj.name), conditions.parse_term(obj.name))) 
             for obj in objects]
    return Task(domain_name, task_name, requirements, oplinit, types, objects, modules,
                predicates, init, goal, actions, durative_actions, axioms, Task.FUNCTION_SYMBOLS, subplan_generators, module_inits, module_exits)
  parse = staticmethod(parse)

  def dump(self):
    print "Problem %s: %s [%s]" % (self.domain_name, self.task_name,
                                   self.requirements)
    print "OplInit:"
    for init in self.oplinit:
      init.dump()
    print "Types:"
    for type in self.types:
      print "  %s" % type
    print "Objects:"
    for obj in self.objects:
      print "  %s" % obj
    print "Modules:"
    for module in self.modules:
      module.dump()
    print "Module Inits:"
    for mod_init in self.module_inits:
      mod_init.dump()
    print "Module Exits:"
    for mod_exit in self.module_exits:
      mod_exit.dump()
    print "Subplan Generators:"
    for spg in self.subplan_generators:
      spg.dump()
    print "Predicates:"
    for pred in self.predicates:
      print "  %s" % pred
    print "Functions:"
    print "  " + str(self.function_symbols)
    print "Init:"
    for fact in self.init:
      fact.dump()
    print "Goal:"
    self.goal.dump()
    print "Derived Functions:"
    self.function_administrator.dump()
    if self.actions:
        print "Actions:"
        for action in self.actions:
            action.dump()
    if self.durative_actions:
        print "Durative Actions:"
        for action in self.durative_actions:
            action.dump()
    if self.axioms:
      print "Axioms:"
      for axiom in self.axioms:
        axiom.dump()

class Requirements(object):
  def __init__(self, requirements):
    self.requirements = requirements
    for req in requirements:
      assert req in (
        ":strips", ":adl", ":typing", ":negation", ":equality",
        ":negative-preconditions", ":disjunctive-preconditions",
        ":existential-preconditions", ":universal-preconditions",
        ":quantified-preconditions", ":conditional-effects",
        ":fluents", ":object-fluents", ":numeric-fluents", ":action-costs",
        ":durative-actions", ":derived-predicates", ":duration-inequalities", ":modules", ":grounding-modules"), req
  def __str__(self):
    return ", ".join(self.requirements)

class DerivedFunctionAdministrator(object):
    #TODO use hash values?
    def __init__(self):
        self.functions = dict()

    def dump(self,indent = "  "):
        for axiom in self.functions.values():
            axiom.dump(indent)
    def get_all_axioms(self):
        return self.functions.values() 
    def get_derived_function(self,exp):
        def get_default_variables(nr):
            return [conditions.Variable("?v%s" % varnr) for varnr in range(nr)]
        def get_new_symbol(key):   
            # introduce new derived function symbol
            used_names = [axiom.name for axiom in self.functions.values()]
            for counter in itertools.count(1):
                new_func_name = "derived!" + str(counter)
                if new_func_name not in used_names:
                    Task.FUNCTION_SYMBOLS[new_func_name]="number"
                    return new_func_name

        assert isinstance(exp,f_expression.FunctionalExpression)
        if isinstance(exp,f_expression.PrimitiveNumericExpression):
            return exp
        elif isinstance(exp,f_expression.NumericConstant):
            key = (exp.value,)
            if key not in self.functions:
                symbol = get_new_symbol(key)
                self.functions[key] = axioms.NumericAxiom(symbol,[],None,[exp])
            args = ()
        elif isinstance(exp,f_expression.AdditiveInverse):
            subexp = self.get_derived_function(exp.parts[0])
            key = (exp.op, subexp.symbol)
            args = subexp.args
            if key not in self.functions:
                symbol = get_new_symbol(key)
                default_args = get_default_variables(len(subexp.args)) 
                subexp = f_expression.PrimitiveNumericExpression(subexp.symbol, default_args)
                self.functions[key] = axioms.NumericAxiom(symbol, default_args, exp.op, [subexp])
        else:
            assert (isinstance(exp,f_expression.ArithmeticExpression) and
                    len(exp.parts) == 2)
            pne1 = self.get_derived_function(exp.parts[0])
            pne2 = self.get_derived_function(exp.parts[1])
            key = (exp.op, pne1.symbol, pne2.symbol)
            args = pne1.args + pne2.args
            if key not in self.functions:
                if exp.op in ("+","*"):
                    key = (exp.op, pne2.symbol, pne1.symbol)
                    pne1,pne2 = pne2,pne1
                    args = pne1.args + pne2.args
                if key not in self.functions:
                    symbol = get_new_symbol(key)
                    default_args = get_default_variables(len(args))
                    pne1 = f_expression.PrimitiveNumericExpression(pne1.symbol, 
                                                default_args[:len(pne1.args)])
                    if pne2.args:
                        pne2 = f_expression.PrimitiveNumericExpression(pne2.symbol, 
                                                default_args[-len(pne2.args):])
                    
                    self.functions[key] = axioms.NumericAxiom(symbol, tuple(default_args),
                                                              exp.op,[pne1,pne2]) 
        pne_symbol = self.functions[key].get_head().symbol
        return f_expression.PrimitiveNumericExpression(pne_symbol,args)

def parse_domain_structure(entry,the_functions,the_axioms,the_actions,the_durative_actions, 
                           the_types, the_predicates):
    if entry[0] == ":derived":
      axiom = axioms.Axiom.parse(entry)
      the_axioms.append(axiom)
    elif entry[0] == ":durative-action":
      action = actions.DurativeAction.parse(entry)
      the_durative_actions.append(action)
    elif entry[0] == ":functions":
      the_functions.extend(pddl_types.parse_typed_list(entry[1:], 
        constructor=functions.Function.parse_typed, functions=True, types=the_types))
      for function in the_functions:
        Task.FUNCTION_SYMBOLS[function.name] = function.type
        if function.type != "number":
            the_predicates.append(
                predicates.Predicate(conditions.function_predicate_name(function.name),
                                     function.arguments + [pddl_types.TypedObject("?val", function.type)]))
    elif entry[0] == ":action":
      action = actions.Action.parse(entry)
      the_actions.append(action)
    else:
      assert False, "unknown entity"

def parse_domain(domain_pddl):
  iterator = iter(domain_pddl)
  
  the_functions = []
  the_axioms = []
  the_actions = []
  the_durative_actions = []
  the_modules = []
  the_subplan_generators = []

  assert iterator.next() == "define"
  domain_line = iterator.next()
  assert domain_line[0] == "domain" and len(domain_line) == 2
  yield domain_line[1]

  opt_requirements = iterator.next()
  if opt_requirements[0] == ":requirements":
    yield Requirements(opt_requirements[1:])
    oplinit = iterator.next()
  else:
    yield Requirements([":strips"])
    oplinit = opt_requirements

  if oplinit[0] == ":oplinit":
    yield [modules.OplInit.parse(mi) for mi in oplinit[1:]]
    opt_types = iterator.next()
  else:
    yield []  
    opt_types = oplinit  

  the_types = [pddl_types.Type("object")]
  if opt_types[0] == ":types":
    the_types.extend(pddl_types.parse_typed_list(opt_types[1:],
                                                 constructor=pddl_types.Type))
    opt_modules = iterator.next()
  else:
    opt_modules = opt_types

  if opt_modules[0] == ":modules":
    for mod_entry in opt_modules[1:]:
      if mod_entry[0] == "subplan_generator":
        the_subplan_generators.append(modules.SubplanGenerator.parse(mod_entry))
      else:
        the_modules.append(modules.Module.parse(mod_entry))
    opt_constants = iterator.next()
  else:
    opt_constants = opt_modules

  if opt_constants[0] == ":constants":
    yield pddl_types.parse_typed_list(opt_constants[1:],types=the_types)
    pred = iterator.next()
  else:
    yield []
    pred = opt_constants

  the_predicates = []
  if pred[0] == ":predicates":
    the_predicates =  ([predicates.Predicate.parse(entry) for entry in pred[1:]] +
         [predicates.Predicate("=",
                               [pddl_types.TypedObject("?x", "object"),
                                pddl_types.TypedObject("?y", "object")])])
  else:
    the_predicates = [predicates.Predicate("=",
                                [pddl_types.TypedObject("?x", "object"),
                                 pddl_types.TypedObject("?y", "object")])]
    parse_domain_structure(pred,the_functions,the_axioms,the_actions,the_durative_actions,the_types,the_predicates)

  for entry in iterator:
    parse_domain_structure(entry,the_functions,the_axioms,the_actions,the_durative_actions,the_types,the_predicates)

  pddl_types.set_supertypes(the_types)
  the_types = [type for type in the_types if type.supertype_names != [] or type.name == "object"]
  yield the_predicates
  yield the_types
  yield the_functions
  yield the_actions
  yield the_durative_actions
  yield the_axioms
  yield the_modules
  yield the_subplan_generators

def parse_task(task_pddl):
  iterator = iter(task_pddl)

  assert iterator.next() == "define"
  problem_line = iterator.next()
  assert problem_line[0] == "problem" and len(problem_line) == 2
  yield problem_line[1]
  domain_line = iterator.next()
  assert domain_line[0] == ":domain" and len(domain_line) == 2
  yield domain_line[1]

  module_opt = iterator.next()
  if module_opt[0] == ":moduleoptions":
    yield [modules.ModuleInit.parse(mi) for mi in module_opt[1:]]
    module_exit_opt = iterator.next()
  else:
    yield []
    module_exit_opt = module_opt

  if module_exit_opt[0] == ":moduleexitoptions":
    yield [modules.ModuleExit.parse(mi) for mi in module_exit_opt[1:]]
    objects_opt = iterator.next()
  else:
    yield []
    objects_opt = module_exit_opt

  if objects_opt[0] == ":objects":
    yield pddl_types.parse_typed_list(objects_opt[1:])
    init = iterator.next()
  else:
    yield []
    init = objects_opt

  assert init[0] == ":init"
  initial = []
  for fact in init[1:]:
    if fact[0] == "=":
        if conditions.is_function_comparison(fact): # numeric function
            initial.append(f_expression.parse_assignment(fact))
        else: # object function
            function = conditions.parse_term(fact[1])
            terms = function.args
            terms.append(conditions.parse_term(fact[2]))
            atomname = conditions.function_predicate_name(function.name)
            initial.append(conditions.Atom(atomname, terms))
    else:
        initial.append(conditions.Atom(fact[0], [conditions.parse_term(term) for term in fact[1:]]))
  yield initial

  goal = iterator.next()
  assert goal[0] == ":goal" and len(goal) == 2
  yield conditions.parse_condition(goal[1])

  for entry in iterator:
    if entry[0] == ":metric" and entry[1]=="minimize":
        if entry[2][0] in ["total-time", "total-cost"] :
            continue
    assert False, "Can only minimize total-time or total-cost, got: " + str(entry)
