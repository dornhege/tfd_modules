import pddl_types
import functions
import sys
import f_expression
import conditions

class Module(object):
    def __init__(self, name, parameters, type, modulecall, effects = [], parent = None):
      self.name = name
      self.parameters = tuple(parameters)
      self.type = type
      self.modulecall = modulecall
      self.effects = tuple(effects)
      self.parent = parent
      self.hash = hash((self.__class__, self.name, self.parameters, 
                        self.type, self.modulecall, self.effects))
      assert self.type in ("conditionchecker", "effect", "cost", "grounding"), self.type
      assert (self.type == "effect") or (len(self.effects) == 0)
    def __hash__(self):
      return self.hash
    def __eq__(self, other):
      # Compare hash first for speed reasons.
      return (self.hash == other.hash and
              self.__class__ is other.__class__ and
              self.name == other.name and
              self.parameters == other.parameters and
              self.type == other.type and
              self.modulecall == other.modulecall and
              self.effects == other.effects)
    def __ne__(self, other):
      return not self == other
    def __str__(self):
      if self.type == "effect":
        return "%s %s(%s) %s %s -> %s" % (self.__class__.__name__, self.name,
                            ", ".join(map(str, self.parameters)), self.type, self.modulecall,
                            ", ".join(map(str, self.effects)))
      else:
        return "%s %s(%s) %s %s" % (self.__class__.__name__, self.name,
                            ", ".join(map(str, self.parameters)), self.type, self.modulecall)
    def toModuleCall(self):
      mod_args = []
      for param in self.parameters:
        if param.name.startswith("?"):
          mod_args.append(conditions.Variable(param.name))
        else:
          mod_args.append(conditions.ObjectTerm(param.name))
      return conditions.ModuleCall(self.name, mod_args)
    def rename_variables(self, renamings):
        new_params = []
        for param in self.parameters:
          new_var = renamings.get(conditions.Variable(param.name), conditions.Variable(param.name))
          new_param = pddl_types.TypedObject(new_var.name, param.type)
          new_params.append(new_param)
        new_effects = []
        for effect in self.effects:
          new_effect = effect.rename_variables(renamings)
          new_effects.append(new_effect)
        return self.__class__(self.name, new_params, self.type, self.modulecall, new_effects)
    def instantiate(self, var_mapping, new_modules):
        new_params = []
        for param in self.parameters:
          new_var = var_mapping.get(conditions.Variable(param.name), conditions.Variable(param.name))
          new_param = pddl_types.TypedObject(new_var.name, param.type)
          new_params.append(new_param)

        new_effects = []
        for effect in self.effects:
          new_effect = effect.rename_variables(var_mapping)
          new_effects.append(new_effect)
        # ahh need to inst the effects too!
        # -- need vars as args
        # also for effects
        mc = Module(self.name, new_params, self.type, self.modulecall, new_effects, self)
        if mc not in new_modules:
          new_modules.add(mc)
    def parse(alist):
      assert len(alist) >= 3
      name = alist[0]
      # effects might contain (set) functions here like (x ?a)
      parameterList = [someParam for someParam in alist[1:-2] if not isinstance(someParam,list)]
      functionParamList = [someParam for someParam in alist[1:-2] if isinstance(someParam, list)]

      parameters = pddl_types.parse_typed_list(parameterList)
      effects = [f_expression.parse_expression(entry) for entry in functionParamList]
      #effects = [functions.Function.parse_typed(entry, "number") for entry in functionParamList]

      modulecall = alist[-1]
      type = alist[-2]
      return Module(name, parameters, type, modulecall, effects)
    parse = staticmethod(parse)
    def dump(self, indent="  "):
      if self.type == "effect":
        print "%s%s(%s) Effects: (%s) %s [%s]" % (indent, self.name, ", ".join(map(str, self.parameters)), ", ".join(map(str, self.effects)), self.type, self.modulecall)
      else:
          print "%s%s(%s) %s [%s]" % (indent, self.name, ", ".join(map(str, self.parameters)), self.type, self.modulecall)

class SubplanGenerator(object):
  def __init__(self, genFn, outputFn, execFn):
    self.genFn = genFn
    self.outputFn = outputFn
    self.execFn = execFn
  def parse(alist):
    assert(alist[0] == "subplan_generator")
    assert(len(alist) == 4)
    return SubplanGenerator(alist[1], alist[2], alist[3])
  def dump(self, indent="  "):
    print "%sGenerator: %s Output: %s Execute: %s" % (indent, self.genFn, self.outputFn, self.execFn)
  parse = staticmethod(parse)

class ModuleInit(object):
  def __init__(self, init_function, params):
    self.init_function = init_function
    self.parameters = params
  def parse(alist):
    init_function = alist[0]
    params = alist[1:]
    return ModuleInit(init_function, params)
  def dump(self, indent="  "):
    print "%sFunction: %s Parameters: %s" % (indent, self.init_function, " ".join(self.parameters))
  parse = staticmethod(parse)

class ModuleExit(object):
  def __init__(self, exit_function, params):
    self.exit_function = exit_function
    self.parameters = params
  def parse(alist):
    exit_function = alist[0]
    params = alist[1:]
    return ModuleExit(exit_function, params)
  def dump(self, indent="  "):
    print "%sFunction: %s Parameters: %s" % (indent, self.exit_function, " ".join(self.parameters))
  parse = staticmethod(parse)

class OplInit(object):
  def __init__(self, init_function):
    self.init_function = init_function
  def parse(alist):
    init_function = alist
    return OplInit(init_function)
  def dump(self, indent="  "):
    print "%sFunction: %s" % (indent, self.init_function)
  parse = staticmethod(parse)
