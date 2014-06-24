/**
 * API for the PDDL external module extension.
 */

#ifndef _PDDL_MODULE_TYPES_H_
#define _PDDL_MODULE_TYPES_H_

#define PDDL_MODULE_VERSION_MAJOR 0
#define PDDL_MODULE_VERSION_MINOR 4
#define PDDL_MODULE_VERSION_STRING "0.4"

#include <math.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <algorithm>
#include <utility>
#include <tr1/unordered_map>
using std::string;
using std::vector;
using std::map;

namespace modules
{
// callback interface typedefs
typedef std::pair<int, int> VarVal;
typedef std::tr1::unordered_map<string, VarVal> PredicateMapping;
typedef std::tr1::unordered_map<string, int> FunctionMapping;
//typedef std::vector<std::pair<std::string, std::string> > ObjectStringList;
typedef std::map<std::string, std::string> ObjectTypeMap;


const double INFINITE_COST = HUGE_VAL;

static string toLower(const string & s)
{
   string ret;
#ifdef _WIN32
   std::transform(s.begin(), s.end(), back_inserter(ret), (int(*)(int)) tolower);
#else
   std::transform(s.begin(), s.end(), back_inserter(ret), (int(*)(int)) std::tolower);
#endif
   return ret;
}

/// A parameter is an object in pddl e.g. block1 of type Block
class Parameter {
   public:
      Parameter(string n, string t, string v = "") {
         if(n.find("?") == 0) // strip leading '?'
            n = n.substr(1);
         name = toLower(n);
         type = toLower(t);
         value = toLower(v);
      }

      string name;    ///< ?client (w/o '?')
      string type;    ///< movable
      string value;   ///< block1
};

typedef vector<Parameter> ParameterList;

class Predicate {
   public:
      Predicate(string n, ParameterList pl, bool v = true) : parameters(pl), value(v) { 
         name = toLower(n);
      }

      string name;
      ParameterList parameters;
      bool value;
};

typedef vector<Predicate> PredicateList;

class NumericalFluent {
   public:
      NumericalFluent(string n, ParameterList pl, double v = 0.0) : parameters(pl), value(v) { 
         name = toLower(n);
      }

      string name;
      ParameterList parameters;
      double value;
};

typedef vector<NumericalFluent> NumericalFluentList;

class RawAction {
    public:
        RawAction(string name, ParameterList pl, double start_time, double duration) :
            name(name), parameters(pl), start_time(start_time), duration(duration) {
            name = toLower(name);
        }

        string name;                ///< the name of the schematic operator
        ParameterList parameters;   ///< the parameters of the grounded operator
        double start_time;
        double duration;
};
typedef vector<RawAction> RawPlan;

/**
 * Callback method which can be used by the module to access the current planning
 * situation in the planner.
 *
 * The planner should implement this method and fill the PredicateList with the 
 * predicates corresponding to the current situation.
 *
 * \param [in,out] predicateList list of predicates, whose values should be filled. Creates full list, if NULL.
 * \return true, if OK.
 */
typedef bool (*predicateCallbackType)(PredicateList* & predicateList);

/** 
 * \param [in,out] numericalFluentList list of fluents, whose values should be filled. Creates full list, if NULL.
 * \return true, if OK.
 */
typedef bool (*numericalFluentCallbackType)(NumericalFluentList* & numericalFluentList);

/**** Begin Actual Module Calls ****/

/// Function pointer to call for a module before any module calls are performed - parameters are passed on from the problem definition file.
typedef void (*moduleInitType)(int argc, char** argv);

/// Function pointer that is called when the planner produces a plan - used for cleanup and writing out computed information to be stored/processed.
/**
 * \param [in] plan the best generated plan. Might be empty if no plan was created.
 * \param [in] argc, argv the parameters as passed on from the problem definition file.
 *              The first parameter will be the module call, the second one the plan number,
 *              following are the ones from the problem file.
 * \param [in] predicateCallback, numericalFluentCallback callback functions that allow to reconstruct the init state
 */
typedef void (*moduleExitType)(const RawPlan & plan, int argc, char** argv,
        predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback);

/// Module for grounding partially grounded operators.
/**
 * FIXME Does relaxed make sense ???
 *
 * \param [in] statePtr pointer that exactly identified the state that is grounded.
 *      Different states that are equal might be expanded, and in that case should
 *      generate the same successors again.
 * \returns a new ground object as an additional operator parameter that was not created
 * by this module before or empty string if it is not possible to ground the operator (any more).
 */
typedef std::string (*groundingModuleType)(const ParameterList & parameterList,
        predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback,
        int relaxed, const void* statePtr);
 
/// Semantic attachment for a condition (predicate).
/**
 * \param [in] relaxed only produce relaxed solution, 0 - produce accurate result, 1 .. n produce an approximate result using method 1 .. n.
 * \return the actual cost or INFINITE_COST when called as cost module, INFINITE_COST if false or 0 (a value smaller INFINITE_COST) if true, when called as conditionChecker 
 *    (i.e. cost modules can be used as condition checkers)
 */
typedef double (*conditionCheckerType)(const ParameterList & parameterList,
        predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback,
        int relaxed);

/// Semantic attachment adding numerical effects.
/**
 * \param [in] relaxed only produce relaxed solution, 0 - produce accurate result, 1 .. n produce an approximate result using method 1 .. n.
 * \param [in, out] writtenVars the variable to be written in the order of the module spec in the domain. The vector is already filled to size.
 * \return If the module returns a value != 0, the writtenVars are applied to the state, otherwise the state is not changed.
 */
typedef int (*applyEffectType)(const ParameterList & parameterList,
        predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback,
        int relaxed, vector<double> & writtenVars);

/**** Interface addition for extracting the plan from a module - irrelevant for the actual planning process  ****/

/// This type should be returned by a module and contain a custom datatype, that contains any information, that might be necessary for actually executing an operator.
typedef void* subplanType;

/// A Module should generate a subplanType for this operator.
/**
 * All parameters will be the same as for previous calls to conditionCheckerType or applyEffectType.
 * The additional operatorName is only given as additional information.
 */
typedef subplanType (*subplanGeneratorType)(const string & operatorName, const ParameterList & parameterList,
        predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int heuristic);
//FIXME: do we need heuristic here?

/// For final plan output: Convert a subplan into a string.
typedef string (*outputSubplanType)(subplanType subplan);

/// The planner might upon plan generation get subplans for all operators in the plan and thus generate a plan for each module.
typedef vector<subplanType> modulePlanType;

/// Execute a plan for a module.
/**
 * This function is incorrect! Subplans should be executed synchronized if multiple modules are present.
 * Fix this, if you need it.
 */
typedef void (*executeModulePlanType)(modulePlanType modulePlan);

std::ostream & operator<<(std::ostream & os, const Parameter & p);
std::ostream & operator<<(std::ostream & os, const ParameterList & pl);
std::ostream & operator<<(std::ostream & os, const Predicate & p);
std::ostream & operator<<(std::ostream & os, const PredicateList & pl);
std::ostream & operator<<(std::ostream & os, const NumericalFluent & n);
std::ostream & operator<<(std::ostream & os, const NumericalFluentList & nl);

} // namespace modules

#define VERIFY_INIT_MODULE_DEF(name) modules::moduleInitType name##_def_check = name
#define VERIFY_EXIT_MODULE_DEF(name) modules::moduleExitType name##_def_check = name
#define VERIFY_GROUNDINGMODULE_DEF(name) modules::groundingModuleType name##_def_check = name
#define VERIFY_CONDITIONCHECKER_DEF(name) modules::conditionCheckerType name##_def_check = name
#define VERIFY_APPLYEFFECT_DEF(name) modules::applyEffectType name##_def_check = name
#define VERIFY_SUBPLANGENERATOR_DEF(name) modules::subplanGeneratorType name##_def_check = name

#endif

