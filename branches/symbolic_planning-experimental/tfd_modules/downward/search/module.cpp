#include "module.h"
#include <cassert>
#include <iostream>
#include <sstream>
#include "state.h"
#include "globals.h"
#include "operator.h"

static const bool s_OutputPredMappings = false;

Module::Module(istream &in)
{
    string function_name, lib;
    in >> function_name >> lib;
    libCall = function_name;
    libCall.append("@").append(lib);

    int count;
    in >> count;
    for (int i = 0; i < count; ++i) {
        string paramname, type, value;
        in >> paramname >> type >> value;
        params.push_back(Parameter(paramname, type, value));
    }
}

void Module::dump()
{
    std::cout << "LibCall: " << libCall.c_str() << std::endl; 
    std::cout << "Params: " << params << std::endl;
}

ConditionModule::ConditionModule(istream &in) :
    Module(in)
{
    in >> var;
    g_variable_types[var] = module;
    checkCondition = g_module_loader->getConditionChecker(libCall);
    if (checkCondition == NULL) {
        printf("Failed to load ConditionModule at \"%s\".", libCall.c_str());
        assert(false);
    }
}

void ConditionModule::dump()
{
    std::cout << "ConditionModule" << std::endl;
    Module::dump();
    std::cout << "Variable: " << var << std::endl;

    std::cout << "checkCondition call is: " << (long)(checkCondition) << std::endl;
}

EffectModule::EffectModule(istream &in) :
    Module(in)
{
    in >> internal_name;
    int count;
    in >> count;
    writtenVars.reserve(count);
    int var_no;
    for (int i = 0; i < count; ++i) {
        in >> var_no;
        writtenVars.push_back(var_no);
    }
    applyEffect = g_module_loader->getApplyEffect(libCall);
    if (applyEffect == NULL) {
        printf("Failed to load EffectModule at \"%s\".", libCall.c_str());
        assert(false);
    }
}

void EffectModule::dump()
{
    std::cout << "EffectModule" << std::endl;
    Module::dump();
    std::cout << "Internal name: " << internal_name << std::endl;
    std::cout << "Written vars: ";
    for(vector<int>::iterator it = writtenVars.begin(); it != writtenVars.end(); it++)
        std::cout << *it << " ";
    std::cout << std::endl;

    std::cout << "ApplyEffect call is: " << (long)(applyEffect) << std::endl;
}

CostModule::CostModule(istream &in) :
    Module(in)
{
    in >> var;
    assert(g_variable_types[var] == costmodule);
    checkCost = g_module_loader->getCostChecker(libCall);
    if (checkCost == NULL) {
        printf("Failed to load CostModule at \"%s\".", libCall.c_str());
        assert(false);
    }
}

void CostModule::dump()
{
    std::cout << "CostModule" << std::endl;
    Module::dump();
    std::cout << "Variable: " << var << std::endl;

    std::cout << "checkCost call is: " << (long)(checkCost) << std::endl;
}

InitModule::InitModule(istream &in)
{
    string function_name, lib;
    in >> function_name >> lib;
    libCall = function_name;
    libCall.append("@").append(lib);

    int count;
    in >> count;
    string param;
    for (int i = 0; i < count; ++i) {
        in >> param;
        parameters.push_back(param);
    }

    initModule = g_module_loader->getModuleInit(libCall);
    if (initModule == NULL) {
        printf("Failed to load InitModule at \"%s\".", libCall.c_str());
        assert(false);
    }
}

void InitModule::execInit()
{
    int argc = parameters.size() + 1;
    char** argv = new char*[argc];

    argv[0] = strdup(libCall.c_str());
    for (int i = 0; i < parameters.size(); i++) {
        argv[i + 1] = strdup(parameters.at(i).c_str());
    }
    initModule(argc, argv);
    for (int i = 0; i < argc; i++) {
        free(argv[i]);
    }
    delete[] argv;
}

Module::~Module()
{
}

// global definitions

const TimeStampedState* g_modulecallback_state = NULL;

map<int, ConditionModule *> g_condition_modules;
vector<EffectModule *> g_effect_modules;
map<int, CostModule*> g_cost_modules;
vector<InitModule *> g_init_modules;
vector<SubplanModuleSet> g_subplan_modules;

PredicateMapping g_pred_mapping;
FunctionMapping g_func_mapping;

PredicateList g_pred_constants;
NumericalFluentList g_func_constants;

PDDLModuleLoader *g_module_loader;

// global interface functions

void dump_modules()
{
    for(map<int, ConditionModule*>::iterator it =  g_condition_modules.begin();
            it != g_condition_modules.end(); it++)
        it->second->dump();
    for(vector<EffectModule *>::iterator it =  g_effect_modules.begin(); it != g_effect_modules.end(); it++)
        (*it)->dump();
    for(map<int, CostModule*>::iterator it =  g_cost_modules.begin(); it != g_cost_modules.end(); it++)
        it->second->dump();
}

bool getPreds(PredicateList* & predicateList)
{
    assert(g_modulecallback_state);
    if (g_modulecallback_state == NULL)
        return false;

    //cout << "Predicate callback!" << endl;

    if (predicateList == NULL) {
        //printf("creating PL\n");
        predicateList = new PredicateList();
        // fill predicate list with subs of all existing predicates
        for (PredicateMapping::iterator it = g_pred_mapping.begin(); it
                != g_pred_mapping.end(); it++) {
            string pred = it->first;
            int var = it->second.first;
            int val = it->second.second;

            std::string token;
            std::istringstream iss(pred);
            string pName;
            ParameterList params;
            while (getline(iss, token, ' ')) {
                if (pName.empty()) {
                    pName = token;
                    continue;
                }
                params.push_back(Parameter("", "", token));
                //   std::cout << token << std::endl;
            }
            Predicate p(pName, params);
            p.value = ((*g_modulecallback_state)[var] == val);
            predicateList->push_back(p);

            //cout << "PREDMAP: \"" << pred << "\" var " << var << " val " << val << endl;
        }
        // add all constants to the predicate list. NOTE: only those predicates that are true in the initial state are added, because
        // constants are NOT grounded during preprocess!!
        predicateList->insert(predicateList->end(), g_pred_constants.begin(),
                g_pred_constants.end());

        return true;
    }

    string key;

    PredicateList::iterator it;
    for (it = predicateList->begin(); it != predicateList->end(); ++it) {
        key = it->name;
        for (ParameterList::const_iterator jt = it->parameters.begin(); jt
                != it->parameters.end(); ++jt) {
            key.append(" ").append(jt->value);
        }
        PredicateMapping::iterator entry = g_pred_mapping.find(key);
        VarVal res;
        if (entry == g_pred_mapping.end()) {
            // check whether the predicate is a constant. NOTE: it's possible the predicate is NOT found because constants
            // are not grounded during preprocess and only those constants exist that are true in the initial state. If the predicate is
            // not found, it is also possible that the predicates name is missspelled, but this function returns the predicate as false
            // HACK!!!
            bool predFound = false;
            for (PredicateList::iterator cit = g_pred_constants.begin(); cit
                    != g_pred_constants.end(); cit++) {
                if (cit->name == it->name) {
                    assert(cit->parameters.size() == it->parameters.size());
                    bool paramsFit = true;
                    for (unsigned int i = 0; i < cit->parameters.size(); i++) {
                        if (cit->parameters[i].value != it->parameters[i].value) {
                            paramsFit = false;
                            break;
                        }
                    }
                    if (paramsFit) {
                        it->value = true;
                        predFound = true;
                        break;
                    }
                }
            }
            if (!predFound)
                it->value = false;
            //cout << "Error! Could not find entry in predicate mapping with key: " << key;
            // exit(1);
        } else {
            res = entry->second;
            it->value = (res.second == (*g_modulecallback_state)[res.first]);
        }
        //it->value = (res.second == (*g_modulecallback_state)[res.first]);
    }

    return true;
}

bool getFuncs(NumericalFluentList* & fluentList)
{
    assert(g_modulecallback_state);
    if (g_modulecallback_state == NULL)
        return false;
    //cout << "Function callback!" << endl;

    if (fluentList == NULL) {
        fluentList = new NumericalFluentList();
        for (FunctionMapping::iterator it = g_func_mapping.begin(); it
                != g_func_mapping.end(); it++) {
            string pred = it->first;
            int var = it->second;

            std::string token;
            std::istringstream iss(pred);
            string pName;
            ParameterList params;
            while (getline(iss, token, ' ')) {
                if (pName.empty()) {
                    pName = token;
                    continue;
                }
                params.push_back(Parameter("", "", token));
                //   std::cout << token << std::endl;
            }
            NumericalFluent f(pName, params);
            f.value = (*(g_modulecallback_state))[var];
            fluentList->push_back(f);
        }
        fluentList->insert(fluentList->end(), g_func_constants.begin(),
                g_func_constants.end());
        return true;
    }

    string key;

    NumericalFluentList::iterator it;
    for (it = fluentList->begin(); it != fluentList->end(); ++it) {
        key = it->name;
        for (ParameterList::const_iterator jt = it->parameters.begin(); jt
                != it->parameters.end(); ++jt)
            key.append(" ").append(jt->value);

        FunctionMapping::iterator entry = g_func_mapping.find(key);
        int res;
        if (entry == g_func_mapping.end()) {

            bool funcFound = false;
            for (NumericalFluentList::iterator cit = g_func_constants.begin(); cit
                    != g_func_constants.end(); cit++) {
                if (cit->name == it->name) {
                    assert(cit->parameters.size() == it->parameters.size());
                    bool paramsFit = true;
                    for (unsigned int i = 0; i < cit->parameters.size(); i++) {
                        if (cit->parameters[i].value != it->parameters[i].value) {
                            paramsFit = false;
                            break;
                        }
                    }
                    if (paramsFit) {
                        it->value = cit->value;
                        funcFound = true;
                        break;
                    }
                }
            }
            if (!funcFound) {
                cout
                    << "Error! Could not find entry in func mapping with key: "
                    << key;
                exit(1);
            }
        } else {
            res = entry->second;
            it->value = (*(g_modulecallback_state))[res];
        }
    }
    return true;
}

void handleSubplans(const vector<PlanStep> & plan)
{
    for (vector<SubplanModuleSet>::iterator it = g_subplan_modules.begin(); it
            != g_subplan_modules.end(); it++) {
        SubplanModuleSet sm = *it;
        Module* generateSp = std::tr1::get<0>(sm);
        Module* outputSp = std::tr1::get<1>(sm);
        Module* execSubplan = std::tr1::get<2>(sm);

        subplanGeneratorType genFn = g_module_loader->getSubplanGenerator(
                generateSp->libCall);
        outputSubplanType outputFn = g_module_loader->getOutputSubplan(
                outputSp->libCall);
        executeModulePlanType execFn = g_module_loader->getExecuteModulePlan(
                execSubplan->libCall);
        if (genFn == NULL || outputFn == NULL || execFn == NULL) {
            cerr << "Error in loading subplan generators." << endl;
            continue;
        }

        // make this temporal and another, read: better interface for outputting plans
        modulePlanType subplan;
        stringstream ss;
        for (int i = 0; i < plan.size(); i++) {
            const PlanStep& step = plan[i];
            ParameterList pl;
            subplanType spt = genFn(step.op->get_name(), pl, NULL, NULL, 0);
            ss << outputFn(spt) << endl << endl;
            subplan.push_back(spt);
        }
        cout << "SubplanPlan:" << endl;
        cout << ss.str() << endl;
        execFn(subplan);
    }
}

string read_name_and_params(istream &in)
{
    string ret, param;
    int paramcount;
    in >> ret >> paramcount;
    for (int i = 0; i < paramcount; ++i) {
        in >> param;
        ret.append(" " + param);
    }
    return ret;
}

void read_pddl_translation(istream &in)
{
    check_magic(in, "begin_pddl_translation");
    int count;
    string name;
    int var, value;
    in >> count;
    for (int i = 0; i < count; ++i) {
        name = read_name_and_params(in);
        in >> var >> value;
        g_pred_mapping.insert(make_pair(name, make_pair(var, value)));
    }
    in >> count;
    for (int i = 0; i < count; ++i) {
        name = read_name_and_params(in);
        in >> var;
        g_func_mapping.insert(make_pair(name, var));
    }
    check_magic(in, "end_pddl_translation");
    if(s_OutputPredMappings) {
        cout << "DEBUG: g_pred_mapping contains:" << endl;
        PredicateMapping::iterator it;
        for (it = g_pred_mapping.begin(); it != g_pred_mapping.end(); ++it) {
            cout << "key: " << it->first << ", var: " << it->second.first
                << ", val: " << it->second.second << endl;
        }
        cout << "DEBUG: g_func_mapping contains:" << endl;
        FunctionMapping::iterator it2;
        for (it2 = g_func_mapping.begin(); it2 != g_func_mapping.end(); ++it2) {
            cout << "key: " << it2->first << ", var: " << it2->second << endl;
        }
    }
}

void read_modules(istream &in)
{
    check_magic(in, "begin_modules");
    int count;
    in >> count;
    for (int i = 0; i < count; ++i) {
        InitModule *init_module = new InitModule(in);
        g_init_modules.push_back(init_module);
    }
    in >> count;
    for (int i = 0; i < count; ++i) {
        string gen, output, exec;
        string lib1, lib2, lib3;
        in >> gen >> lib1 >> output >> lib2 >> exec >> lib3;
        g_subplan_modules.push_back(tr1::make_tuple(new Module(
                        gen.append("@").append(lib1)), new Module(
                        output.append("@").append(lib2)), new Module(
                        exec.append("@").append(lib3))));
    }

    in >> count;
    for (int i = 0; i < count; ++i) {
        ConditionModule *cond_module = new ConditionModule(in);
        g_condition_modules[cond_module->var] = cond_module;
    }
    in >> count;
    for (int i = 0; i < count; ++i) {
        EffectModule *eff_module = new EffectModule(in);
        g_effect_modules.push_back(eff_module);
    }
    in >> count;
    for (int i = 0; i < count; i++) {
        CostModule *cost_module = new CostModule(in);
        g_cost_modules[cost_module->var] = cost_module;
    }

    check_magic(in, "end_modules");
}

void read_constant_facts(istream& in)
{
    check_magic(in, "begin_constant_facts");

    int count;
    in >> count;
    for (int i = 0; i < count; i++) {
        string name;
        in >> name;

        int paramCount;
        in >> paramCount;

        ParameterList params;

        for (int j = 0; j < paramCount; j++) {
            string param;
            in >> param;
            params.push_back(Parameter("", "", param));
        }
        g_pred_constants.push_back(Predicate(name, params, true));
    }

    in >> count;
    for (int i = 0; i < count; i++) {
        string name;
        in >> name;

        int paramCount;
        in >> paramCount;

        ParameterList params;

        for (int j = 0; j < paramCount; j++) {
            string param;
            in >> param;
            params.push_back(Parameter("", "", param));
        }
        double value;
        in >> value;

        g_func_constants.push_back(NumericalFluent(name, params, value));
    }

    check_magic(in, "end_constant_facts");
}

