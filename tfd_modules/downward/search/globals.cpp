#include "globals.h"

#include <sstream>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <cassert>
#include <cmath>
using namespace std;
using namespace modules;

#include "axioms.h"
//#include "causal_graph.h"
#include "domain_transition_graph.h"
#include "operator.h"
#include "state.h"
#include "successor_generator.h"
#include "plannerParameters.h"

static const bool s_OutputPredMappings = false;

void PlanStep::dump() const
{
    cout << start_time << ": " << op->get_name() << "[" << duration << "]"
        << endl;
}

bool compareContext(modules::plannerContextPtr p1,
        modules::plannerContextPtr p2)
{
    if(p1 == NULL || p2 == NULL) {
        cerr << __func__ << " WARNING: Received invalid plannerContextPtr for comparison!" << endl;
        return true;
    }
    pair<const TimeStampedState*, const Operator*>* pair1 = static_cast<pair<
        const TimeStampedState*, const Operator*> *> (p1);
    pair<const TimeStampedState*, const Operator*>* pair2 = static_cast<pair<
        const TimeStampedState*, const Operator*> *> (p2);
    if (pair1->first == NULL || pair2->first == NULL)
        return true;
    if (pair1->second == NULL || pair2->second == NULL)
        return true;

    //FIXME: better to change to compared values, instead of pointers?
    if (pair1->first < pair2->first)
        return true;
    else if (pair1->first > pair2->first)
        return false;
    if (pair1->second < pair2->second)
        return true;
    else if (pair1->second > pair2->second)
        return false;
    assert(pair1->first == pair2->first && pair1->second == pair2->second);
    return false;
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

bool double_equals(double a, double b)
{
    return std::abs(a - b) < EPSILON;
}

void check_magic(istream &in, string magic)
{
    string word;
    in >> word;
    if(word != magic) {
        cout << "Failed to match magic word '" << magic << "'." << endl;
        cout << "Got '" << word << "'." << endl;
        exit(1);
    }
}

void read_variables(istream &in)
{
    check_magic(in, "begin_variables");
    int count;
    in >> count;
    for(int i = 0; i < count; i++) {
        string name;
        in >> name;
        g_variable_name.push_back(name);
        int range;
        in >> range;
        g_variable_domain.push_back(range);
        int layer;
        in >> layer;
        g_axiom_layers.push_back(layer);
        //identify variable type
        if(range == -3) {
            g_variable_types.push_back(costmodule);
        } else if (range == -2) {
            g_variable_types.push_back(module);
        } else if (range != -1) {
            g_variable_types.push_back(logical);
            //changes to comparison if a comparison axiom is detected
        } else {
            g_variable_types.push_back(primitive_functional);
            //changes to subterm_functional if a numeric axiom is detected
        }
    }
    check_magic(in, "end_variables");
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

void read_goal(istream &in)
{
    check_magic(in, "begin_goal");
    int count;
    in >> count;
    for(int i = 0; i < count; i++) {
        int var;
        double val;
        in >> var >> val;
        g_goal.push_back(make_pair(var, val));
    }
    check_magic(in, "end_goal");
}

void dump_goal()
{
    cout << "Goal Conditions:" << endl;
    for(int i = 0; i < g_goal.size(); i++)
        cout << "  " << g_variable_name[g_goal[i].first] << ": "
            << g_goal[i].second << endl;
}

void read_operators(istream &in)
{
    int count;
    in >> count;
    for(int i = 0; i < count; i++)
        g_operators.push_back(Operator(in));
}

void read_logic_axioms(istream &in)
{
    int count;
    in >> count;
    for(int i = 0; i < count; i++) {
        LogicAxiom *ax = new LogicAxiom(in);
        g_axioms.push_back(ax);
    }
}

void read_numeric_axioms(istream &in)
{
    int count;
    in >> count;
    for(int i = 0; i < count; i++) {
        NumericAxiom *ax = new NumericAxiom(in);
        g_axioms.push_back(ax);
        // ax->dump();
    }
}

void evaluate_axioms_in_init()
{
    g_axiom_evaluator = new AxiomEvaluator;
    g_axiom_evaluator->evaluate(*g_initial_state);
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

void read_everything(istream &in)
{
    read_variables(in);
    read_pddl_translation(in);
    read_constant_facts(in);
    read_modules(in);

    g_initial_state = new TimeStampedState(in);
    //g_initial_state->dump();
    read_goal(in);
    read_operators(in);
    read_logic_axioms(in);
    read_numeric_axioms(in);
    evaluate_axioms_in_init();
    check_magic(in, "begin_SG");
    g_successor_generator = read_successor_generator(in);
    check_magic(in, "end_SG");
    g_causal_graph = new CausalGraph(in);
    DomainTransitionGraph::read_all(in);
}

void dump_everything()
{
    cout << "Variables (" << g_variable_name.size() << "):" << endl;
    for(int i = 0; i < g_variable_name.size(); i++)
        cout << "  " << g_variable_name[i] << " (range "
            << g_variable_domain[i] << ")" << endl;
    cout << "Initial State:" << endl;
    g_initial_state->dump(true);
    dump_goal();
    cout << "Successor Generator:" << endl;
    g_successor_generator->dump();
    for(int i = 0; i < g_variable_domain.size(); i++)
        g_transition_graphs[i]->dump();
}

void dump_DTGs()
{
    for(int i = 0; i < g_variable_domain.size(); i++) {
        cout << "DTG of variable " << i;
        g_transition_graphs[i]->dump();
    }
}

const TimeStampedState* g_modulecallback_state = NULL;

int g_last_arithmetic_axiom_layer;
int g_comparison_axiom_layer;
int g_first_logic_axiom_layer;
int g_last_logic_axiom_layer;
vector<string> g_variable_name;
vector<int> g_variable_domain;
vector<int> g_axiom_layers;
vector<double> g_default_axiom_values;
vector<variable_type> g_variable_types;
TimeStampedState *g_initial_state;
vector<pair<int, double> > g_goal;
vector<Operator> g_operators;
vector<Axiom*> g_axioms;
AxiomEvaluator *g_axiom_evaluator;
SuccessorGenerator *g_successor_generator;
vector<DomainTransitionGraph *> g_transition_graphs;
CausalGraph *g_causal_graph;

PlannerParameters g_parameters;

Operator *g_let_time_pass;
Operator *g_wait_operator;

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

istream& operator>>(istream &is, assignment_op &aop)
{
    string strVal;
    is >> strVal;
    if(!strVal.compare("="))
        aop = assign;
    else if(!strVal.compare("+"))
        aop = increase;
    else if(!strVal.compare("-"))
        aop = decrease;
    else if(!strVal.compare("*"))
        aop = scale_up;
    else if(!strVal.compare("/"))
        aop = scale_down;
    else {
        cout << "SEVERE ERROR: expected assignment operator, read in " << strVal << endl;
        assert(false);
    }
    return is;
}

ostream& operator<<(ostream &os, const assignment_op &aop)
{
    switch (aop) {
        case assign:
            os << ":=";
            break;
        case scale_up:
            os << "*=";
            break;
        case scale_down:
            os << "/=";
            break;
        case increase:
            os << "+=";
            break;
        case decrease:
            os << "-=";
            break;
        default:
            cout << "Error: aop has value " << (int)aop << endl;

            assert(false);
            break;
    }
    return os;
}

istream& operator>>(istream &is, binary_op &bop)
{
    string strVal;
    is >> strVal;
    if(!strVal.compare("+"))
        bop = add;
    else if(!strVal.compare("-"))
        bop = subtract;
    else if(!strVal.compare("*"))
        bop = mult;
    else if(!strVal.compare("/"))
        bop = divis;
    else if(!strVal.compare("<"))
        bop = lt;
    else if(!strVal.compare("<="))
        bop = le;
    else if(!strVal.compare("="))
        bop = eq;
    else if(!strVal.compare(">="))
        bop = ge;
    else if(!strVal.compare(">"))
        bop = gt;
    else if(!strVal.compare("!="))
        bop = ue;
    else {
        cout << strVal << " was read" << endl;
        assert(false);
    }
    return is;
}

ostream& operator<<(ostream &os, const binary_op &bop)
{
    switch (bop) {
        case mult:
            os << "*";
            break;
        case divis:
            os << "/";
            break;
        case add:
            os << "+";
            break;
        case subtract:
            os << "-";
            break;
        case lt:
            os << "<";
            break;
        case le:
            os << "<=";
            break;
        case eq:
            os << "=";
            break;
        case ge:
            os << ">=";
            break;
        case gt:
            os << ">";
            break;
        case ue:
            os << "!=";
            break;
        default:
            assert(false);
            break;
    }
    return os;
}

istream& operator>>(istream &is, trans_type &tt)
{
    string strVal;
    is >> strVal;
    if(!strVal.compare("s"))
        tt = start;
    else if(!strVal.compare("e"))
        tt = end;
    else if(!strVal.compare("c"))
        tt = compressed;
    else if(!strVal.compare("a"))
        tt = ax;
    else {
        cout << strVal << " was read." << endl;
        assert(false);
    }
    return is;
}

ostream& operator<<(ostream &os, const trans_type &tt)
{
    switch (tt) {
        case start:
            os << "s";
            break;
        case end:
            os << "e";
            break;
        case compressed:
            os << "c";
            break;
        case ax:
            os << "a";
        default:
            // cout << "Error: Encountered binary operator " << bop << "." << endl;
            assert(false);
            break;
    }
    return os;
}

istream& operator>>(istream &is, condition_type &ct)
{
    string strVal;
    is >> strVal;
    if(!strVal.compare("s"))
        ct = start_cond;
    else if(!strVal.compare("o"))
        ct = overall_cond;
    else if(!strVal.compare("e"))
        ct = end_cond;
    else if(!strVal.compare("a"))
        ct = ax_cond;
    else
        assert(false);
    return is;
}

ostream& operator<<(ostream &os, const condition_type &ct)
{
    switch (ct) {
        case start_cond:
            os << "s";
            break;
        case overall_cond:
            os << "o";
            break;
        case end_cond:
            os << "e";
            break;
        case ax_cond:
            os << "a";
            break;
        default:
            assert(false);
    }
    return os;
}

void printSet(const set<int> s)
{
    set<int>::const_iterator it;
    for(it = s.begin(); it != s.end(); ++it)
        cout << *it << ",";
    cout << endl;
}
