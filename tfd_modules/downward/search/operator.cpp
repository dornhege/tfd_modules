#include "globals.h"
#include "operator.h"
#include "module.h"
#include "best_first_search.h"
#include "plannerParameters.h"
#include "ros_printouts.h"
#include "tfd_modules/opl/stringutil.h"

#include <iostream>
using namespace std;

Prevail::Prevail(istream &in)
{
    in >> var >> prev;
}

bool Prevail::is_applicable(const TimeStampedState &state, const Operator* op, bool allowRelaxed) const
{
    assert(var >= 0 && var < g_variable_name.size());
    assert((prev >= 0 && prev < g_variable_domain[var]) || (g_variable_types[var] == module));
    if(g_variable_types[var] == module) {
        g_setModuleCallbackState(&state);
        predicateCallbackType pct = getPreds;
        numericalFluentCallbackType nct = getFuncs;

        modules::ParameterList parameters = g_condition_modules[var]->params;
        op->addGroundParameters(parameters);
        double cost = g_condition_modules[var]->checkCondition(
                parameters, pct, nct, allowRelaxed);
        return cost < INFINITE_COST;
    } else {
        return state_equals(state[var], prev);
    }
}

PrePost::PrePost(istream &in)
{
    int cond_count;
    in >> cond_count;
    for(int i = 0; i < cond_count; i++)
        cond_start.push_back(Prevail(in));
    in >> cond_count;
    for(int i = 0; i < cond_count; i++)
        cond_overall.push_back(Prevail(in));
    in >> cond_count;
    for(int i = 0; i < cond_count; i++)
        cond_end.push_back(Prevail(in));
    in >> var;
    if(is_functional(var)) {
        in >> fop >> var_post;
        // HACK: just use some arbitrary values for pre and post
        // s.t. they do not remain uninitialized
        pre = post = -1;
    } else {
        in >> pre >> post;
        // HACK: just use some arbitrary values for var_post and fop
        // s.t. they do not remain uninitialized
        var_post = -1;
        fop = assign;
    }
}

ModuleEffect::ModuleEffect(istream &in)
{
    int cond_count;
    in >> cond_count;
    for(int i = 0; i < cond_count; i++)
        cond_start.push_back(Prevail(in));
    in >> cond_count;
    for(int i = 0; i < cond_count; i++)
        cond_overall.push_back(Prevail(in));
    in >> cond_count;
    for(int i = 0; i < cond_count; i++)
        cond_end.push_back(Prevail(in));
    string name;
    in >> name;
    name = name.substr(3);
    module = g_effect_modules[atoi(name.c_str())];
    ROS_ASSERT(module != NULL);
}

ModuleGrounding::ModuleGrounding(std::istream &in)
{
    string name;
    in >> name;
    name = name.substr(3);
    module = g_grounding_modules[atoi(name.c_str())];
    ROS_ASSERT(module != NULL);
}

void ModuleGrounding::dump() const
{
    module->dump();
}

bool PrePost::is_applicable(const TimeStampedState &state) const
{
    assert(var >= 0 && var < g_variable_name.size());
    assert(pre == -1 || (pre >= 0 && pre < g_variable_domain[var]));
    return pre == -1 || (state_equals(state[var], pre));
}

void Operator::sort_prevails(vector<Prevail> &prevails)
{
    int swapIndex = prevails.size() - 1;
    for(int i = 0; i <= swapIndex; ++i) {
        if(g_variable_types[prevails[i].var] == module) {
            std::swap(prevails[i], prevails[swapIndex]);
            i--;
            swapIndex--;
        }
    }
}

Operator::Operator(istream &in) : grounding_parent(NULL)
{
    check_magic(in, "begin_operator");
    in >> ws;
    getline(in, name);
    int groundingCount;
    in >> groundingCount;
    assert(groundingCount >= 0 && groundingCount < 2);  // exactly 0 or 1 grounding module
    for(int i = 0; i < groundingCount; ++i) {
        mod_groundings.push_back(ModuleGrounding(in));
    }
    int count;
    binary_op bop;
    in >> bop >> duration_var;
    if(bop != eq) {
        cout << "Error: The duration constraint must be of the form\n";
        cout << "       (= ?duration (arithmetic_term))" << endl;
        exit(1);
    }

    in >> count; //number of prevail at-start conditions
    for(int i = 0; i < count; i++)
        prevail_start.push_back(Prevail(in));
    in >> count; //number of prevail overall conditions
    for(int i = 0; i < count; i++)
        prevail_overall.push_back(Prevail(in));
    in >> count; //number of prevail at-end conditions
    for(int i = 0; i < count; i++)
        prevail_end.push_back(Prevail(in));
    in >> count; //number of pre_post_start conditions (symbolical)
    for(int i = 0; i < count; i++)
        pre_post_start.push_back(PrePost(in));
    in >> count; //number of pre_post_end conditions (symbolical)
    for(int i = 0; i < count; i++)
        pre_post_end.push_back(PrePost(in));
    in >> count; //number of pre_post_start conditions (functional)
    for(int i = 0; i < count; i++)
        pre_post_start.push_back(PrePost(in));
    in >> count; //number of pre_post_end conditions (functional)
    for(int i = 0; i < count; i++)
        pre_post_end.push_back(PrePost(in));

    // sort prevails such that conditions on module variables come last
    sort_prevails(prevail_start);
    sort_prevails(prevail_overall);
    sort_prevails(prevail_end);

    in >> count; //numer of module start effects
    for(int i = 0; i < count; ++i) {
        mod_effs_start.push_back(ModuleEffect(in));
    }
    in >> count; //number of module end effects
    for(int i = 0; i < count; ++i) {
        mod_effs_end.push_back(ModuleEffect(in));
    }
    check_magic(in, "end_operator");

    numBranches = 0;
}

Operator::Operator(bool uses_concrete_time_information) : grounding_parent(NULL)
{
    prevail_start   = vector<Prevail>();
    prevail_overall = vector<Prevail>();
    prevail_end     = vector<Prevail>();
    pre_post_start  = vector<PrePost>();
    pre_post_end    = vector<PrePost>();
    if(!uses_concrete_time_information) {
        name = "let_time_pass";
        duration_var = -1;
    } else {
        name = "wait";
        duration_var = -2;
    }

    numBranches = 0;
}

bool Operator::isGrounded() const
{
    // if there are no grounding modules left, we are grounded
    return mod_groundings.empty();
}

Operator Operator::ground(const TimeStampedState & state, bool relaxed, bool & ok) const
{
    Operator ret(*this);
    ret.grounding_parent = this;

    if(isGrounded()) {
        ok = false;
        return ret;
    }

    ROS_ASSERT(mod_groundings.size() == 1);

    // perform the actual grounding.
    string name = get_name();
    // name = drive loc1 loc2 -> params: loc1 loc2
    std::vector<string> parts = StringUtil::split(name, " ");
    ParameterList params;
    for(unsigned int i = 1; i < parts.size(); i++) {
        params.push_back(Parameter("", "", parts[i]));
    }

    predicateCallbackType pct = getPreds;
    numericalFluentCallbackType nct = getFuncs;
    g_setModuleCallbackState(&state);

    string groundParam = mod_groundings.front().module->groundingModule(params, pct, nct, relaxed, &state);

    if(groundParam.empty()) {
        ok = false;
        return ret;
    }

    // created a grounded operator from that.
    ok = true;
    numBranches++;
    BranchGroundingCountMap::iterator it = numBranchesByState.find(state);
    if(it == numBranchesByState.end()) {
        numBranchesByState[state] = 1;
    } else {
        it->second++;
    }

    ret.name += " " + groundParam;
    ret.mod_groundings.clear();

    return ret;
}


Operator Operator::groundManually(const std::string& groundParam, bool & ok) const
{
    Operator ret(*this);
    ret.grounding_parent = this;

    if(isGrounded()) {
        ok = false;
        return ret;
    }

    if(groundParam.empty()) {
        ok = false;
        return ret;
    }

    // created a grounded operator from that.
    ok = true;

    ret.name += " " + groundParam;
    ret.mod_groundings.clear();

    return ret;
}

int Operator::getNumBranches(const TimeStampedState* state) const
{
    if(g_parameters.grounding_number_depends_on_state) {
        BranchGroundingCountMap::iterator it = numBranchesByState.find(*state);
        if(it == numBranchesByState.end()) {
            return 0;
        }
        return it->second;
    }
    return numBranches;
}

void Operator::addGroundParameters(modules::ParameterList & parameters) const
{
    if(grounding_parent == NULL)    // this op was never partially grounded -> no need to add params
        return;

    // add grounding param from op name
    std::string name = get_name();     // drive loc1 loc2
    std::vector<string> parts = StringUtil::split(name, " ");
    if(parts.empty()) {
        ROS_ERROR("%s: Bad Operator Name: %s", __func__, name.c_str());
        return;
    } 
    std::vector<string> op_params;
    for(unsigned int i = 1; i < parts.size(); i++) {
        op_params.push_back(parts[i]);
    }

    if(parameters.size() > op_params.size()) {
        ROS_ERROR("%s: more module params (%zu) than op params for op: %s", __func__, parameters.size(),
                name.c_str());
        return;
    }

    // now parameters.size <= op_params.size
    // check all params match until parameters.size and add the additional ones.
    for(unsigned int i = 0; i < op_params.size(); i++) {
        if(i < parameters.size()) {     // already stored in modules parameters
            string mod_param = parameters[i].value;
            if(op_params[i] != mod_param) {
                ROS_ERROR("%s: mismatched module parameter %s at %d for op %s",
                        __func__, mod_param.c_str(), i, name.c_str());
            }
        } else {
            parameters.push_back(modules::Parameter("?grounding", "groundingobject", op_params[i]));
        }
    }
}


void Prevail::dump() const
{
    cout << g_variable_name[var] << ": " << prev << endl;
}

void PrePost::dump() const
{
    cout << "var: " << g_variable_name[var] << ", pre: " << pre
        << " , var_post: " << var_post << ", post: " << post << endl;
}

void Operator::dump() const
{
    cout << name << endl;
    cout << "Grounding: ";
    if(mod_groundings.empty()) {
        cout << "Grounded." << endl;
    } else {
        cout << endl;
        for(int i = 0; i < mod_groundings.size(); i++) {
            mod_groundings[i].dump();
        }
    }
    cout << "Prevails start:" << endl;
    for(int i = 0; i < prevail_start.size(); ++i) {
        prevail_start[i].dump();
    }
    cout << "Prevails overall:" << endl;
    for(int i = 0; i < prevail_overall.size(); ++i) {
        prevail_overall[i].dump();
    }
    cout << "Prevails end:" << endl;
    for(int i = 0; i < prevail_end.size(); ++i) {
        prevail_end[i].dump();
    }
    cout << "Preposts start:" << endl;
    for(int i = 0; i < pre_post_start.size(); ++i) {
        pre_post_start[i].dump();
    }
    cout << "Preposts end:" << endl;
    for(int i = 0; i < pre_post_end.size(); ++i) {
        pre_post_end[i].dump();
    }
    cout << "Mod Effs start:" << endl;
    for(int i = 0; i < mod_effs_start.size(); ++i) {
        mod_effs_start[i].dump();
    }
    cout << "Mod Effs end:" << endl;
    for(int i = 0; i < mod_effs_end.size(); ++i) {
        mod_effs_end[i].dump();
    }
    cout << endl;
}

bool Operator::is_applicable(const TimeStampedState & state, int allowRelaxed,
        TimedSymbolicStates* timedSymbolicStates, bool use_modules) const
{
    if(g_parameters.disallow_concurrent_actions && !state.operators.empty())
        return false;

    if(g_parameters.epsilonize_internally) {
        for(unsigned int i = 0; i < state.operators.size(); ++i) {
            double time_increment = state.operators[i].time_increment;
            if(time_equals(time_increment, g_parameters.epsSchedulingGapTime)) {
                return false;
            }
        }
    }

    if(g_parameters.use_cost_modules_for_applicability || (g_variable_types[duration_var] != costmodule)) {
        double duration = get_duration(&state, allowRelaxed);
        if(duration < 0 || duration >= INFINITE_COST) {
            return false;
        }
    }

    for(int i = 0; i < prevail_start.size(); i++) {
        if(prevail_start[i].is_module() && !use_modules)
            continue;
        if(!prevail_start[i].is_applicable(state, this, allowRelaxed))
            return false;
    }
    for(int i = 0; i < pre_post_start.size(); i++) {
        if(!pre_post_start[i].is_applicable(state))
            return false;
    }

    // There may be no simultaneous applications of two instances of the
    // same ground operator (for technical reasons, to simplify the task
    // of keeping track of durations committed to at the start of the
    // operator application)
    for(int i = 0; i < state.operators.size(); i++)
        if(state.operators[i].get_name() == get_name())
            return false;

    // FIXME: We could do this better by looping through the next call
    // but this will stop to work as soon as an effect module needs application
    if(!use_modules)
        return true;

    return TimeStampedState(state, *this, allowRelaxed).
        is_consistent_when_progressed(allowRelaxed, timedSymbolicStates);
}

bool Operator::isDisabledBy(const Operator* other) const
{
    if(name.compare(other->name) == 0)
        return false;
    if(deletesPrecond(prevail_start, other->pre_post_start))
        return true;
    if(deletesPrecond(prevail_start, other->pre_post_end))
        return true;
    if(deletesPrecond(prevail_overall, other->pre_post_start))
        return true;
    if(deletesPrecond(prevail_overall, other->pre_post_end))
        return true;
    if(deletesPrecond(prevail_end, other->pre_post_start))
        return true;
    if(deletesPrecond(prevail_end, other->pre_post_end))
        return true;
    if(deletesPrecond(pre_post_start, other->pre_post_start))
        return true;
    if(deletesPrecond(pre_post_start, other->pre_post_end))
        return true;
    if(deletesPrecond(pre_post_end, other->pre_post_start))
        return true;
    if(deletesPrecond(pre_post_end, other->pre_post_end))
        return true;
    //    if(writesOnSameVar(pre_post_start,other->pre_post_start)) return true;
    //    if(writesOnSameVar(pre_post_start,other->pre_post_end)) return true;
    //    if(writesOnSameVar(pre_post_end,other->pre_post_start)) return true;
    //    if(writesOnSameVar(pre_post_end,other->pre_post_end)) return true;

    return false;
}

bool Operator::enables(const Operator* other) const
{
    if(name.compare(other->name) == 0)
        return false;
    if(achievesPrecond(pre_post_start, other->prevail_start))
        return true;
    if(achievesPrecond(pre_post_start, other->prevail_overall))
        return true;
    if(achievesPrecond(pre_post_start, other->prevail_end))
        return true;
    if(achievesPrecond(pre_post_end, other->prevail_start))
        return true;
    if(achievesPrecond(pre_post_end, other->prevail_overall))
        return true;
    if(achievesPrecond(pre_post_end, other->prevail_end))
        return true;
    if(achievesPrecond(pre_post_start, other->pre_post_start))
        return true;
    if(achievesPrecond(pre_post_start, other->pre_post_end))
        return true;
    if(achievesPrecond(pre_post_end, other->pre_post_start))
        return true;
    if(achievesPrecond(pre_post_end, other->pre_post_end))
        return true;
    return false;
}

bool Operator::achievesPrecond(const vector<PrePost>& effects, const vector<
        Prevail>& conds) const
{
    for(int i = 0; i < effects.size(); ++i) {
        for(int j = 0; j < conds.size(); ++j) {
            if(effects[i].var == conds[j].var && state_equals(
                        effects[i].post, conds[j].prev)) {
                return true;
            }
        }
    }
    return false;
}

bool Operator::achievesPrecond(const vector<PrePost>& effs1, const vector<
        PrePost>& effs2) const
{
    for(int i = 0; i < effs1.size(); ++i) {
        for(int j = 0; j < effs2.size(); ++j) {
            if(effs1[i].var == effs2[j].var && state_equals(effs1[i].post,
                        effs2[j].pre)) {
                return true;
            }
        }
    }
    return false;
}

//FIXME: numerical effects?? conditional effects?? axioms??
bool Operator::deletesPrecond(const vector<Prevail>& conds, const vector<
        PrePost>& effects) const
{
    for(int i = 0; i < conds.size(); ++i) {
        for(int j = 0; j < effects.size(); ++j) {
            if(conds[i].var == effects[j].var && !state_equals(conds[i].prev,
                        effects[j].post)) {
                return true;
            }
        }
    }
    return false;
}

bool Operator::deletesPrecond(const vector<PrePost>& effs1, const vector<
        PrePost>& effs2) const
{
    for(int i = 0; i < effs1.size(); ++i) {
        for(int j = 0; j < effs2.size(); ++j) {
            if(effs1[i].var == effs2[j].var && !state_equals(effs1[i].pre,
                        effs2[j].post)) {
                //	  if(effs1[i].var == effs2[j].var &&
                //	     !state_equals(effs1[i].pre,effs2[j].post) &&
                //	     !state_equals(effs1[i].pre,-1.0)) 
                return true;
            }
        }
    }
    return false;
}

bool Operator::writesOnSameVar(const vector<PrePost>& effs1, const vector<
        PrePost>& effs2) const
{
    for(int i = 0; i < effs1.size(); ++i) {
        for(int j = 0; j < effs2.size(); ++j) {
            if(effs1[i].var == effs2[j].var/* && effs1[i].post != effs2[j].post*/) {
                return true;
            }
        }
    }
    return false;
}

double Operator::get_duration(const TimeStampedState* state, int relaxed, bool allow_module) const
{
    assert(duration_var >= 0);
    assert(state != NULL);

    if(g_variable_types[duration_var] == costmodule) {
        if(!allow_module) {
            return 0.0;
        }

        g_setModuleCallbackState(state);

        modules::ParameterList parameters = g_cost_modules[duration_var]->params;
        addGroundParameters(parameters);

        predicateCallbackType pct = getPreds;
        numericalFluentCallbackType nct = getFuncs;
        double duration = g_cost_modules[duration_var]->checkCost(
                parameters, pct, nct, relaxed);
        //printf("Duration from module: %f\n", duration);
        return duration;
    }

    // default behaviour: duration is defined by duration_var
    return (*state)[duration_var];
}

bool Operator::operator<(const Operator &other) const
{
    return name < other.name;
}
