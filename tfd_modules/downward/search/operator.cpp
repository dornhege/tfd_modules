#include "globals.h"
#include "operator.h"
#include "module.h"
#include "best_first_search.h"

#include <iostream>
using namespace std;

Prevail::Prevail(istream &in)
{
    in >> var >> prev;
}

bool Prevail::is_applicable(const TimeStampedState &state, const Operator * op,
        bool allowRelaxed) const
{
    assert(var >= 0 && var < g_variable_name.size());
    assert((prev >= 0 && prev < g_variable_domain[var]) || (g_variable_types[var] == module));
    if(g_variable_types[var] == module) {
        assert(op != NULL);     // there should always be an operator unless 
                                // called from LogicAxiom, where it should not be a module
        if(op == NULL)
            return false;

        pair<const TimeStampedState*, const Operator*> * pcPtr = new pair<
            const TimeStampedState*, const Operator*> (&state, op);
        plannerContextPtr pc = pcPtr;
        plannerContextCompareType pcct = compareContext;
        g_modulecallback_state = &state;
        predicateCallbackType pct = getPreds;
        numericalFluentCallbackType nct = getFuncs;
        bool tookContext = true;
        double cost = g_condition_modules[var]->checkCondition(
                g_condition_modules[var]->params, pct, nct, allowRelaxed, pc,
                pcct, tookContext);
        if(!tookContext) {
            delete pcPtr;
        }
        return cost < INFINITE_COST;
    } else {
        return double_equals(state[var], prev);
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
}

bool PrePost::is_applicable(const TimeStampedState &state) const
{
    assert(var >= 0 && var < g_variable_name.size());
    assert(pre == -1 || (pre >= 0 && pre < g_variable_domain[var]));
    return pre == -1 || (double_equals(state[var], pre));
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

Operator::Operator(istream &in)
{
    check_magic(in, "begin_operator");
    in >> ws;
    getline(in, name);
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
}

Operator::Operator(bool uses_concrete_time_information)
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
    cout << endl;
}

bool Operator::is_applicable(const TimeStampedState & state,
        TimedSymbolicStates& timedSymbolicStates, bool allowRelaxed) const
{
    // query duration now (wasted call) just to check applicability: caching?
    double duration = get_duration(&state);
    if(duration <= 0 || duration >= INFINITE_COST)
        return false;

    for(int i = 0; i < pre_post_start.size(); i++)
        if(!pre_post_start[i].is_applicable(state))
            return false;

    for(int i = 0; i < prevail_start.size(); i++)
        if(!prevail_start[i].is_applicable(state, this, allowRelaxed)) //FIXME: is this a temporaray??????????
            return false;

    // Make sure that there is no other operator currently running, that
    // has the same end timepoint as this operator would have.
    //for(int i = 0; i < state.scheduled_effects.size(); i++) {
    //if(double_equals(state.scheduled_effects[i].time_increment,
    //    state[duration_var])) {
    //    return false;
    //}
    //}

    // There may be no simultaneous applications of two instances of the
    // same ground operator (for technical reasons, to simplify the task
    // of keeping track of durations committed to at the start of the
    // operator application)
    for(int i = 0; i < state.operators.size(); i++)
        if(state.operators[i].get_name() == get_name())
            return false;

    return TimeStampedState(state, *this).is_consistent_when_progressed(timedSymbolicStates);
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
            if(effects[i].var == conds[j].var && double_equals(
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
            if(effs1[i].var == effs2[j].var && double_equals(effs1[i].post,
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
            if(conds[i].var == effects[j].var && !double_equals(conds[i].prev,
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
            if(effs1[i].var == effs2[j].var && !double_equals(effs1[i].pre,
                        effs2[j].post)) {
                //	  if(effs1[i].var == effs2[j].var &&
                //	     !double_equals(effs1[i].pre,effs2[j].post) &&
                //	     !double_equals(effs1[i].pre,-1.0)) 
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

double Operator::get_duration(const TimeStampedState* state, int relaxed) const
{
    assert(duration_var >= 0);
    assert(state != NULL);

    if(g_variable_types[duration_var] == costmodule) {
        pair<const TimeStampedState*, const Operator*> * pcPtr = 
            new pair<const TimeStampedState*, const Operator*> (state, this);
        plannerContextPtr pc = pcPtr;
        plannerContextCompareType pcct = compareContext;
        bool tookContext = true;
        g_modulecallback_state = state;
        predicateCallbackType pct = getPreds;
        numericalFluentCallbackType nct = getFuncs;
        double duration = g_cost_modules[duration_var]->checkCost(
                g_cost_modules[duration_var]->params, pct, nct, relaxed,
                pc, pcct, tookContext);
        //printf("Duration from module: %f\n", duration);
        if(!tookContext) {
            delete pcPtr;
        }

        return duration;
    }

    // default behaviour: duration is defined by duration_var
    return (*state)[duration_var];
}

bool Operator::operator<(const Operator &other) const
{
    return name < other.name;
}
