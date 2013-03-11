#ifndef STATE_H
#define STATE_H

#include <algorithm>
#include <cassert>
#include <iostream>
#include <vector>
#include <set>
#include "globals.h"
#include "module.h"
#include "ros_printouts.h"

using namespace std;

class Operator;
class TimeStampedState;

struct Prevail
{
    int var;
    double prev;
    Prevail(istream &in);
    Prevail(int v, double p) :
        var(v), prev(p)
    {
    }
    virtual ~Prevail()
    {
    }

    bool is_applicable(const TimeStampedState & state, const Operator* op, bool allowRelaxed) const;

    /// Will this prevail be computed by a module?
    bool is_module() const;

    void dump() const;

    bool operator<(const Prevail &other) const {
        if(var < other.var)
            return true;
        if(var > other.var)
            return false;
        return prev < other.prev;
    }
};

struct PrePost
{
    int var;
    double pre;
    int var_post;
    double post;
    vector<Prevail> cond_start;
    vector<Prevail> cond_overall;
    vector<Prevail> cond_end;
    assignment_op fop;

    PrePost()
    {
    } // Needed for axiom file-reading constructor, unfortunately.
    PrePost(std::istream &in);
    PrePost(int v, double pr, int vpo, double po, const std::vector<Prevail> &co_start,
            const std::vector<Prevail> &co_oa,
            const std::vector<Prevail> &co_end, assignment_op fo = assign) :
        var(v), pre(pr), var_post(vpo), post(po),
        cond_start(co_start), cond_overall(co_oa), cond_end(co_end), fop(fo)
    {
    }
    PrePost(int v, double p) : var(v), post(p) {}
    virtual ~PrePost() {}

    bool is_applicable(const TimeStampedState &state) const;

    bool does_fire(const TimeStampedState &state, const Operator* op, bool relaxed) const {
        for(unsigned int i = 0; i < cond_start.size(); i++)
            if(!cond_start[i].is_applicable(state, op, relaxed))
                return false;
        return true;
    }

    void dump() const;
};

struct ModuleEffect
{
    vector<Prevail> cond_start;
    vector<Prevail> cond_overall;
    vector<Prevail> cond_end;
    EffectModule *module;
    ModuleEffect(std::istream &in);
    ModuleEffect(vector<Prevail> &_cond_start,
            vector<Prevail> &_cond_overall, vector<Prevail> &_cond_end,
            EffectModule *_module) :
        cond_start(_cond_start), cond_overall(_cond_overall), cond_end(
                _cond_end), module(_module)
    {
        ROS_ASSERT(module != NULL);
    }

    bool does_fire(const TimeStampedState &state, const Operator* op, bool relaxed) const
    {
        for(unsigned int i = 0; i < cond_start.size(); i++)
            if(!cond_start[i].is_applicable(state, op, relaxed))
                return false;
        return true;
    }

    void dump() const;
};

struct ModuleGrounding
{
    GroundingModule *module;
    ModuleGrounding(std::istream &in);

    void dump() const;
};


struct ScheduledEffect : public PrePost
{
    double time_increment;
    const Operator* parent_op;  ///< the op this PrePost belongs to

    ScheduledEffect(double t, vector<Prevail> &cas, vector<Prevail> &coa, vector<Prevail> &cae,
        int va, int vi, assignment_op op,
        const Operator* parent_op) :
        PrePost(va, -1.0, vi, -1.0, cas, coa, cae, op), time_increment(t), parent_op(parent_op)
    {
        initialize();
    }
    ScheduledEffect(double t, const PrePost& pp, const Operator* parent_op) :
        PrePost(pp), time_increment(t), parent_op(parent_op)
    {
        initialize();
    }
    void initialize()
    {
        sort(cond_start.begin(), cond_start.end());
        sort(cond_overall.begin(), cond_overall.end());
        sort(cond_end.begin(), cond_end.end());
    }
    bool operator<(const ScheduledEffect &other) const
    {
        if(time_increment < other.time_increment)
            return true;
        if(time_increment > other.time_increment)
            return false;
        if(var < other.var)
            return true;
        if(var > other.var)
            return false;
        if(pre < other.pre)
            return true;
        if(pre > other.pre)
            return false;
        if(var_post < other.var_post)
            return true;
        if(var_post > other.var_post)
            return false;
        if(post < other.post)
            return true;
        if(post > other.post)
            return false;
        if(fop < other.fop)
            return true;
        if(fop > other.fop)
            return false;
        if(cond_start.size() < other.cond_start.size())
            return true;
        if(cond_start.size() > other.cond_start.size())
            return false;
        if(cond_overall.size() < other.cond_overall.size())
            return true;
        if(cond_overall.size() > other.cond_overall.size())
            return false;
        if(cond_end.size() < other.cond_end.size())
            return true;
        if(cond_end.size() > other.cond_end.size())
            return false;
        if(lexicographical_compare(cond_start.begin(), cond_start.end(),
                    other.cond_start.begin(), other.cond_start.end()))
            return true;
        if(lexicographical_compare(other.cond_start.begin(), other.cond_start.end(),
            cond_start.begin(), cond_start.end()))
            return false;
        if(lexicographical_compare(cond_overall.begin(), cond_overall.end(),
            other.cond_overall.begin(), other.cond_overall.end()))
            return true;
        if(lexicographical_compare(other.cond_overall.begin(), other.cond_overall.end(),
            cond_overall.begin(), cond_overall.end()))
            return false;
        if(lexicographical_compare(cond_end.begin(), cond_end.end(),
                    other.cond_end.begin(), other.cond_end.end()))
            return true;
        if(lexicographical_compare(other.cond_end.begin(), other.cond_end.end(),
            cond_end.begin(), cond_end.end()))
            return false;
        return false;
    }
};

struct ScheduledModuleEffect : public ModuleEffect
{
    double time_increment;
    const Operator* parent_op;  ///< the op this ModuleEffect belongs to

    ScheduledModuleEffect(double t, vector<Prevail> &cas,
            vector<Prevail> &coa, vector<Prevail> &cae,
            EffectModule *module,
            const Operator* parent_op) :
        ModuleEffect(cas, coa, cae, module), time_increment(t), parent_op(parent_op)
    {
        initialize();
    }

    ScheduledModuleEffect(double _time_increment,
            const ModuleEffect &_mod_eff,
            const Operator* parent_op) :
        ModuleEffect(_mod_eff), time_increment(_time_increment), parent_op(parent_op)
    {
    }

    void initialize()
    {
        sort(cond_start.begin(), cond_start.end());
        sort(cond_overall.begin(), cond_overall.end());
        sort(cond_end.begin(), cond_end.end());
    }

    bool operator<(const ScheduledModuleEffect &other) const
    {
        if(time_increment < other.time_increment)
            return true;
        if(time_increment > other.time_increment)
            return false;
        if(cond_start.size() < other.cond_start.size())
            return true;
        if(cond_start.size() > other.cond_start.size())
            return false;
        if(cond_overall.size() < other.cond_overall.size())
            return true;
        if(cond_overall.size() > other.cond_overall.size())
            return false;
        if(cond_end.size() < other.cond_end.size())
            return true;
        if(cond_end.size() > other.cond_end.size())
            return false;
        if(lexicographical_compare(cond_start.begin(), cond_start.end(),
                    other.cond_start.begin(), other.cond_start.end()))
            return true;
        if(lexicographical_compare(other.cond_start.begin(),
                    other.cond_start.end(), cond_start.begin(),
                    cond_start.end()))
            return false;
        if(lexicographical_compare(cond_overall.begin(),
                    cond_overall.end(), other.cond_overall.begin(),
                    other.cond_overall.end()))
            return true;
        if(lexicographical_compare(other.cond_overall.begin(),
                    other.cond_overall.end(), cond_overall.begin(),
                    cond_overall.end()))
            return false;
        if(lexicographical_compare(cond_end.begin(), cond_end.end(),
                    other.cond_end.begin(), other.cond_end.end()))
            return true;
        if(lexicographical_compare(other.cond_end.begin(),
                    other.cond_end.end(), cond_end.begin(), cond_end.end()))
            return false;
        return (module->internal_name.compare(other.module->internal_name));
    }
};

struct ScheduledCondition : public Prevail
{
    double time_increment;
    const Operator* parent_op;  ///< the op this ModuleEffect belongs to

    ScheduledCondition(double t, int v, double p, const Operator* parent_op) :
        Prevail(v, p), time_increment(t), parent_op(parent_op)
    {
    }
    ScheduledCondition(double t, const Prevail &prev, const Operator* parent_op)
        : Prevail(prev), time_increment(t), parent_op(parent_op)
    {
    }
    bool operator<(const ScheduledCondition &other) const
    {
        if(time_increment < other.time_increment)
            return true;
        if(time_increment > other.time_increment)
            return false;
        if(var < other.var)
            return true;
        if(var > other.var)
            return false;
        return prev < other.prev;
    }
};

class ScheduledOperator;

typedef std::pair<std::vector<double>, double> TimedSymbolicState;
typedef std::vector<TimedSymbolicState> TimedSymbolicStates;

struct TssHash
{
    std::size_t operator()(const TimeStampedState & tss) const;
};

bool prevailEquals(const Prevail &prev1, const Prevail &prev2);

bool scheduledConditionEquals(const ScheduledCondition &cond1, const ScheduledCondition &cond2);

bool scheduledEffectEquals(const ScheduledEffect &eff1, const ScheduledEffect &eff2);

struct TssEquals
{
    bool operator()(const TimeStampedState &tss1, const TimeStampedState &tss2) const;
};

class TimeStampedState
{
    friend class RelaxedState;
    friend class AxiomEvaluator;
    friend struct PrePost;
    friend struct Prevail;
    friend class Operator;
    friend class NoHeuristic;
    friend class CyclicCGHeuristic;
    friend class ConsistencyCache;
    friend struct TssCompareIgnoreTimestamp;

    private:
        bool satisfies(const Prevail& cond, const Operator* op, bool relaxed) const
        {
            return cond.is_applicable(*this, op, relaxed);
        }

        bool satisfies(const vector<Prevail>& conds, const Operator* op, bool relaxed) const
        {
            for(unsigned int i = 0; i < conds.size(); i++)
                if(!satisfies(conds[i], op, relaxed))
                    return false;
            return true;
        }

        bool satisfies(const pair<int, double>& goal) const
        {
            return state_equals(state[goal.first], goal.second);
        }

        void apply_numeric_effect(int lhs, assignment_op op, int rhs)
        {
            switch(op) {
                case assign:
                    state[lhs] = state[rhs];
                    break;
                case scale_up:
                    state[lhs] *= state[rhs];
                    break;
                case scale_down:
                    state[lhs] /= state[rhs];
                    break;
                case increase:
                    state[lhs] += state[rhs];
                    break;
                case decrease:
                    state[lhs] -= state[rhs];
                    break;
                default:
                    assert(false);
                    break;
            }
        }

        void apply_discrete_effect(int var, double post)
        {
            state[var] = post;
        }

        void apply_module_effect(string internal_name, const Operator* op, bool relaxed);

        void apply_effect(int lhs, assignment_op op, int rhs, double post)
        {
            if(is_functional(lhs)) {
                apply_numeric_effect(lhs, op, rhs);
            } else {
                apply_discrete_effect(lhs, post);
            }
        }

        void initialize()
        {
            sort(scheduled_effects.begin(), scheduled_effects.end());
            sort(conds_over_all.begin(), conds_over_all.end());
            sort(conds_at_end.begin(), conds_at_end.end());
        }
        double eps_time(double offset) const;

    public:
        vector<double> state;
        vector<ScheduledEffect> scheduled_effects;
        vector<ScheduledModuleEffect> scheduled_module_effects;
        vector<ScheduledCondition> conds_over_all;
        vector<ScheduledCondition> conds_at_end;

        double timestamp;
        vector<ScheduledOperator> operators;

        TimeStampedState(istream &in);
        // apply an operator
        TimeStampedState(const TimeStampedState &predecessor, const Operator &op, int relaxed);
        // let time pass without applying an operator
        TimeStampedState let_time_pass(
            bool go_to_intermediate_between_now_and_next_happening,     // usually false
            bool skip_eps_steps,            // usually false
            bool relaxed) const;            // usually false

        // unused TimeStampedState increase_time_stamp_by(double increment) const;

        double &operator[](int index)
        {
            return state[index];
        }
        double operator[](int index) const
        {
            return state[index];
        }
        void dump(bool verbose) const;

        /// Pretty PDDL like output for state.
        /**
         * \param [in] logical include logical predicates
         * \param [in] onlyTruePredicates when listing logical predicates, only list true ones
         * \param [in] numerical include numerical fluents
         * \param [in] separator how fluents should be separated
         * \param [in] maxLineLength how many characters should go on one line
         */
        std::string toPDDL(bool logical, bool onlyTruePredicates, bool numerical,
                std::string separator = " ", int maxLineLength = -1) const;

        void scheduleEffect(ScheduledEffect effect);

        double next_happening() const;

        bool is_consistent_now(bool relaxed) const;
        bool is_consistent_when_progressed(bool relaxed, TimedSymbolicStates* timedSymbolicStates) const;

        const double &get_timestamp() const
        {
            return timestamp;
        }

        bool satisfies(const vector<pair<int, double> >& goal) const
        {
            for(unsigned int i = 0; i < goal.size(); i++)
                if(!satisfies(goal[i]))
                    return false;
            return true;
        }
};

//TimeStampedState &buildTestState(TimeStampedState &state);

#endif
