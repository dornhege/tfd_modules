#ifndef OPERATOR_H
#define OPERATOR_H

#include <cassert>
#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>
#include <math.h>

#include "globals.h"
#include "state.h"

class Operator
{
        vector<Prevail> prevail_start; // var, val
        vector<Prevail> prevail_overall; // var, val
        vector<Prevail> prevail_end; // var, val
        vector<PrePost> pre_post_start; // var, old-val, new-val
        vector<PrePost> pre_post_end; // var, old-val, new-val
        vector<ModuleEffect> mod_effs_start;
        vector<ModuleEffect> mod_effs_end;
        int duration_var;
        string name;

        bool deletesPrecond(const vector<Prevail>& conds,
                const vector<PrePost>& effects) const;
        bool deletesPrecond(const vector<PrePost>& effs1,
                const vector<PrePost>& effs2) const;
        bool achievesPrecond(const vector<PrePost>& effects, const vector<
                Prevail>& conds) const;
        bool achievesPrecond(const vector<PrePost>& effs1,
                const vector<PrePost>& effs2) const;
        bool writesOnSameVar(const vector<PrePost>& conds,
                const vector<PrePost>& effects) const;

        void sort_prevails(vector<Prevail> &prevails);
    public:
        Operator(std::istream &in);
        explicit Operator(bool uses_concrete_time_information);
        void dump() const;
        const vector<Prevail> &get_prevail_start() const {
            return prevail_start;
        }
        const vector<Prevail> &get_prevail_overall() const {
            return prevail_overall;
        }
        const vector<Prevail> &get_prevail_end() const {
            return prevail_end;
        }
        const vector<PrePost> &get_pre_post_start() const {
            return pre_post_start;
        }
        const vector<PrePost> &get_pre_post_end() const {
            return pre_post_end;
        }
        const vector<ModuleEffect> &get_mod_effs_start() const {
            return mod_effs_start;
        }
        const vector<ModuleEffect> &get_mod_effs_end() const {
            return mod_effs_end;
        }
        const int &get_duration_var() const {
            return duration_var;
        }
        const string &get_name() const {
            return name;
        }

        bool operator<(const Operator &other) const;

        /// Calculate the duration of this operator when applied in state.
        /**
         * This function will retrieve the duration and handle cost modules correctly.
         * The state pointer should keep on valid as it can be used for caching in modules.
         */
        double get_duration(const TimeStampedState* state, bool relaxed = false) const;

        /// Compute applicability of this operator in state.
        /**
         * \param [in] allowRelaxed if true, only relaxed module calls will be performed.
         * \param [out] timedSymbolicStates if not NULL the timedSymbolicStates will be computed
         */
        bool is_applicable(const TimeStampedState & state, bool allowRelaxed = false,
            TimedSymbolicStates* timedSymbolicStates = NULL) const;

        bool isDisabledBy(const Operator* other) const;

        bool enables(const Operator* other) const;

        virtual ~Operator()
        {
        }
};

class ScheduledOperator : public Operator
{
    public:
        double time_increment;
        ScheduledOperator(double t, const Operator& op) : Operator(op), time_increment(t)
        {
        }
        ScheduledOperator(double t) : Operator(true), time_increment(t)
        {
            if (time_increment >= HUGE_VAL) {
                printf("WARNING: Created scheduled operator with time_increment %f\n", t);
            }
        }
};

#endif
