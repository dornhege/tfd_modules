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
#include "closed_list.h"
#include <tr1/unordered_map>

class Operator
{
    private:
        vector<Prevail> prevail_start; // var, val
        vector<Prevail> prevail_overall; // var, val
        vector<Prevail> prevail_end; // var, val
        vector<PrePost> pre_post_start; // var, old-val, new-val
        vector<PrePost> pre_post_end; // var, old-val, new-val
        vector<ModuleEffect> mod_effs_start;
        vector<ModuleEffect> mod_effs_end;
        vector<ModuleGrounding> mod_groundings;
        int duration_var;
        string name;
 
        /// For grounded ops that were live grounded - the ungrounded op that 
        /// this was grounded from, NULL otherwise
        const Operator* grounding_parent;

        mutable int numBranches;    ///< For ungrounded ops - how often was this one grounded.

        typedef tr1::unordered_map<TimeStampedState, int, TssHash, TssEquals> BranchGroundingCountMap;
        mutable BranchGroundingCountMap numBranchesByState;

    private:
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

        /// Is this operator fully grounded or just partially.
        bool isGrounded() const;

        /// Ground this ungrounded operator in state.
        /**
         * \param [out] ok set to true if a new grounded operator was constructed.
         *      Set to false in any other case, i.e. if no further grounding for
         *      a partially grounded operator was possible or if this operator
         *      is already grounded (it could not be grounded any more than that).
         *      If ok is false the returned operator should not be used!
         * \returns the newly grounded operator.
         */
        Operator ground(const TimeStampedState & state, bool relaxed, bool & ok) const;

        /// Ground manually for monitoring
        Operator groundManually(const std::string& groundParam, bool & ok) const;

        /// Add additional ground parameters from a grounded operators name
        /// to the partially grounded parameters of a module call.
        /**
         * \param [in, out] parameters if grounding added parameters to the operator's name
         *      those are added to parameters. It should be the case that all parameters
         *      up to that point are the same in parameters and the operator's name.
         *      FIXME: Must this be true at all or is it sufficient to just add the grounded
         *      parameter. Maybe a module wants other params like a subset or different order.
         *      Maybe a module wants to ignore the grounded param? Better add ?grounding tag in
         *      param descriptions?
         */
        void addGroundParameters(modules::ParameterList & parameters) const;

        const Operator* getGroundingParent() const { return grounding_parent; }

        /// How often did we branch off this (in state given parameter setting).
        int getNumBranches(const TimeStampedState* state) const;

        /// Calculate the duration of this operator when applied in state.
        /**
         * This function will retrieve the duration and handle cost modules correctly.
         * The state pointer should keep on valid as it can be used for caching in modules.
         *
         * \param [in] allow_module if false and the duration is determined by a module, 
         *      do NOT call the module and return 0.0
         */
        double get_duration(const TimeStampedState* state, int relaxed, bool allow_module = true) const;

        /// Compute applicability of this operator in state.
        /**
         * \param [in] allowRelaxed if true, only relaxed module calls will be performed.
         * \param [out] timedSymbolicStates if not NULL the timedSymbolicStates will be computed
         * \param [in] use_modules if false, module prevails will not be checked and assumed true
         */
        bool is_applicable(const TimeStampedState & state, int allowRelaxed,
            TimedSymbolicStates* timedSymbolicStates = NULL, bool use_modules = true) const;

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
