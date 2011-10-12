#ifndef BEST_FIRST_SEARCH_H
#define BEST_FIRST_SEARCH_H

#include <vector>
#include <queue>
#include "closed_list.h"
#include "search_engine.h"
#include "state.h"
#include "operator.h"
#include <tr1/tuple>
#include "search_statistics.h"

class Heuristic;

typedef std::tr1::tuple<const TimeStampedState *, const Operator *, double> OpenListEntry;

class OpenListEntryCompare
{
    public:
        bool operator()(const OpenListEntry left_entry, const OpenListEntry right_entry) const
        {
            return std::tr1::get<2>(right_entry) < std::tr1::get<2>(left_entry);
        }
};

typedef priority_queue<OpenListEntry, std::vector<OpenListEntry>,OpenListEntryCompare> OpenList;

struct OpenListInfo
{
    OpenListInfo(Heuristic *heur, bool only_pref);
    Heuristic *heuristic;
    bool only_preferred_operators;
    //    OpenList<OpenListEntry> open;
    OpenList open;
    int priority; // low value indicates high priority
};

/// Maps logical state to best timestamp for that state
typedef std::map<std::vector<double>, double> LogicalStateClosedList;
bool knownByLogicalStateOnly(LogicalStateClosedList& scl,
        const TimedSymbolicStates& timedSymbolicStates);

class BestFirstSearchEngine : public SearchEngine
{
    private:
        std::vector<Heuristic *> heuristics;
        std::vector<Heuristic *> preferred_operator_heuristics;
        std::vector<OpenListInfo> open_lists;
        ClosedList closed_list;
        
        LogicalStateClosedList logical_state_closed_list;

        std::vector<double> best_heuristic_values;
        std::vector<const TimeStampedState*> best_states;

        TimeStampedState current_state;
        const TimeStampedState *current_predecessor;
        const Operator *current_operator;

        time_t start_time;
        int currentQueueIndex;

        SearchStatistics search_statistics;

    private:
        bool is_dead_end();
        bool check_goal();
        bool check_progress(const TimeStampedState* state);
        void report_progress();
        void reward_progress();
        void generate_successors(const TimeStampedState *parent_ptr);
        void dump_transition() const;
        /// Dump the whole knowledge of search engine.
        void dump_everything() const;
        OpenListInfo *select_open_queue();

        void dump_plan_prefix_for_current_state() const;
        void dump_plan_prefix_for__state(const TimeStampedState &state) const;

        /// Compute G depending on the current g mode.
        double getG(const TimeStampedState* state_ptr, const TimeStampedState* closed_ptr, const Operator* op) const;
        double getGc(const TimeStampedState *parent_ptr) const;
        double getGc(const TimeStampedState *parent_ptr, const Operator *op) const;
        double getGm(const TimeStampedState *parent_ptr) const;
        double getGt(const TimeStampedState *parent_ptr) const;

    protected:
        virtual SearchEngine::status step();

    public:
        enum QueueManagementMode
        {
            ROUND_ROBIN, PRIORITY_BASED
        } mode;

        BestFirstSearchEngine(QueueManagementMode _mode);
        ~BestFirstSearchEngine();
        void add_heuristic(Heuristic *heuristic, bool use_estimates,
                bool use_preferred_operators);
        virtual void statistics(time_t & current_time);
        virtual void initialize();
        SearchEngine::status fetch_next_state();

    public:
        double bestMakespan;
        double bestSumOfGoals;
};

#endif
