#ifndef ANALYSIS_H
#define ANALYSIS_H

#include "globals.h"
#include <fstream>
#include <string>
#include <set>
#include <deque>
#include <map>
#include <utility>
#include <tr1/tuple>
#include <tr1/unordered_map>
#include <tr1/unordered_set>
#include "closed_list.h"

using namespace std;

class Analysis
{
    public:
        Analysis();

        int getCurrentEventNumber() const { return currentEventNumber; }

        /// Write all records to a dot file starting with filePrefix and appending the current event nr.
        /**
         * \returns true, if the file was written.
         */
        bool writeDot(const std::string & filePrefix);

        /// Record a basic step, where succ is closed and expanded.
        /**
         * \param [in] pred predecessor pointer that is safe to be stored
         * \param [in] op operator pointer that is safe to be stored
         * \param [in] succ successor that is safe to be stored
         */
        void recordClosingStep(const TimeStampedState* pred, const Operator* op,
                const TimeStampedState* succ);

        /// Record a tentative step, where succ is not closed, but discarded.
        /**
         * \param [in] pred predecessor pointer that is safe to be stored
         * \param [in] op operator pointer that is safe to be stored
         * \param [in] succ successor that is not safe to be stored and needs to be replicated/matched
         */
        void recordDiscardingStep(const TimeStampedState* pred, const Operator* op,
                const TimeStampedState & succ, const ClosedList & closedList);

        /// Record a tentative step, where op is module relaxed, but not applicable.
        /**
         * \param [in] pred predecessor pointer that is safe to be stored
         * \param [in] op operator pointer that is safe to be stored
         */
        void recordModuleRelaxedDiscardingStep(const TimeStampedState* pred, const Operator* op);

        /// checkGoal passed for goalState.
        /**
         * \param [in] goalState goal that is not safe to be stored and needs to be matched/repl.
         */
        void recordGoal(const TimeStampedState & goalState);

        /// op was pushed as (module relaxed) applicable for parent into openIndex open queue.
        /**
         * \param [in] parent parent pointer that is safe to be stored
         * \param [in] op operator pointer that is safe to be stored
         * \param [in] openIndex index of the open queue
         * \param [in] priority priority used when pushing
         */
        void recordOpenPush(const TimeStampedState* parent, const Operator* op,
                int openIndex, double priority);

        /// Live grounding fetched an ungrounded operator, grounded that to op, which was inapplicable.
        /// If live grounding grounded an applicable one after fetching the ungrounded operator,
        /// it will appear normally in step().
        /**
         * \param [in] pred predecessor pointer that is safe to be stored
         * \param [in] op operator pointer that is safe to be stored
         */
        void recordLiveGroundingDiscard(const TimeStampedState* pred, const Operator* op);

        /// The ungrounded operator op could not be grounded any more in pred.
        void recordLiveGroundingGroundedOut(const TimeStampedState* pred, const Operator* op);

        /// An ungrounded was supported to be pushed, but discarded.
        void recordLiveGroundingUngroundedDiscard(const TimeStampedState* pred, const Operator* op);

        /// The operator op has just been grounded successfully.
        void recordLiveGrounding(const TimeStampedState* pred, const Operator* op);

    protected:
        struct OpenEntry {
            int eventNumber;
            int openIndex;
            double priority;

            bool operator<(const OpenEntry & rhs) const {
                return eventNumber < rhs.eventNumber;
            }
        };

        typedef map< pair<const TimeStampedState*, const Operator*>,
                pair<int, const TimeStampedState*> > CloseRecordMap;
        typedef map< pair<const TimeStampedState*, const Operator*>, int > EventRecordMap;
        /// open push for TimeStampedState, Operator in queue
        /**
         * Might happen multiple times, current_state might be rediscovered by a better path,
         * pushes into different queues, live grounding pushes exaclty the same
         */
        typedef map< pair<const TimeStampedState*, const Operator*>,
                deque<OpenEntry> > OpenRecordMap;

        void writeDotNodes(std::ofstream & of);

        void writeDotEdges(std::ofstream & of);

        std::string generateCloseEdgeLabel(const CloseRecordMap::value_type & edge,
                const OpenRecordMap & openRecords,
                set< pair<const TimeStampedState*, const Operator*> > & handledTransitions);

        std::string generateModuleDiscardEdgeLabel(const EventRecordMap::value_type & edge,
                const OpenRecordMap & openRecords,
                set< pair<const TimeStampedState*, const Operator*> > & handledTransitions);

        std::string generateLiveGroundingDiscardEdgeLabel(const EventRecordMap::value_type & edge,
                const OpenRecordMap & openRecords,
                set< pair<const TimeStampedState*, const Operator*> > & handledTransitions);

        std::string generateOpenEdgeLabel(const OpenRecordMap::value_type & edge);

        void writeDotEdgesAll(std::ofstream & of);
        void writeDotEdgesCondensed(std::ofstream & of);

        void writeDotEqualStates(std::ofstream & of);

        /// Generate a unique node name "state_XXXXXXXX" based on this state pointer.
        std::string generateNodeName(const TimeStampedState* state);

        /// Generate a unique node name "state_XXXXX_op_XXXXX" for this state and op.
        std::string generateUngroundedOpNodeName(pair<const TimeStampedState*, const Operator*> sop);

        /// Generate a new unused name.
        std::string createAnonymousNode(std::ofstream & of, const std::string & shape);

        /// Generate a node label for this state using the actual state content.
        std::string generateNodeLabel(const TimeStampedState* state);

        /// If state cannot be stored, find a storable instance in the records.
        /// If that does not exist, create one.
        const TimeStampedState* findOrReplicateMatchingState(const TimeStampedState & state,
                bool warnIfReplicated = true);

        /// Is this state in the closed list or do we only have that locally.
        bool isReplicated(const TimeStampedState* state);

        /// Split a string label with newlines (\\n) if it is longer than maxLength
        std::string breakStringLabel(const std::string & s, unsigned int maxLength);

    protected:
        unsigned int lastAnonymousNr;
        unsigned int currentEventNumber;

        struct TssHashTimestamp
        {
            std::size_t operator()(const TimeStampedState & tss) const;

            TssHash tssHash;
        };

        /// TimeStampedState equals that requires same timestamp for equality.
        struct TssEqualsTimestamp
        {
            bool operator()(const TimeStampedState &tss1, const TimeStampedState &tss2) const;

            TssEquals tssEquals;
        };

        /// Recorded states that are safe to store permanently.
        /// State -> Recorded Ptr. The ptr should be unique per state.
        typedef tr1::unordered_map<TimeStampedState, const TimeStampedState*,
                TssHashTimestamp, TssEqualsTimestamp> RecordedStatesMap;
        RecordedStatesMap recordedStates;

        /// Some states might not exist permanently, stored them here to have a safe pointer.
        typedef tr1::unordered_set<TimeStampedState,
                TssHashTimestamp, TssEqualsTimestamp> ReplicatedStatesSet;
        ReplicatedStatesSet replicatedStates;

        CloseRecordMap closedRecords;
        CloseRecordMap discardRecords;
        EventRecordMap moduleRelaxedDiscardRecords;
        EventRecordMap liveGroundingDiscardRecords;
        EventRecordMap operatorGroundingRecords;
        EventRecordMap operatorGroundedOutRecords;
        EventRecordMap ungroundedOpDiscardRecords;
        OpenRecordMap openRecords;
        map<const TimeStampedState*, int> goalRecords;
};

#endif

