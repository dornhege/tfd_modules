#ifndef ANALYSIS_H
#define ANALYSIS_H

#include "globals.h"
#include <fstream>
#include <string>
#include <set>
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

        void setEnabled(bool on) { enabled = on; }

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
                const TimeStampedState & succ);

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

        // TODO what is the ideal desired output?
        // Some of these overlap in transitions (open/closed), maybe should be marked as double/multi edge labels, - on the other hand should be mostly default for open->close, but might be different as open->ground->close (op grounded was never in open), open->discard
        // Can we have nil transitions going nowhere for grounding stop? (and open pushes???) - how should those be reflected? -> just use node?
        // Uniqueness of nodes before/after closed as closed list entries and current_state (is that always the same ptr?) - are we interested in the same "equal" node or in the exact same state?
        // current_state is always the same, pred/op are pointers usually from globals ops list + pred from closed list!
        // What is with let-time_pass? can we skip those?

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
        typedef map< pair<const TimeStampedState*, const Operator*>, int > DiscardRecordMap;
        typedef map< pair<const TimeStampedState*, const Operator*>, OpenEntry > OpenRecordMap;

        void writeDotNodes(std::ofstream & of);

        void writeDotEdges(std::ofstream & of);

        std::string generateCloseEdgeLabel(const CloseRecordMap::value_type & edge,
                const OpenRecordMap & openRecords,
                set< pair<const TimeStampedState*, const Operator*> > & handledTransitions);

        void writeDotEdgesAll(std::ofstream & of);
        void writeDotEdgesCondensed(std::ofstream & of);

        /// Generate a unique node name "state_XXXXXXXX" based on this state pointer.
        std::string generateNodeName(const TimeStampedState* state);

        /// Generate a new unused name.
        std::string generateAnonymousName();

        /// Generate a node label for this state using the actual state content.
        std::string generateNodeLabel(const TimeStampedState* state);

        /// If state cannot be stored, find a storable instance in the records.
        /// If that does not exist, create one.
        const TimeStampedState* findOrReplicateMatchingState(const TimeStampedState & state);

    protected:
        bool enabled;
        bool includeNumericalFluents;

        bool condenseEvents;    ///< events like opening and closing the same transition are one edge.

        unsigned int lastAnonymousNr;
        unsigned int currentEventNumber;

        /// Recorded states that are safe to store permanently.
        /// State -> Recorded Ptr. The ptr should be unique per state.
        typedef tr1::unordered_map<TimeStampedState, const TimeStampedState*, TssHash, TssEquals> RecordedStatesMap;
        RecordedStatesMap recordedStates;

        /// Some states might not exist permanently, stored them here to have a safe pointer.
        typedef tr1::unordered_set<TimeStampedState, TssHash, TssEquals> ReplicatedStatesSet;
        ReplicatedStatesSet replicatedStates;

        CloseRecordMap closedRecords;
        CloseRecordMap discardRecords;
        DiscardRecordMap moduleRelaxedDiscardRecords;
        DiscardRecordMap liveGroundingDiscardRecords;
        OpenRecordMap openRecords;
        map<const TimeStampedState*, int> goalRecords;
};

#endif

