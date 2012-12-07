#include "analysis.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <algorithm>
#include "operator.h"
#include "plannerParameters.h"
#include "closed_list.h"
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

static const string dot_class_state = "shape=box";
static const string dot_class_goal_state = "shape=box,color=green";
static const string dot_class_closed = "style=bold,color=black,weight=10";
static const string dot_class_discard = "color=gray,constraint=false";
// TODO switch those two on param
// Even better: switch if we generated the new state or if it is a discard reason!
//static const string dot_class_discard = "color=gray";
static const string dot_class_module_relaxed_discard = "style=dashed,color=gray,weight=3";
static const string dot_class_grounding_discard = "style=dotted,color=gray,weight=3";
static const string dot_class_open = "style=dashed,weight=3";

std::string formatString(const char* str, ...)
{
   va_list ap;
   va_start(ap, str);
   char* buf;
   if(vasprintf(&buf, str, ap) <= 0) {
      va_end(ap);
      return std::string();
   }
   va_end(ap);

   std::string ret = buf;
   free(buf);
   return ret;
}

std::size_t Analysis::TssHashTimestamp::operator()(const TimeStampedState & tss) const
{
    return tssHash(tss) + tss.timestamp;
}

bool Analysis::TssEqualsTimestamp::operator()(const TimeStampedState &tss1, const TimeStampedState &tss2) const
{
    if(!time_equals(tss1.timestamp, tss2.timestamp))
        return false;

    return tssEquals(tss1, tss2);
}

Analysis::Analysis() : enabled(false), includeNumericalFluents(false), condenseEvents(true),
    lastAnonymousNr(0), currentEventNumber(0)
{
}

bool Analysis::writeDot(const std::string & filePrefix)
{
    std::stringstream filename;
    filename << filePrefix << "_" << currentEventNumber << ".dot";
    ofstream of(filename.str().c_str());
    if(!of.good()) {
        return false;
    }
    of << "digraph StateSpace {" << endl;
    writeDotNodes(of);
    writeDotEdges(of);
    of << "}" << endl;

    of.close();
    return true;
}

void Analysis::recordClosingStep(const TimeStampedState* pred, const Operator* op,
        const TimeStampedState* succ)
{
    if(!enabled)
        return;

    currentEventNumber++;

    if(closedRecords.find(make_pair(pred, op)) != closedRecords.end()) {
        ROS_ERROR("Closing step for %s, %s already existed.", pred->toPDDL(true, true, true).c_str(),
                op->get_name().c_str());
        return;
    }

    if(pred != NULL)  {      // catch init state
        RecordedStatesMap::iterator it = recordedStates.find(*pred);
        if(it != recordedStates.end() && it->second != pred) {
            ROS_ERROR("Closing Step mismatched pred state: %s in: %08X, new: %08X",
                    pred->toPDDL(true, true, true).c_str(),
                    (unsigned int)(long)(long)it->second, (unsigned int)(long)pred);
        } else {
            recordedStates.insert(make_pair(*pred, pred));
        }
    }
    RecordedStatesMap::iterator it = recordedStates.find(*succ);
    if(it != recordedStates.end() && it->second != succ) {
        ROS_ERROR("Closing Step mismatched succ state: \n%s (%f)\n%08X\nin: \n%s (%f)\n%08X",
                succ->toPDDL(true, true, true).c_str(),
                succ->get_timestamp(),
                (unsigned int)(long)succ,
                it->second->toPDDL(true, true, true).c_str(),
                it->second->get_timestamp(),
                (unsigned int)(long)it->second);
    } else {
        recordedStates.insert(make_pair(*succ, succ));
    }
    closedRecords[make_pair(pred, op)] = make_pair(currentEventNumber, succ);
}

/**
 * Not sure what is correct now:
 *
 * We should have the back edge to a state that is already there, but we are discarding because of (even  if we are other Timestamp)?
 * We should have a new edge, if we are closing a new state for a better TS
 *
 */

void Analysis::recordDiscardingStep(const TimeStampedState* pred, const Operator* op,
        const TimeStampedState & succ, const ClosedList & closedList)
{
    if(!enabled)
        return;

    currentEventNumber++;
    if(discardRecords.find(make_pair(pred, op)) != discardRecords.end()) {
        ROS_ERROR("Discarding step for ..., %s already existed.", op->get_name().c_str());
        return;
    }

    RecordedStatesMap::iterator it = recordedStates.find(*pred);
    if(it != recordedStates.end() && it->second != pred) {
        ROS_ERROR("Discarding Step mismatched state: ... in: %08X, new: %08X",
                (unsigned int)(long)it->second, (unsigned int)(long)pred);
    } else {
        recordedStates.insert(make_pair(*pred, pred));
    }
    printf("%s\n", __func__);
    const TimeStampedState* discardedState = NULL;
    if(g_parameters.analyzeDiscardedStatesByReason) {
        // we discarded either because this equal state is closed (with better timestamp)
        // or because our timestamp is larger than the best plan
        if(closedList.contains(succ)) {
            const TimeStampedState & reason = closedList.get(succ);
            discardedState = &reason;   // safe to store - closed
        } else {    // not in closed list, we are new, but worse than the best plan
            discardedState = findOrReplicateMatchingState(succ, false);
        }
    } else {
            discardedState = findOrReplicateMatchingState(succ, false);
    }
    ROS_ASSERT(discardedState != NULL);
    discardRecords[make_pair(pred, op)] = make_pair(currentEventNumber,
            discardedState);
}

void Analysis::recordModuleRelaxedDiscardingStep(const TimeStampedState* pred, const Operator* op)
{
    if(!enabled)
        return;

    currentEventNumber++;

    if(moduleRelaxedDiscardRecords.find(make_pair(pred, op)) != moduleRelaxedDiscardRecords.end()) {
        ROS_ERROR("moduleRelaxedDiscarding step for ..., %s already existed.", op->get_name().c_str());
        return;
    }

    RecordedStatesMap::iterator it = recordedStates.find(*pred);
    if(it != recordedStates.end() && it->second != pred) {
        ROS_ERROR("moduleRelaxedDiscarding mismatched state: ... in: %08X, new: %08X",
                (unsigned int)(long)it->second, (unsigned int)(long)pred);
    } else {
        recordedStates.insert(make_pair(*pred, pred));
    }
    moduleRelaxedDiscardRecords[make_pair(pred, op)] = currentEventNumber;
}

void Analysis::recordGoal(const TimeStampedState & goalState)
{
    if(!enabled)
        return;

    currentEventNumber++;

    printf("%s\n", __func__);
    const TimeStampedState* goal = findOrReplicateMatchingState(goalState);
    if(goalRecords.find(goal) != goalRecords.end()) {
        ROS_ERROR("goal record for ... already existed.");
        return;
    }

    goalRecords[goal] = currentEventNumber;
}

void Analysis::recordOpenPush(const TimeStampedState* parent, const Operator* op,
        int openIndex, double priority)
{
    if(!enabled)
        return;

    currentEventNumber++;

    OpenRecordMap::iterator openRecordIt = openRecords.find(make_pair(parent, op));
    if(openRecordIt == openRecords.end()) {   // make sure we have an entry to push to
        openRecords[make_pair(parent, op)] = deque<OpenEntry>();
        openRecordIt = openRecords.find(make_pair(parent, op));
    }
    ROS_ASSERT(openRecordIt != openRecords.end());

    OpenEntry entry;
    entry.eventNumber = currentEventNumber;
    entry.openIndex = openIndex;
    entry.priority = priority;
    RecordedStatesMap::iterator it = recordedStates.find(*parent);
    if(it != recordedStates.end() && it->second != parent) {
        ROS_ERROR("open push mismatched state: ... in: %08X, new: %08X",
                (unsigned int)(long)it->second, (unsigned int)(long)parent);
    } else {
        recordedStates.insert(make_pair(*parent, parent));
    }
    openRecordIt->second.push_back(entry);
}

void Analysis::recordLiveGroundingDiscard(const TimeStampedState* pred, const Operator* op)
{
    if(!enabled)
        return;

    currentEventNumber++;
    if(liveGroundingDiscardRecords.find(make_pair(pred, op)) != liveGroundingDiscardRecords.end()) {
        ROS_ERROR("liveGroundingDiscarding for ..., %s already existed.", op->get_name().c_str());
        return;
    }

    RecordedStatesMap::iterator it = recordedStates.find(*pred);
    if(it != recordedStates.end() && it->second != pred) {
        ROS_ERROR("liveGroundingDiscarding mismatched state: ... in: %08X, new: %08X",
                (unsigned int)(long)it->second, (unsigned int)(long)pred);
    } else {
        recordedStates.insert(make_pair(*pred, pred));
    }
    liveGroundingDiscardRecords[make_pair(pred, op)] = currentEventNumber;
}

void Analysis::writeDotNodes(std::ofstream & of)
{
    // collect all nodes first
    of << "node[" << dot_class_state << "]" << endl;
    set<const TimeStampedState*> all_states;
    forEach(CloseRecordMap::value_type & vt, closedRecords) {
        if(vt.first.first != NULL)      // catch init
            all_states.insert(vt.first.first);
        ROS_ASSERT(vt.second.second);
        all_states.insert(vt.second.second);
    }
    forEach(CloseRecordMap::value_type & vt, discardRecords) {
        ROS_ASSERT(vt.first.first);
        all_states.insert(vt.first.first);
        ROS_ASSERT(vt.second.second);
        all_states.insert(vt.second.second);
    }
    forEach(DiscardRecordMap::value_type & vt, moduleRelaxedDiscardRecords) {
        ROS_ASSERT(vt.first.first);
        all_states.insert(vt.first.first);
    }
    forEach(DiscardRecordMap::value_type & vt, liveGroundingDiscardRecords) {
        ROS_ASSERT(vt.first.first);
        all_states.insert(vt.first.first);
    }
    forEach(OpenRecordMap::value_type & vt, openRecords) {
        ROS_ASSERT(vt.first.first);
        all_states.insert(vt.first.first);
    }
    forEach(const TimeStampedState* s, all_states) {
        if(goalRecords.find(s) != goalRecords.end()) {
            of << generateNodeName(s) << "[label=\"" << goalRecords[s] << ": "
                << generateNodeLabel(s) << "\"," <<
                dot_class_goal_state << "]" << endl;
        } else {
            of << generateNodeName(s) << "[label=\"" << generateNodeLabel(s) << "\"]" << endl;
        }
    }

    // FIXME: Do we need to re-consolidate states that were properly closed later on
    // and might have been replicated here?
}

void Analysis::writeDotEdges(std::ofstream & of)
{
    if(condenseEvents)
        writeDotEdgesCondensed(of);
    else 
        writeDotEdgesAll(of);
}

std::string Analysis::generateDiscardEdgeLabel(const DiscardRecordMap::value_type & edge,
        const OpenRecordMap & openRecords,
        set< pair<const TimeStampedState*, const Operator*> > & handledTransitions)
{
    // collect matching open transitions
    vector<OpenEntry> matchingOpenTransitions;
    forEach(const OpenRecordMap::value_type & ort, openRecords) {
        if(edge.first != ort.first)
            continue;

        forEach(OpenEntry oe, ort.second)
            matchingOpenTransitions.push_back(oe);
        handledTransitions.insert(ort.first);
    }
    sort(matchingOpenTransitions.begin(), matchingOpenTransitions.end());

    stringstream ss;
    // print (openEv1, openEv2 - closeEv)
    bool first = true;
    forEach(const OpenEntry & oe, matchingOpenTransitions) {
        if(first)
            first = false;
        else
            ss << ", ";
        ss << oe.eventNumber;
    }
    // actual edge label: the op name
    ss << " -> " << edge.second << ": " << edge.first.second->get_name();
    ss << " ";
    // print (openId1, prior1), (openId2, prior2)
    first = true;
    forEach(const OpenEntry & oe, matchingOpenTransitions) {
        if(first)
            first = false;
        else
            ss << ", ";
        ss << "(" << oe.openIndex << ", " << std::fixed << std::setprecision(2) << oe.priority << ")";
    }
    return ss.str();
}

std::string Analysis::generateCloseEdgeLabel(const CloseRecordMap::value_type & edge,
        const OpenRecordMap & openRecords,
        set< pair<const TimeStampedState*, const Operator*> > & handledTransitions)
{
    // collect matching open transitions
    vector<OpenEntry> matchingOpenTransitions;
    forEach(const OpenRecordMap::value_type & ort, openRecords) {
        if(edge.first != ort.first)
            continue;

        forEach(OpenEntry oe, ort.second)
            matchingOpenTransitions.push_back(oe);
        handledTransitions.insert(ort.first);
    }
    sort(matchingOpenTransitions.begin(), matchingOpenTransitions.end());

    stringstream ss;
    // print (openEv1, openEv2 - closeEv)
    bool first = true;
    forEach(const OpenEntry & oe, matchingOpenTransitions) {
        if(first)
            first = false;
        else
            ss << ", ";
        ss << oe.eventNumber;
    }
    // actual edge label: the op name
    ss << " -> " << edge.second.first << ": " << edge.first.second->get_name();
    ss << " ";
    // print (openId1, prior1), (openId2, prior2)
    first = true;
    forEach(const OpenEntry & oe, matchingOpenTransitions) {
        if(first)
            first = false;
        else
            ss << ", ";
        ss << "(" << oe.openIndex << ", " << std::fixed << std::setprecision(2) << oe.priority << ")";
    }
    return ss.str();
}

std::string Analysis::generateOpenEdgeLabel(const OpenRecordMap::value_type & edge)
{
    deque<OpenEntry> pushes = edge.second;
    sort(pushes.begin(), pushes.end());
    stringstream ss;
    bool first = true;
    forEach(const OpenEntry & oe, pushes) {
        if(first)
            first = false;
        else
            ss << ", ";
        ss << oe.eventNumber;
    }
    ss << ": " << edge.first.second->get_name();

    first = true;
    forEach(const OpenEntry & oe, pushes) {
        if(first)
            first = false;
        else
            ss << ", ";
        ss << "(" << oe.openIndex << ", " << std::fixed << std::setprecision(2) << oe.priority << ")";
    }

    return ss.str();
}

void Analysis::writeDotEdgesCondensed(std::ofstream & of)
{
    set< pair<const TimeStampedState*, const Operator*> > handledTransitions;

    lastAnonymousNr = 0;

    // go through all the possible edges
    // for each edge collect all other possibilities and annotations
    // for every next edge type, make sure we haven't done that already.
    forEach(CloseRecordMap::value_type & vt, closedRecords) {
        if(vt.first.first == NULL) {
            of << "pre_init[shape=point,label=\"\"]" << endl;
            of << "pre_init -> " << generateNodeName(vt.second.second);
            of << " [label=\"";
            of << vt.second.first << "\"]" << endl;
            continue;
        }   // catch init

        of << generateNodeName(vt.first.first) << " -> " << generateNodeName(vt.second.second);

        of << " [label=\"";
        of << generateCloseEdgeLabel(vt, openRecords, handledTransitions);
        of << "\"," << dot_class_closed << "]" << endl;
    }

    forEach(CloseRecordMap::value_type & vt, discardRecords) {
        of << generateNodeName(vt.first.first) << " -> " << generateNodeName(vt.second.second);

        of << " [label=\"";
        of << generateCloseEdgeLabel(vt, openRecords, handledTransitions);
        of << "\"," << dot_class_discard << "]" << endl;
    }

    // TODO consolidation for grounding possible somehow???
    forEach(DiscardRecordMap::value_type & vt, moduleRelaxedDiscardRecords) {
        std::string invalidNode = createAnonymousNode(of);
        of << generateNodeName(vt.first.first) << " -> " << invalidNode;
        of << " [label=\"";
        of << generateDiscardEdgeLabel(vt, openRecords, handledTransitions);
        of << "\"," << dot_class_module_relaxed_discard << "]" << endl;
    }
    forEach(DiscardRecordMap::value_type & vt, liveGroundingDiscardRecords) {
        std::string invalidNode = createAnonymousNode(of);
        of << generateNodeName(vt.first.first) << " -> " << invalidNode;
        of << " [label=\"";
        of << generateDiscardEdgeLabel(vt, openRecords, handledTransitions);
        of << "\"," << dot_class_grounding_discard << "]" << endl;
    }

    forEach(OpenRecordMap::value_type & vt, openRecords) {
        if(handledTransitions.find(vt.first) != handledTransitions.end())
            continue;
        std::string invalidNode = createAnonymousNode(of);
        of << generateNodeName(vt.first.first) << " -> " << invalidNode;
        of << " [label=\"";
        of << generateOpenEdgeLabel(vt);
        //stringstream ss;
        //ss << vt.second.eventNumber << ": " << vt.first.second->get_name();
        //ss << " (" << vt.second.openIndex << ", " << vt.second.priority << ")";
        //of << ss.str();
        of << "\"," << dot_class_open << "]" << endl;
    }
}

void Analysis::writeDotEdgesAll(std::ofstream & of)
{
    // allow multiple edges (open edges might lead to nowhere/anon nodes)
    // i.e. draw opening and closing transitions as separate edges

    lastAnonymousNr = 0;

    forEach(CloseRecordMap::value_type & vt, closedRecords) {
        if(vt.first.first == NULL) {
            of << "pre_init[shape=point,label=\"\"]" << endl;
            of << "pre_init -> " << generateNodeName(vt.second.second);
            of << " [label=\"";
            of << vt.second.first << "\"]" << endl;
            continue;
        }   // catch init

        of << generateNodeName(vt.first.first) << " -> " << generateNodeName(vt.second.second);
        of << " [label=\"";
        stringstream ss;
        ss << vt.second.first << ": " << vt.first.second->get_name();
        of << ss.str();
        of << "\"," << dot_class_closed << "]" << endl;
    }

    forEach(CloseRecordMap::value_type & vt, discardRecords) {
        of << generateNodeName(vt.first.first) << " -> " << generateNodeName(vt.second.second);
        of << " [label=\"";
        stringstream ss;
        ss << vt.second.first << ": " << vt.first.second->get_name();
        of << ss.str();
        of << "\"," << dot_class_discard << "]" << endl;
    }

    forEach(DiscardRecordMap::value_type & vt, moduleRelaxedDiscardRecords) {
        std::string invalidNode = createAnonymousNode(of);
        of << generateNodeName(vt.first.first) << " -> " << invalidNode;
        of << " [label=\"";
        stringstream ss;
        ss << vt.second << ": " << vt.first.second->get_name();
        of << ss.str();
        of << "\"," << dot_class_module_relaxed_discard << "]" << endl;
    }
    forEach(DiscardRecordMap::value_type & vt, liveGroundingDiscardRecords) {
        std::string invalidNode = createAnonymousNode(of);
        of << generateNodeName(vt.first.first) << " -> " << invalidNode;
        of << " [label=\"";
        stringstream ss;
        ss << vt.second << ": " << vt.first.second->get_name();
        of << ss.str();
        of << "\"," << dot_class_grounding_discard << "]" << endl;
    }

    forEach(OpenRecordMap::value_type & vt, openRecords) {
        std::string openNode = createAnonymousNode(of);
        of << generateNodeName(vt.first.first) << " -> " << openNode;
        of << " [label=\"";
        of << generateOpenEdgeLabel(vt);
        //stringstream ss;
        //ss << vt.second.eventNumber << ": " << vt.first.second->get_name();
        //ss << " (" << vt.second.openIndex << ", " << vt.second.priority << ")";
        //of << ss.str();
        of << "\"," << dot_class_open << "]" << endl;
    }
}

std::string Analysis::generateNodeName(const TimeStampedState* state)
{
    return formatString("state_%08X", state);
}

std::string Analysis::createAnonymousNode(std::ofstream & of)
{
    lastAnonymousNr++;
    std::string name = formatString("invalid_state_anon_%09d", lastAnonymousNr);

    of << name << "[shape=doublecircle,label=\"\"]" << endl;
    return name;
}

std::string Analysis::generateNodeLabel(const TimeStampedState* state)
{
    stringstream ss;
    // symbolic/numeric optional
    std::string stateStr = state->toPDDL(true, true, includeNumericalFluents, " ", 30);

    string::size_type pos = stateStr.find("\n");
    while(pos != string::npos) {
        stateStr.replace(pos, 1, "\\n");
        pos = stateStr.find("\n");
    }

    ss << state->get_timestamp() << ": " << stateStr;
    return ss.str();
}

const TimeStampedState* Analysis::findOrReplicateMatchingState(const TimeStampedState & state,
        bool warnIfReplicated)
{
    // first check the recorded ones from planning
    RecordedStatesMap::iterator it = recordedStates.find(state);
    if(it != recordedStates.end()) {
        ROS_INFO("findOrReplicateMatchingState: found recorded state.");
        return it->second;
    }

    // next check if we already replicated that
    ReplicatedStatesSet::iterator replIt = replicatedStates.find(state);
    if(replIt != replicatedStates.end()) {
        ROS_INFO("findOrReplicateMatchingState: found replicated state.");
        return &(*replIt);
    }

    if(warnIfReplicated)
        ROS_WARN("findOrReplicateMatchingState new entry for %s", state.toPDDL(true, true, true).c_str());

    // finally: we didn't, so actually create this new one.
    pair<ReplicatedStatesSet::iterator, bool> insIt = replicatedStates.insert(state);
    return &(*(insIt.first));
}

