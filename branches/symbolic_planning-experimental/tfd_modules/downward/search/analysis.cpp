#include "analysis.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <algorithm>
#include "operator.h"
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

static const string dot_class_state = "shape=box";
static const string dot_class_goal_state = "shape=box,color=green";
static const string dot_class_closed = "style=bold,color=black";
static const string dot_class_discard = "color=gray,constraint=false";
static const string dot_class_module_relaxed_discard = "style=dashed,color=gray";
static const string dot_class_grounding_discard = "style=dotted,color=gray";
static const string dot_class_open = "style=dashed";

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
        ROS_ERROR("Closing Step mismatched succ state: %s in: %08X, new: %08X",
                succ->toPDDL(true, true, true).c_str(),
                (unsigned int)(long)it->second, (unsigned int)(long)succ);
    } else {
        recordedStates.insert(make_pair(*succ, succ));
    }
    closedRecords[make_pair(pred, op)] = make_pair(currentEventNumber, succ);
}

void Analysis::recordDiscardingStep(const TimeStampedState* pred, const Operator* op,
        const TimeStampedState & succ)
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
    discardRecords[make_pair(pred, op)] = make_pair(currentEventNumber,
            findOrReplicateMatchingState(succ));
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

    if(openRecords.find(make_pair(parent, op)) != openRecords.end()) {
        ROS_ERROR("open push for ..., %s already existed.", op->get_name().c_str());
        return;
    }

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
    openRecords[make_pair(parent, op)] = entry;
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

    // TODO
    // re-consolidate states that were properly closed later on and might have been
    // replicated here
    // Can this later stuff actually happen? Maybe for condesned mode??? but we almost
    // never generate, do we? Do we ever??? CHECK

    // check which have good names, for others invent/generate some (maybe keep them when
    // in recording already?), but name generated ones differently (_anon)
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

        matchingOpenTransitions.push_back(ort.second);
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

        matchingOpenTransitions.push_back(ort.second);
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

    // can debug consolidation here with no manip planner running
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
        stringstream ss;
        ss << vt.second.eventNumber << ": " << vt.first.second->get_name();
        ss << " (" << vt.second.openIndex << ", " << vt.second.priority << ")";
        of << ss.str();
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
        // TODO mark as irrelevant for sorting
    }

    // try to enforce ordering by node depth (and ignore backlinks for that)

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
        stringstream ss;
        ss << vt.second.eventNumber << ": " << vt.first.second->get_name();
        ss << " (" << vt.second.openIndex << ", " << vt.second.priority << ")";
        of << ss.str();
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

const TimeStampedState* Analysis::findOrReplicateMatchingState(const TimeStampedState & state)
{
    // first check the recorded ones from planning
    RecordedStatesMap::iterator it = recordedStates.find(state);
    if(it != recordedStates.end()) {
        return it->second;
    }

    // next check if we already replicated that
    ReplicatedStatesSet::iterator replIt = replicatedStates.find(state);
    if(replIt != replicatedStates.end()) {
        return &(*replIt);
    }

    // finally: we didn't, so actually create this new one.
    pair<ReplicatedStatesSet::iterator, bool> insIt = replicatedStates.insert(state);
    return &(*(insIt.first));
}

