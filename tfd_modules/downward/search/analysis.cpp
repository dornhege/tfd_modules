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
#include "tfd_modules/opl/stringutil.h"
#define forEach BOOST_FOREACH

static const string dot_class_state = "shape=box";
static const string dot_class_goal_state = "shape=box,color=green";

static const string dot_class_state_equal = "style=dashed,constraint=false,dir=none";

static const string dot_class_closed = "style=bold,color=black,weight=10";
static const string dot_class_discard = "color=gray,constraint=false";
/// discard should be drawn with constraint if there is no discard reason (i.e. it is not a back arrow)
static const string dot_class_discard_constraint = "color=gray";
static const string dot_class_module_relaxed_discard = "style=dashed,color=gray,weight=3";
static const string dot_class_grounding = "style=dotted,weight=3";
static const string dot_class_grounding_discard = "style=dotted,color=gray,weight=3";
static const string dot_class_grounding_grounded_out = "style=solid,color=black,weight=3,arrowhead=tee";
static const string dot_class_grounding_ungrounded_discard = "style=solid,color=gray,weight=3,arrowhead=tee";
static const string dot_class_open = "style=dashed,weight=3";

static const string dot_node_shape_grounding = "circle";
static const string dot_node_shape_grounded_out = "none";
static const string dot_node_shape_ungrounded_discard = "none";
static const string dot_node_shape_ungrounded_intermediate = "triangle";
static const string dot_node_shape_open = "doublecircle";
static const string dot_node_shape_module_relaxed_discard = "circle";
static const string dot_node_shape_live_grounding_discard = "circle";

static const unsigned int op_name_max_length = 20;

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

Analysis::Analysis() : lastAnonymousNr(0), currentEventNumber(0)
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
    writeDotEqualStates(of);
    of << "}" << endl;

    of.close();
    return true;
}

void Analysis::recordClosingStep(const TimeStampedState* pred, const Operator* op,
        const TimeStampedState* succ)
{
    if(!g_parameters.analyze)
        return;

    currentEventNumber++;
    if(pred == NULL)
        ROS_DEBUG_NAMED("analyze", "%s: %d INIT", __func__, currentEventNumber);
    else
        ROS_DEBUG_NAMED("analyze", "%s: %d for %s\n%s", __func__, currentEventNumber,
                pred->toPDDL(true, true, true).c_str(), op->get_name().c_str());

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

void Analysis::recordDiscardingStep(const TimeStampedState* pred, const Operator* op,
        const TimeStampedState & succ, const ClosedList & closedList)
{
    if(!g_parameters.analyze)
        return;

    currentEventNumber++;
    ROS_DEBUG_NAMED("analyze", "%s: %d for %s\n%s", __func__, currentEventNumber,
            pred->toPDDL(true, true, true).c_str(), op->get_name().c_str());

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
    if(!g_parameters.analyze)
        return;

    currentEventNumber++;
    ROS_DEBUG_NAMED("analyze", "%s: %d for %s\n%s", __func__, currentEventNumber,
            pred->toPDDL(true, true, true).c_str(), op->get_name().c_str());

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
    if(!g_parameters.analyze)
        return;

    currentEventNumber++;
    ROS_DEBUG_NAMED("analyze", "%s: %d for %s", __func__, currentEventNumber,
            goalState.toPDDL(true, true, true).c_str());

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
    if(!g_parameters.analyze)
        return;

    currentEventNumber++;
    ROS_DEBUG_NAMED("analyze", "%s: %d for %s\n%s", __func__, currentEventNumber,
            parent->toPDDL(true, true, true).c_str(), op->get_name().c_str());

    OpenRecordMap::iterator openRecordIt = openRecords.find(make_pair(parent, op));
    if(openRecordIt == openRecords.end()) {   // make sure we have an entry to push to
        openRecords[make_pair(parent, op)] = deque<OpenEntry>();
        openRecordIt = openRecords.find(make_pair(parent, op));
    } else {
        if(g_parameters.grounding_mode != PlannerParameters::GroundSingleReinsert) {
            // consistency check: the same open push for the same queue shouldn't happen twice.
            set<int> queuesPushed;
            forEach(const OpenEntry & oe, openRecordIt->second) {
                if(queuesPushed.find(oe.openIndex) != queuesPushed.end()) {
                    ROS_ERROR("Multiple Open Push for same queue: %d at state %s\nop: %s", oe.openIndex,
                            parent->toPDDL(true, true, true).c_str(), op->get_name().c_str());
                }
                queuesPushed.insert(oe.openIndex);
            }
        }

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
    if(!g_parameters.analyze)
        return;

    currentEventNumber++;
    ROS_DEBUG_NAMED("analyze", "%s: %d for %s\n%s", __func__, currentEventNumber,
            pred->toPDDL(true, true, true).c_str(), op->get_name().c_str());

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

void Analysis::recordLiveGroundingGroundedOut(const TimeStampedState* pred, const Operator* op)
{
    if(!g_parameters.analyze)
        return;

    currentEventNumber++;
    ROS_DEBUG_NAMED("analyze", "%s: %d for %s\n%s", __func__, currentEventNumber,
            pred->toPDDL(true, true, true).c_str(), op->get_name().c_str());


    if(operatorGroundedOutRecords.find(make_pair(pred, op)) != operatorGroundedOutRecords.end()) {
        ROS_ERROR("recordLiveGroundedOut for ..., %s already existed.", op->get_name().c_str());
        return;
    }

    RecordedStatesMap::iterator it = recordedStates.find(*pred);
    if(it != recordedStates.end() && it->second != pred) {
        ROS_ERROR("recordLiveGroundedOut mismatched state: ... in: %08X, new: %08X",
                (unsigned int)(long)it->second, (unsigned int)(long)pred);
    } else {
        recordedStates.insert(make_pair(*pred, pred));
    }
    operatorGroundedOutRecords[make_pair(pred, op)] = currentEventNumber;
}

void Analysis::recordLiveGroundingUngroundedDiscard(const TimeStampedState* pred, const Operator* op)
{
    if(!g_parameters.analyze)
        return;

    currentEventNumber++;
    ROS_DEBUG_NAMED("analyze", "%s: %d for %s\n%s", __func__, currentEventNumber,
            pred->toPDDL(true, true, true).c_str(), op->get_name().c_str());

    if(ungroundedOpDiscardRecords.find(make_pair(pred, op)) != ungroundedOpDiscardRecords.end()) {
        ROS_ERROR("%s for ..., %s already existed.", __func__, op->get_name().c_str());
        return;
    }

    RecordedStatesMap::iterator it = recordedStates.find(*pred);
    if(it != recordedStates.end() && it->second != pred) {
        ROS_ERROR("recordLiveGroundedOut mismatched state: ... in: %08X, new: %08X",
                (unsigned int)(long)it->second, (unsigned int)(long)pred);
    } else {
        recordedStates.insert(make_pair(*pred, pred));
    }
    ungroundedOpDiscardRecords[make_pair(pred, op)] = currentEventNumber;
}

void Analysis::recordLiveGrounding(const TimeStampedState* pred, const Operator* op)
{
    if(!g_parameters.analyze)
        return;

    currentEventNumber++;
    ROS_DEBUG_NAMED("analyze", "%s: %d for %s\n%s", __func__, currentEventNumber,
            pred->toPDDL(true, true, true).c_str(), op->get_name().c_str());

    if(operatorGroundingRecords.find(make_pair(pred, op)) != operatorGroundingRecords.end()) {
        ROS_ERROR("recordLiveGrounding for ..., %s already existed.", op->get_name().c_str());
        return;
    }

    RecordedStatesMap::iterator it = recordedStates.find(*pred);
    if(it != recordedStates.end() && it->second != pred) {
        ROS_ERROR("recordLiveGrounding mismatched state: ... in: %08X, new: %08X",
                (unsigned int)(long)it->second, (unsigned int)(long)pred);
    } else {
        recordedStates.insert(make_pair(*pred, pred));
    }
    operatorGroundingRecords[make_pair(pred, op)] = currentEventNumber;
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
    forEach(EventRecordMap::value_type & vt, moduleRelaxedDiscardRecords) {
        ROS_ASSERT(vt.first.first);
        all_states.insert(vt.first.first);
    }
    forEach(EventRecordMap::value_type & vt, liveGroundingDiscardRecords) {
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

    // write intermediate nodes for ungrounded ops in a state, i.e. state_STATE_PTR_OP_PTR

    set<pair<const TimeStampedState*, const Operator*> > ungroundedIntermediateStates;
    typedef set<pair<const TimeStampedState*, const Operator*> >::value_type StateOp;
    forEach(OpenRecordMap::value_type & vt, openRecords) {
        if(vt.first.second->isGrounded())
            continue;
        ungroundedIntermediateStates.insert(vt.first);
    }
 
    forEach(const StateOp & sop, ungroundedIntermediateStates) {
        of << generateUngroundedOpNodeName(sop) << "[label=\"\",shape="
            << dot_node_shape_ungrounded_intermediate << "]" << endl;
    }
}

void Analysis::writeDotEdges(std::ofstream & of)
{
    if(g_parameters.analyzeCondensedOutput)
        writeDotEdgesCondensed(of);
    else 
        writeDotEdgesAll(of);
}

std::string Analysis::generateModuleDiscardEdgeLabel(const EventRecordMap::value_type & edge,
        const OpenRecordMap & openRecords,
        set< pair<const TimeStampedState*, const Operator*> > & handledTransitions)
{
    ROS_ASSERT(edge.first.second->getGroundingParent() == NULL);

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
    ss << " -> " << edge.second << ": "
        << breakStringLabel(edge.first.second->get_name(), op_name_max_length);
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

std::string Analysis::generateLiveGroundingDiscardEdgeLabel(const EventRecordMap::value_type & edge,
        const OpenRecordMap & openRecords,
        set< pair<const TimeStampedState*, const Operator*> > & handledTransitions)
{
    ROS_ASSERT(edge.first.second->getGroundingParent() != NULL);

    // collect matching operatorGroundingRecords
    int matchingGroundingEvent = -1;
    forEach(const EventRecordMap::value_type & er, operatorGroundingRecords) {
        if(edge.first != er.first)
            continue;

        // there should only be one grounding per state/ground_op
        ROS_ASSERT(matchingGroundingEvent == -1);

        matchingGroundingEvent = er.second;
    }
    ROS_ASSERT(matchingGroundingEvent != -1);   // we should have an event.

    // actual edge label: the op name
    stringstream ss;
    ss << matchingGroundingEvent << " -> " << edge.second << ": "
        << breakStringLabel(edge.first.second->get_name(), op_name_max_length);
    return ss.str();
}

std::string Analysis::generateCloseEdgeLabel(const CloseRecordMap::value_type & edge,
        const OpenRecordMap & openRecords,
        set< pair<const TimeStampedState*, const Operator*> > & handledTransitions)
{
    stringstream ss;
    vector<OpenEntry> matchingOpenTransitions;

    if(g_parameters.grounding_mode == PlannerParameters::GroundSingleReinsert &&
            edge.first.second->getGroundingParent() != NULL) {
        // collect matching operatorGroundingRecords
        int matchingGroundingEvent = -1;
        forEach(const EventRecordMap::value_type & er, operatorGroundingRecords) {
            if(edge.first != er.first)
                continue;

            // there should only be one grounding per state/ground_op
            ROS_ASSERT(matchingGroundingEvent == -1);

            matchingGroundingEvent = er.second;
        }
        ROS_ASSERT(matchingGroundingEvent != -1);   // we should have an event.
        ss << matchingGroundingEvent;
    } else {
        // collect matching open transitions
        forEach(const OpenRecordMap::value_type & ort, openRecords) {
            if(edge.first != ort.first)
                continue;

            forEach(OpenEntry oe, ort.second)
                matchingOpenTransitions.push_back(oe);
            handledTransitions.insert(ort.first);
        }
        sort(matchingOpenTransitions.begin(), matchingOpenTransitions.end());

        // print (openEv1, openEv2 - closeEv)
        bool first = true;
        forEach(const OpenEntry & oe, matchingOpenTransitions) {
            if(first)
                first = false;
            else
                ss << ", ";
            ss << oe.eventNumber;
        }
    }

    // actual edge label: the op name
    ss << " -> " << edge.second.first << ": "
        << breakStringLabel(edge.first.second->get_name(), op_name_max_length);

    if(g_parameters.grounding_mode != PlannerParameters::GroundSingleReinsert ||
            edge.first.second->getGroundingParent() == NULL) {
        ss << " ";
        // print (openId1, prior1), (openId2, prior2)
        bool first = true;
        forEach(const OpenEntry & oe, matchingOpenTransitions) {
            if(first)
                first = false;
            else
                ss << ", ";
            ss << "(" << oe.openIndex << ", " << std::fixed << std::setprecision(2) << oe.priority << ")";
        }
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
    ss << ": " << breakStringLabel(edge.first.second->get_name(), op_name_max_length);

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

        std::string headnode;
        bool wasLiveGrounded = false;
        if(g_parameters.grounding_mode == PlannerParameters::GroundSingleReinsert &&
                vt.first.second->getGroundingParent() != NULL)
            wasLiveGrounded = true;
        if(!wasLiveGrounded) {   // this op wasn't live grounded
            headnode = generateNodeName(vt.first.first);
        } else {        // open push to ungrounded op
            headnode = generateUngroundedOpNodeName(
                    make_pair(vt.first.first, vt.first.second->getGroundingParent()));
        }

        of << headnode << " -> " << generateNodeName(vt.second.second);

        of << " [label=\"";
        of << generateCloseEdgeLabel(vt, openRecords, handledTransitions);
        of << "\"," << dot_class_closed << "]" << endl;
    }

    forEach(CloseRecordMap::value_type & vt, discardRecords) {
        std::string headnode;
        bool wasLiveGrounded = false;
        if(g_parameters.grounding_mode == PlannerParameters::GroundSingleReinsert &&
                vt.first.second->getGroundingParent() != NULL)
            wasLiveGrounded = true;
        if(!wasLiveGrounded) {   // this op wasn't live grounded
            headnode = generateNodeName(vt.first.first);
        } else {        // open push to ungrounded op
            headnode = generateUngroundedOpNodeName(
                    make_pair(vt.first.first, vt.first.second->getGroundingParent()));
        }

        of << headnode << " -> " << generateNodeName(vt.second.second);

        of << " [label=\"";
        of << generateCloseEdgeLabel(vt, openRecords, handledTransitions);
        if(isReplicated(vt.second.second))
            of << "\"," << dot_class_discard_constraint << "]" << endl;
        else
            of << "\"," << dot_class_discard << "]" << endl;
    }

    forEach(EventRecordMap::value_type & vt, moduleRelaxedDiscardRecords) {
        std::string invalidNode = createAnonymousNode(of, dot_node_shape_module_relaxed_discard);
        of << generateNodeName(vt.first.first) << " -> " << invalidNode;
        of << " [label=\"";
        of << generateModuleDiscardEdgeLabel(vt, openRecords, handledTransitions);
        of << "\"," << dot_class_module_relaxed_discard << "]" << endl;
    }
    forEach(EventRecordMap::value_type & vt, liveGroundingDiscardRecords) {
        std::string invalidNode = createAnonymousNode(of, dot_node_shape_live_grounding_discard);
        of << generateUngroundedOpNodeName(
                make_pair(vt.first.first, vt.first.second->getGroundingParent()))
                << " -> " << invalidNode;
        of << " [label=\"";
        of << generateLiveGroundingDiscardEdgeLabel(vt, openRecords, handledTransitions);
        of << "\"," << dot_class_grounding_discard << "]" << endl;
    }
    forEach(EventRecordMap::value_type & vt, operatorGroundedOutRecords) {
        std::string invalidNode = createAnonymousNode(of, dot_node_shape_grounded_out);
        of << generateUngroundedOpNodeName(
                make_pair(vt.first.first, vt.first.second))
                << " -> " << invalidNode;
        of << " [label=\"";
        of << vt.second;
        of << "\"," << dot_class_grounding_grounded_out << "]" << endl;
    }
    forEach(EventRecordMap::value_type & vt, ungroundedOpDiscardRecords) {
        std::string invalidNode = createAnonymousNode(of, dot_node_shape_ungrounded_discard);
        // it can happen that there is no ungrounded op node, because there actually never
        // was an open push for this ungrounded op. So this ungrounded discard happens
        // immediately when we try to push (e.g. try to push ungrounded op, there
        // already is a better plan -> discard it immediately)
        // In this case, there shouldn't be an open entry for state + ungrounded op, then
        // just take the state as the origin node.
        bool hasOpenPush = false;
        forEach(OpenRecordMap::value_type & oe, openRecords) {
            if(vt.first == oe.first) {
                hasOpenPush = true;
                break;
            }
        }
        if(hasOpenPush){
            of << generateUngroundedOpNodeName(make_pair(vt.first.first, vt.first.second));
        } else {
            of << generateNodeName(vt.first.first);
        }
        of << " -> " << invalidNode;
        of << " [label=\"";
        of << vt.second;
        if(!hasOpenPush) {
            of << ": " << breakStringLabel(vt.first.second->get_name(), op_name_max_length);
        }
        of << "\"," << dot_class_grounding_ungrounded_discard << "]" << endl;
    }

    forEach(OpenRecordMap::value_type & vt, openRecords) {
        if(handledTransitions.find(vt.first) != handledTransitions.end())
            continue;
        std::string node;
        if(vt.first.second->isGrounded()) {   // open push that wasn't closed/discarded above
            node = createAnonymousNode(of, dot_node_shape_open);
        } else {        // open push to ungrounded op
            node = generateUngroundedOpNodeName(vt.first);
        }
        of << generateNodeName(vt.first.first) << " -> " << node;
        of << " [label=\"";
        of << generateOpenEdgeLabel(vt);
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

        std::string headnode;
        bool wasLiveGrounded = false;
        if(g_parameters.grounding_mode == PlannerParameters::GroundSingleReinsert &&
                vt.first.second->getGroundingParent() != NULL)
            wasLiveGrounded = true;
        if(!wasLiveGrounded) {   // this op wasn't live grounded
            headnode = generateNodeName(vt.first.first);
        } else {        // open push to ungrounded op
            headnode = generateUngroundedOpNodeName(
                    make_pair(vt.first.first, vt.first.second->getGroundingParent()));
        }

        of << headnode << " -> " << generateNodeName(vt.second.second);

        of << " [label=\"";
        stringstream ss;
        ss << vt.second.first << ": "
            << breakStringLabel(vt.first.second->get_name(), op_name_max_length);
        of << ss.str();
        of << "\"," << dot_class_closed << "]" << endl;
    }

    forEach(CloseRecordMap::value_type & vt, discardRecords) {
        std::string headnode;
        bool wasLiveGrounded = false;
        if(g_parameters.grounding_mode == PlannerParameters::GroundSingleReinsert &&
                vt.first.second->getGroundingParent() != NULL)
            wasLiveGrounded = true;
        if(!wasLiveGrounded) {   // this op wasn't live grounded
            headnode = generateNodeName(vt.first.first);
        } else {        // open push to ungrounded op
            headnode = generateUngroundedOpNodeName(
                    make_pair(vt.first.first, vt.first.second->getGroundingParent()));
        }

        of << headnode << " -> " << generateNodeName(vt.second.second);

        of << " [label=\"";
        stringstream ss;
        ss << vt.second.first << ": "
            << breakStringLabel(vt.first.second->get_name(), op_name_max_length);
        of << ss.str();
        of << "\"," << dot_class_discard_constraint << "]" << endl;
    }

    forEach(EventRecordMap::value_type & vt, moduleRelaxedDiscardRecords) {
        std::string invalidNode = createAnonymousNode(of, dot_node_shape_module_relaxed_discard);
        of << generateNodeName(vt.first.first) << " -> " << invalidNode;
        of << " [label=\"";
        stringstream ss;
        ss << vt.second << ": "
            << breakStringLabel(vt.first.second->get_name(), op_name_max_length);
        of << ss.str();
        of << "\"," << dot_class_module_relaxed_discard << "]" << endl;
    }

    forEach(EventRecordMap::value_type & vt, operatorGroundingRecords) {
        std::string invalidNode = createAnonymousNode(of, dot_node_shape_grounding);
        of << generateUngroundedOpNodeName(
                make_pair(vt.first.first, vt.first.second->getGroundingParent()))
                << " -> " << invalidNode;
        of << " [label=\"";
        of << vt.second << ": " << breakStringLabel(vt.first.second->get_name(), op_name_max_length);
        of << "\"," << dot_class_grounding << "]" << endl;
    }
    forEach(EventRecordMap::value_type & vt, liveGroundingDiscardRecords) {
        std::string invalidNode = createAnonymousNode(of, dot_node_shape_live_grounding_discard);
        of << generateUngroundedOpNodeName(
                make_pair(vt.first.first, vt.first.second->getGroundingParent()))
                << " -> " << invalidNode;
        of << " [label=\"";
        stringstream ss;
        ss << vt.second << ": "
            << breakStringLabel(vt.first.second->get_name(), op_name_max_length);
        of << ss.str();
        of << "\"," << dot_class_grounding_discard << "]" << endl;
    }
    forEach(EventRecordMap::value_type & vt, operatorGroundedOutRecords) {
        std::string invalidNode = createAnonymousNode(of, dot_node_shape_grounded_out);
        of << generateUngroundedOpNodeName(
                make_pair(vt.first.first, vt.first.second))
                << " -> " << invalidNode;
        of << " [label=\"";
        of << vt.second;
        of << "\"," << dot_class_grounding_grounded_out << "]" << endl;
    }
    forEach(EventRecordMap::value_type & vt, ungroundedOpDiscardRecords) {
        std::string invalidNode = createAnonymousNode(of, dot_node_shape_ungrounded_discard);

        // handle immediate discards that didn't have a push
        bool hasOpenPush = false;
        forEach(OpenRecordMap::value_type & oe, openRecords) {
            if(vt.first == oe.first) {
                hasOpenPush = true;
                break;
            }
        }
        if(hasOpenPush){
            of << generateUngroundedOpNodeName(make_pair(vt.first.first, vt.first.second));
        } else {
            of << generateNodeName(vt.first.first);
        }
        of << " -> " << invalidNode;
        of << " [label=\"";
        of << vt.second;
        if(!hasOpenPush) {
            of << ": " << breakStringLabel(vt.first.second->get_name(), op_name_max_length);
        }
        of << "\"," << dot_class_grounding_ungrounded_discard << "]" << endl;
    }


    forEach(OpenRecordMap::value_type & vt, openRecords) {
        std::string openNode;
        if(vt.first.second->isGrounded()) {   // open push that wasn't closed/discarded above
            openNode = createAnonymousNode(of, dot_node_shape_open);
        } else {        // open push to ungrounded op
            openNode = generateUngroundedOpNodeName(vt.first);
        }
        of << generateNodeName(vt.first.first) << " -> " << openNode;
        of << " [label=\"";
        of << generateOpenEdgeLabel(vt);
        of << "\"," << dot_class_open << "]" << endl;
    }
}

void Analysis::writeDotEqualStates(std::ofstream & of)
{
    if(!g_parameters.analyzeLinkEqualStates)
        return;

    // draw line, no constraint, between equal closed states (-timestamp)
    // should happen when better state is found in later event, otherwise we get the
    // direct discard back edges.
    // if discards are explicit, not as discard reason, then those should also have the
    // dashed lines

    // first collect all equal states (- timestamp)
    tr1::unordered_set<TimeStampedState, TssHash, TssEquals> equalStates;
    tr1::unordered_multimap<TimeStampedState, const TimeStampedState*, TssHash, TssEquals> equalMap;
    typedef 
        tr1::unordered_multimap<TimeStampedState, const TimeStampedState*, TssHash, TssEquals>::iterator
        EqualMapIt;

    forEach(const RecordedStatesMap::value_type & vt, recordedStates) {
        equalStates.insert(vt.first);
        equalMap.insert(make_pair(vt.first, vt.second));
    }
    forEach(const TimeStampedState & tss, replicatedStates) {
        equalStates.insert(tss);
        equalMap.insert(make_pair(tss, &tss));
    }

    forEach(const TimeStampedState & tss, equalStates) {
        pair<EqualMapIt, EqualMapIt> r = equalMap.equal_range(tss);
        // go over all pairs of equal states
        for(EqualMapIt it = r.first; it != r.second; it++) {
            EqualMapIt it2 = it;
            it2++;
            for(; it2 != r.second; it2++) {
                of << generateNodeName(it->second) << " -> " << generateNodeName(it2->second);
                of << " [" << dot_class_state_equal << "]" << endl;
            }
        }
    }
}

std::string Analysis::generateNodeName(const TimeStampedState* state)
{
    return formatString("state_%08X", state);
}

std::string Analysis::generateUngroundedOpNodeName(pair<const TimeStampedState*, const Operator*> sop)
{
    return formatString("state_%08X_op_%08X", sop.first, sop.second);
}

std::string Analysis::createAnonymousNode(std::ofstream & of, const std::string & shape)
{
    lastAnonymousNr++;
    std::string name = formatString("invalid_state_anon_%09d", lastAnonymousNr);

    of << name << "[shape=" << shape << ",label=\"\"]" << endl;
    return name;
}

std::string Analysis::generateNodeLabel(const TimeStampedState* state)
{
    stringstream ss;
    // symbolic/numeric optional
    std::string stateStr = state->toPDDL(true, true, g_parameters.analyzeOutputNumericalFluents, " ", 30);

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
        //ROS_INFO("findOrReplicateMatchingState: found recorded state.");
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

bool Analysis::isReplicated(const TimeStampedState* state)
{
    return replicatedStates.find(*state) != replicatedStates.end();
}

std::string Analysis::breakStringLabel(const std::string & s, unsigned int maxLength)
{
    // basically split it up first and puzzle it back together
    std::vector<string> parts = StringUtil::split(s, " ");
    std::string ret;
    std::string curLine;
    forEach(const std::string & p, parts) {
        if(curLine.empty()) {
            curLine = p;
            continue;
        }
        if(curLine.length() + 1 + p.length() > maxLength) {     // cur + " " + p
            if(ret.empty())
                ret = curLine;
            else
                ret += "\\n" + curLine;
            curLine = p;
        } else {
            curLine += " " + p;
        }
    }
    if(!curLine.empty()) {
        if(ret.empty())
            ret = curLine;
        else
            ret += "\\n" + curLine;
    }
    return ret;
}

