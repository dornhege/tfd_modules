#include "continual_planning_executive/symbolicState.h"
#include <sstream>
#include <ros/ros.h>

bool Predicate::operator<(const Predicate & p) const
{
    if(name < p.name)
        return true;
    else if(name > p.name)
        return false;
    if(parameters.size() < p.parameters.size())
        return true;
    else if(parameters.size() > p.parameters.size())
        return false;
    for(unsigned int i = 0; i < parameters.size(); i++) {
        if(parameters[i] < p.parameters[i])
            return true;
        else if(parameters[i] > p.parameters[i])
            return false;
    }
    // equal -> false (not <)
    return false;
}

std::ostream & operator<<(std::ostream & os, const Predicate & p)
{
    os << "(" << p.name;
    forEach(const string & s, p.parameters) {
        os << " " << s;
    }
    os << ")";
    return os;
}


SymbolicState::SymbolicState()
{
}

SymbolicState::~SymbolicState()
{
}

void SymbolicState::addObject(string obj, string type)
{
    pair< multimap<string,string>::iterator, multimap<string,string>::iterator > ret;
    ret = _typedObjects.equal_range(type);
    // check if map contains typed obj of same name
    for(multimap<string,string>::iterator it = ret.first; it != ret.second; it++) {
        if(it->second == obj)
            return;
    }
    // inserting the same obj for different types shouldnt happen.
    for(multimap<string,string>::iterator it = _typedObjects.begin(); it != _typedObjects.end(); it++) {
        ROS_ASSERT(it->second != obj);   // if its the same type, the previous loop should have gone out, otherwise error
    }

    // insert it
    _typedObjects.insert(std::make_pair(type, obj));
}

void SymbolicState::setBooleanPredicate(string name, vector<string> parameters, bool value)
{
    Predicate bp;
    bp.name = name;
    bp.parameters = parameters;

    _booleanPredicates[bp] = value;
}

void SymbolicState::setBooleanPredicate(string name, string parameters, bool value)
{
    vector<string> params = buildParameterList(parameters);
    setBooleanPredicate(name, params, value);
}

void SymbolicState::setNumericalFluent(string name, vector<string> parameters, double value)
{
    Predicate bp;
    bp.name = name;
    bp.parameters = parameters;

    _numericalFluents[bp] = value;
}

void SymbolicState::setNumericalFluent(string name, string parameters, double value)
{
    vector<string> params = buildParameterList(parameters);
    setNumericalFluent(name, params, value);
}

bool SymbolicState::hasBooleanPredicate(const Predicate & p, bool* value) const
{
    map<Predicate, bool>::const_iterator it = _booleanPredicates.find(p);
    if(it == _booleanPredicates.end())
        return false;
    if(value != NULL)
        *value = it->second;
    return true;
}

bool SymbolicState::hasNumericalFluent(const Predicate & p, double* value) const
{
    map<Predicate, double>::const_iterator it = _numericalFluents.find(p);
    if(it == _numericalFluents.end())
        return false;
    if(value != NULL)
        *value = it->second;
    return true;
}


bool SymbolicState::isFulfilledBy(const SymbolicState & state) const
{
    forEach(const BooleanPredicateEntry & bp, _booleanPredicates) {
        bool value;
        // state needs to have every goal predicate
        if(!state.hasBooleanPredicate(bp.first, &value))
            return false;
        // and they need to be the same truth value
        if(value != bp.second)
            return false;
    }

    forEach(const NumericalFluentEntry & nf, _numericalFluents) {
        double value;
        // state needs to have every goal predicate
        if(!state.hasNumericalFluent(nf.first, &value))
            return false;
        // and they need to be the same value
        if(!double_equals(value, nf.second))
            return false;
    }

    return true;
}

bool SymbolicState::booleanEquals(const SymbolicState & other) const
{
    // TODO: better: collect merged list (should be same) and then only check truth
    // all our predicates are the same in other
    forEach(const BooleanPredicateEntry & bp, _booleanPredicates) {
        bool value;
        // state needs to have every goal predicate
        if(!other.hasBooleanPredicate(bp.first, &value))
            return false;
        // and they need to be the same truth value
        if(value != bp.second)
            return false;
    }
    // all other's predicates are the same in ours
    forEach(const BooleanPredicateEntry & bp, other._booleanPredicates) {
        bool value;
        // state needs to have every goal predicate
        if(!hasBooleanPredicate(bp.first, &value))
            return false;
        // and they need to be the same truth value
        if(value != bp.second)
            return false;
    }
    return true;
}

bool SymbolicState::numericalEquals(const SymbolicState & other) const
{
    // TODO
    return false;
}

bool SymbolicState::equals(const SymbolicState & other) const
{
    return booleanEquals(other) && numericalEquals(other);
}


void SymbolicState::toPDDLProblem(std::ostream & os) const
{
    os << "  (:objects ";
    string lastType = "";
    for(multimap<string,string>::const_iterator it = _typedObjects.begin(); it != _typedObjects.end(); it++) {
        if(lastType != "" && it->first != lastType) {  // type changed
            os << "- " << lastType << " ";
        }
        lastType = it->first;
        os << it->second << " ";
    }
    if(lastType != "")
        os << "- " << lastType;
    os << ")" << std::endl;
    os << "  (:init" << std::endl;
    forEach(const BooleanPredicateEntry & p, _booleanPredicates) {
        if(p.second)
            os << "    " << p.first << std::endl;
    }
    forEach(const NumericalFluentEntry & nf, _numericalFluents) {
        os << "    (= " << nf.first << " " << nf.second << ")" << std::endl;
    }
    os << "  )" << std::endl;
}

void SymbolicState::toPDDLGoal(std::ostream & os) const
{
    // prevent empty conjunction in goal
    if(_booleanPredicates.empty() && _numericalFluents.empty()) {
        os << "  (:goal " << std::endl;
        os << "  )" << std::endl;
        return;
    }

    os << "  (:goal (and" << std::endl;
    forEach(const BooleanPredicateEntry & p, _booleanPredicates) {
        if(p.second)
            os << "    " << p.first << std::endl;
        else
            os << "    (not " << p.first << ")" << std::endl;
    }
    forEach(const NumericalFluentEntry & nf, _numericalFluents) {
        os << "    (= " << nf.first << " " << nf.second << ")" << std::endl;
    }
    os << "  ))" << std::endl;
}


vector<string> SymbolicState::buildParameterList(string params)
{
    vector<string> ret;

    while(params.size() > 0) {
        // strip leading whitespace
        while(params.size() > 0 && params[0] == ' ') {
            params = params.substr(1);
        }
        // something left
        if(params.size() > 0) {
            size_t ind = params.find_first_of(" ");
            ret.push_back(params.substr(0, ind));     // insert next word
            if(ind == string::npos) {  // nothing left
                params = "";
            } else {
                params = params.substr(ind);              // and remove that from string
            }
        }
    }

    return ret;
}

std::ostream & operator<<(std::ostream & os, const SymbolicState & ss) {
    os << "Objects:" << std::endl;
    string lastType = "";
    for(multimap<string,string>::const_iterator it = ss._typedObjects.begin(); it != ss._typedObjects.end(); it++) {
        if(lastType != "" && it->first != lastType) {  // type changed
            os << "- " << lastType << "   ";
        }
        lastType = it->first;
        os << it->second << " ";
    }
    if(lastType != "")
        os << "- " << lastType;

    os << std::endl;
    os << "True Predicates:" << std::endl;
    forEach(const SymbolicState::BooleanPredicateEntry & bp, ss._booleanPredicates) {
        if(bp.second)
            os << bp.first << " ";
    }
    os << std::endl;
    os << "False Predicates:" << std::endl;
    forEach(const SymbolicState::BooleanPredicateEntry & bp, ss._booleanPredicates) {
        if(!bp.second)
            os << bp.first << " ";
    }
    os << std::endl;
    os << "Numerical Fluents:" << std::endl;
    int count = 0;
    forEach(const SymbolicState::NumericalFluentEntry & nf, ss._numericalFluents) {
        os << nf.first << " = " << nf.second << " ";
        // print newline every 5 outputs
        count++;
        if(count >= 5) {
            count = 0;
            os << std::endl;
        }
    }
    os << std::endl;
    return os;
}

