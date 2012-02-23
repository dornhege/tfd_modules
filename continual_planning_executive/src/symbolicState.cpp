#include "continual_planning_executive/symbolicState.h"
#include <sstream>
#include <ros/ros.h>

bool Predicate::operator<(const Predicate & p) const
{
#if 0
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
#endif
    // sorting procedure for predicates
    // no particular reason then to put them in the map somehow
    // so: tailored to nice outputs
    //
    // First: "global" preds = no parameters
    // Then: sorted by their first parameter
    // -> Idea: If that is an object all preds of the same object in one bunch
    // Next: sorted by name
    // -> Same predicates for the object together
    // finally sorted by the following parameters

    // 0-size params go before all others
    if(parameters.empty() && !p.parameters.empty())
        return true;
    if(!parameters.empty() && p.parameters.empty())
        return false;
    // if both 0-size, sort by name
    if(parameters.empty() && p.parameters.empty())
        return name < p.name;

    // now both parameters not empty
    // sort by first parameter
    if(parameters[0] < p.parameters[0])
        return true;
    if(parameters[0] > p.parameters[0])
        return false;
    // first is same, sort by name
    if(name < p.name)
        return true;
    else if(name > p.name)
        return false;
    // names are same, just sort by the following params
    if(parameters.size() < p.parameters.size())
        return true;
    else if(parameters.size() > p.parameters.size())
        return false;
    for(unsigned int i = 1; i < parameters.size(); i++) {
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

void SymbolicState::removeObject(string obj, bool removePredicates)
{
    for(multimap<string,string>::iterator it = _typedObjects.begin(); it != _typedObjects.end(); it++) {
        if(it->second == obj) {
            _typedObjects.erase(it);    // it invalid now
            break;  // once should be OK, only 1 object per name
        }
    }

    if(!removePredicates)
        return;

    bool removedSomething = true;   // true, to get in loop
    // unfortunately the erase function returning an iterator is only C++11
    // so for now we do the ugly (loop until match, erase once), until nothing changed - approach
    while(removedSomething) {
        removedSomething = false;
        for(map<Predicate, bool>::iterator it = _booleanPredicates.begin(); it != _booleanPredicates.end(); it++) {
            bool foundObj = false;
            for(vector<string>::const_iterator paramIt = it->first.parameters.begin();
                    paramIt != it->first.parameters.end(); paramIt++) {
                if(obj == *paramIt) {
                    foundObj = true;
                    break;
                }
            }
            if(foundObj) {
                _booleanPredicates.erase(it);    // it invalid
                removedSomething = true;
                break;
            }
        }
    }
    // the same for numerical fluents
    removedSomething = true;
    while(removedSomething) {
        removedSomething = false;
        for(map<Predicate, double>::iterator it = _numericalFluents.begin(); it != _numericalFluents.end(); it++) {
            bool foundObj = false;
            for(vector<string>::const_iterator paramIt = it->first.parameters.begin();
                    paramIt != it->first.parameters.end(); paramIt++) {
                if(obj == *paramIt) {
                    foundObj = true;
                    break;
                }
            }
            if(foundObj) {
                _numericalFluents.erase(it);    // it invalid
                removedSomething = true;
                break;
            }
        }
    }
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

void SymbolicState::setAllBooleanPredicates(string name, bool value)
{
    forEach(SymbolicState::BooleanPredicateEntry & bp, _booleanPredicates) {
        if(bp.first.name == name)
            bp.second = value;
    }
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

void SymbolicState::setAllNumericalFluents(string name, double value)
{
    forEach(SymbolicState::NumericalFluentEntry & nf, _numericalFluents) {
        if(nf.first.name == name)
            nf.second = value;
    }
}

void SymbolicState::setObjectFluent(string name, vector<string> parameters, string value)
{
    Predicate bp;
    bp.name = name;
    bp.parameters = parameters;

    _objectFluents[bp] = value;
}

void SymbolicState::setObjectFluent(string name, string parameters, string value)
{
    vector<string> params = buildParameterList(parameters);
    setObjectFluent(name, params, value);
}

void SymbolicState::setAllObjectFluents(string name, string value)
{
    forEach(SymbolicState::ObjectFluentEntry & of, _objectFluents) {
        if(of.first.name == name)
            of.second = value;
    }
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
        
bool SymbolicState::hasObjectFluent(const Predicate & p, string* value) const
{
    map<Predicate, string>::const_iterator it = _objectFluents.find(p);
    if(it == _objectFluents.end())
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

    forEach(const ObjectFluentEntry & of, _objectFluents) {
        string value;
        // state needs to have every goal predicate
        if(!state.hasObjectFluent(of.first, &value))
            return false;
        // and they need to be the same value
        if(value != of.second)
            return false;
    }

    return true;
}

bool SymbolicState::booleanEquals(const SymbolicState & other) const
{
    // better: collect merged list (should be same) and then only check truth
    // all our predicates are the same in other
    forEach(const BooleanPredicateEntry & bp, _booleanPredicates) {
        bool value;
        // state needs to have every predicate
        if(!other.hasBooleanPredicate(bp.first, &value))
            return false;
        // and they need to be the same truth value
        if(value != bp.second)
            return false;
    }
    // all other's predicates are the same in ours
    forEach(const BooleanPredicateEntry & bp, other._booleanPredicates) {
        bool value;
        // state needs to have every predicate
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
    forEach(const NumericalFluentEntry & nf, _numericalFluents) {
        double value;
        // state needs to have every predicate
        if(!other.hasNumericalFluent(nf.first, &value))
            return false;
        // and they need to be the same truth value
        if(!double_equals(value, nf.second))
            return false;
    }
    // all other's predicates are the same in ours
    forEach(const NumericalFluentEntry & nf, other._numericalFluents) {
        double value;
        // state needs to have every predicate
        if(!hasNumericalFluent(nf.first, &value))
            return false;
        // and they need to be the same truth value
        if(!double_equals(value, nf.second))
            return false;
    }
    return true;
}

bool SymbolicState::objectFluentsEquals(const SymbolicState & other) const
{
    forEach(const ObjectFluentEntry & of, _objectFluents) {
        string value;
        // state needs to have every predicate
        if(!other.hasObjectFluent(of.first, &value))
            return false;
        // and they need to be the same truth value
        if(value != of.second)
            return false;
    }
    // all other's predicates are the same in ours
    forEach(const ObjectFluentEntry & of, other._objectFluents) {
        string value;
        // state needs to have every predicate
        if(!hasObjectFluent(of.first, &value))
            return false;
        // and they need to be the same truth value
        if(value != of.second)
            return false;
    }
    return true;
}

bool SymbolicState::equals(const SymbolicState & other) const
{
    return booleanEquals(other) && numericalEquals(other) && objectFluentsEquals(other);
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
    forEach(const ObjectFluentEntry & of, _objectFluents) {
        os << "    (= " << of.first << " " << of.second << ")" << std::endl;
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
    forEach(const ObjectFluentEntry & of, _objectFluents) {
        os << "    (= " << of.first << " " << of.second << ")" << std::endl;
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
    os << "Object Fluents:" << std::endl;
    count = 0;
    forEach(const SymbolicState::ObjectFluentEntry & nf, ss._objectFluents) {
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

