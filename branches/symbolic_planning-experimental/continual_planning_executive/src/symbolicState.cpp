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
    typedef pair< multimap<string,string>::iterator, multimap<string,string>::iterator > StringMapRange;
    multimap<string,string>::iterator mapIt;
   
    // insert obj for its type and all its supertypes
    StringMapRange ret = _superTypes.equal_range(type);
    for(mapIt = ret.first; mapIt != ret.second; mapIt++) {
        string st = mapIt->second;
        // should now insert (st, obj) into _typedObjects

        StringMapRange objRange = _typedObjects.equal_range(st);
        bool alreadyIn = false;
        for(multimap<string,string>::iterator it = objRange.first; it != objRange.second; it++) {
            if(it->second == obj) {
                alreadyIn = true;
                break;
            }
        }
        if(!alreadyIn) {
            _typedObjects.insert(make_pair(st, obj));
        }
    }

    // This code is only for backwards compatibility
    // If no types have been inserted into _superTypes, we
    // still add the pair (type, obj) into _typedObjects

    ret = _typedObjects.equal_range(type);
    // check if map contains typed obj of same name
    for(multimap<string,string>::iterator it = ret.first; it != ret.second; it++) {
        if(it->second == obj)
            return;
    }

    // insert it
    _typedObjects.insert(std::make_pair(type, obj));
}

void SymbolicState::addSuperType(string type, string supertype)
{
    // door_location - location
    multimap<string,string>::iterator mapIt;
    pair< multimap<string,string>::iterator, multimap<string,string>::iterator > ret;

    deque<pair<string, string> > queue; // all new connections
    queue.push_back(make_pair(type, supertype));
    queue.push_back(make_pair(type, type));
    queue.push_back(make_pair(supertype, supertype));

    // add supertype pairs in graph and readd their
    // dependencies to the queue to compute the transitive closure
    // as a fixed point computation (no more new pairs)
    while(!queue.empty()) {
        pair<string, string> stp = queue.front();
        queue.pop_front();

        // check if stp is already in
        ret = _superTypes.equal_range(stp.first);
        bool alreadyIn = false;
        for(mapIt = ret.first; mapIt != ret.second; mapIt++) {
            string st = mapIt->second;
            if(stp.second == st) {
                alreadyIn = true;
                break;
            }
        }
        if(alreadyIn)
            continue;

        // this is a new pair, insert it
        _superTypes.insert(stp);

        // now for all pairs (t, st) in the map
        // 1. if stp.second == t, make sure the pair (stp.first, st) is in
        // 2. if stp.first == st, make sure the pair (t, stp.second) is in
        for(mapIt = _superTypes.begin(); mapIt != _superTypes.end(); mapIt++) {
            // mapIt = (t, st)
            if(stp.second == mapIt->first)      // 1.
                queue.push_back(make_pair(stp.first, mapIt->second));
            if(stp.first == mapIt->second)      // 2.
                queue.push_back(make_pair(mapIt->first, stp.second));
        }
    }
}

bool SymbolicState::isMostSpecificType(string obj, string type) const
{
    // check if there is a pair (t, st) where type == st (!= t), then t is more specific
    // (given that (t, obj) is in _typedObjects)
    multimap<string,string>::const_iterator mapIt;
    for(mapIt = _superTypes.begin(); mapIt != _superTypes.end(); mapIt++) {
        // mapIt = (t, st)
        if(type == mapIt->second && mapIt->first != mapIt->first) { // t is a true subtype of type
            string t = mapIt->first;    // check if there is an entry (t, obj) in _typedObjects

            pair< multimap<string,string>::const_iterator, multimap<string,string>::const_iterator > ret;
            ret = _typedObjects.equal_range(t);

            multimap<string,string>::const_iterator objectIt;
            for(objectIt = ret.first; objectIt != ret.second; objectIt++) {
                if(objectIt->second == obj) {   // (t, obj) is a valid pair, this type is not most specific
                    return false;
                }
            }
        }
    }

    // if we found no more specific one or 
    // if there is no _superTypes entry, return true
    return true;
}

void SymbolicState::printSuperTypes()
{
    // desired output:
    // door_location is a (object, door_location, location)
    // manipulation_location is a (object, manipulation_location, location)
    // location is a (object, location)
    string currentType;
    multimap<string,string>::iterator mapIt;
    for(mapIt = _superTypes.begin(); mapIt != _superTypes.end(); mapIt++) {
        // mapIt = (t, st)
        if(mapIt->first != currentType) {   // different type
            if(!currentType.empty()) {  // this is not the first, close the last one
                printf(")\n");
            }
            currentType = mapIt->first;
            printf("%s is a (object", currentType.c_str());
        }
        printf(", %s", mapIt->second.c_str());
    }
}

void SymbolicState::removeObject(string obj, bool removePredicates)
{
    for(multimap<string,string>::iterator it = _typedObjects.begin(); it != _typedObjects.end(); ) {
        multimap<string, string>::iterator current = it;    // remember current for deletion
        it++;       // but forward it now, before deleting that
        if(current->second == obj) {
            _typedObjects.erase(current);    // it invalid now
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

void SymbolicState::setForEachGoalStatement(string objectType, string predicateName, bool value)
{
    _forEachGoalStatements.insert(pair<string, pair<string, bool> >(objectType, pair<string, bool>(predicateName, value)));
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

    // evaluate foreach goal statements
    forEach(const ForEachGoalStatements::value_type & fobp, _forEachGoalStatements) {
        string objectType = fobp.first;
        // check for all objects of the specified type
        pair< multimap<string,string>::const_iterator, multimap<string,string>::const_iterator > ret;
        ret = _typedObjects.equal_range(objectType);
        for(multimap<string,string>::const_iterator objectIt = ret.first; objectIt != ret.second; objectIt++)
        {
            Predicate bp;
            bp.name = fobp.second.first;
            bp.parameters = buildParameterList(objectIt->second);
            bool value;
            // state needs to have every goal predicate
            if(!state.hasBooleanPredicate(bp, &value))
                return false;
            // and they need to be the same truth value
            if(value != fobp.second.second)
                return false;
        }
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
        if(!isMostSpecificType(it->second, it->first))  // only insert obj entries for its most specific type
            continue;

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
    if(_booleanPredicates.empty() && _numericalFluents.empty() && _forEachGoalStatements.empty()) {
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

    // from: multimap<objectType, pair<predicateName, value> >
    // produce: (forall (?o - objectType) (predicateName ?o))
    // or:      (forall (?o - objectType) (not (predicateName ?o)))
    forEach(const ForEachGoalStatements::value_type & p, _forEachGoalStatements) {
        os << "    (forall (?o - " << p.first << ") (";
        if(p.second.second)
            os << p.second.first << " ?o))" << std::endl;
        else
            os << "not (" << p.second.first << " ?o)))" << std::endl;
    }
    os << "  ))" << std::endl;
}


vector<string> SymbolicState::buildParameterList(string params) const
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
        if(!ss.isMostSpecificType(it->second, it->first))  // only insert obj entries for its most specific type
            continue;

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

