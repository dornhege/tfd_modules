#ifndef SYMBOLIC_STATE_H
#define SYMBOLIC_STATE_H

#include <boost/foreach.hpp>
#ifdef __CDT_PARSER__
#define forEach(a, b) for(a : b)
#else
#define forEach BOOST_FOREACH
#endif

#include <math.h>
#include <stdarg.h>
#include <string>
#include <deque>
#include <vector>
#include <map>
#include <utility>
using std::string;
using std::deque;
using std::vector;
using std::map;
using std::multimap;
using std::pair;
using std::make_pair;
#include <ostream>

inline bool double_equals(double d1, double d2)
{
    return fabs(d1 - d2) < 0.0001;
}

/// Any predicate specification.
/** e.g. (on cup1 table0) (connected a b) (capacity robot1)  */
class Predicate
{
    public:
        string name;
        vector<string> parameters;

        bool operator<(const Predicate & p) const;
};
std::ostream & operator<<(std::ostream & os, const Predicate & p);

/// Generic class for a symbolic state.
/**
 * A symbolic state represents grounded boolean predicates, numeric and object fluents.
 *
 * The contents should be filled by a robot specific implementation.
 */
class SymbolicState
{
    public:
        friend std::ostream & operator<<(std::ostream & os, const SymbolicState & ss);
        class OStreamMode {
            public:
                static bool forceNewlines;
        };

        SymbolicState();
        virtual ~SymbolicState();

        typedef multimap<string,string>::iterator TypedObjectIterator;
        typedef multimap<string,string>::const_iterator TypedObjectConstIterator;

        void clear() {
            _typedObjects.clear();
            _booleanPredicates.clear();
            _numericalFluents.clear();
            _objectFluents.clear();
        }

        // Setter/Getter

        /// Add an object to the list of objects.
        /**
         * The object will be entered for type and all its supertypes.
         * Thus all super types have to be defined by addSuperType before.
         */
        void addObject(string obj, string type);

        /// e.g. door_location - location
        void addSuperType(string type, string supertype);

        /// Output the super types map.
        void printSuperTypes() const;

        /// Remove an object from the state.
        /**
         * \param [in] removePredicates if true, all predicates/fluents that contain obj will also be removed.
         */
        void removeObject(string obj, bool removePredicates = true);

        /// Get a map from type -> objects of that type.
        const multimap<string, string> & getTypedObjects() const { return _typedObjects; }

        /// Get the map of Boolean predicates
        const map<Predicate, bool> & getBooleanPredicates() const { return _booleanPredicates; }

        /// Get the map of numerical fluents
        const map<Predicate, double> & getNumericalFluents() const { return _numericalFluents; }

        /// Get the map of object fluents
        const map<Predicate, string> & getObjectFluents() const { return _objectFluents; }




        /// Set a boolean predicate in the state. If it does not exist it will be created.
        void setBooleanPredicate(string name, vector<string> parameters, bool value);
        /// Set a boolean predicate using handcoded parameters.
        /**
         * The parameters string is a space-separated string of the form: "robot0 x1 test"
         * and will be parsed into the single parameters.
         */
        void setBooleanPredicate(string name, string parameters, bool value);
        /// Set all boolean predicates with name to value.
        void setAllBooleanPredicates(string name, bool value);

        /// Set a numerical fluent. If it does not exist it will be created.
        void setNumericalFluent(string name, vector<string> parameters, double value);
        /// Set a numerical fluent using handcoded parameters.
        void setNumericalFluent(string name, string parameters, double value);
        /// Set all numerical fluents with name to value.
        void setAllNumericalFluents(string name, double value);

        /// Set a object fluent. If it does not exist it will be created.
        void setObjectFluent(string name, vector<string> parameters, string value);
        /// Set a object fluent using handcoded parameters.
        void setObjectFluent(string name, string parameters, string value);
        /// Set all object fluents with name to value.
        void setAllObjectFluents(string name, string value);

        /// Create a forall statement for a object type with the specified predicate and value.
        /// Useful for goal conditions.
        void setForEachGoalStatement(string objectType, string predicateName, bool value);
        /// set the goal condition directly.
        /// mutually exclusive with forall goals.
        void setStringGoalStatement(string goalStatement);

        /// Determine if this state has the given predicate.
        /**
         * \param [out] value, if != NULL and the hasBooleanPredicate is true, the value will be filled.
         */
        bool hasBooleanPredicate(const Predicate & p, bool* value) const;
        bool hasNumericalFluent(const Predicate & p, double* value) const;
        bool hasObjectFluent(const Predicate & p, string* value) const;

        // state matching

        /// Does state fulfill this state?
        /**
         * Check if the other state fulfills this (partial) state, i.e. for each predicate in this state, there is a
         * corresponding one in the other state and it has the same truth value.
         * Predicates in the other state that do not exist in this state are ignored.
         * Numeric fluents need to be the same up to double_equals.
         *
         * Commonly this is a goal state and the other state is the current state.
         *
         * \returns true, if for each predicate in this state, there is a consistent one in other.
         */
        bool isFulfilledBy(const SymbolicState & other) const;

        /// Determine if the boolean predicates in this state are the same as in other and vice-versa.
        bool booleanEquals(const SymbolicState & other) const;
        /// Determine if the numerical fluents in this state are the same as in other and vice-versa.
        bool numericalEquals(const SymbolicState & other) const;
        /// Determine if the object fluents in this state are the same as in other and vice-versa.
        bool objectFluentsEquals(const SymbolicState & other) const;
        /// Determine if all fluents in this state are the same as in other and vice-versa.
        bool equals(const SymbolicState & other) const;

        // conversion/writing

        /// Give a PDDL notation as required for writing a PDDL problem.
        virtual void toPDDLProblem(std::ostream & ss) const;
        /// Output a goal condition of all predicates.
        virtual void toPDDLGoal(std::ostream & ss) const;

    protected:
        /// split string at " "
        vector<string> buildParameterList(string params) const;

        /// \returns true, if type is the most specific type for obj
        /**
         * door_location - location
         *
         * doorl1 - door_location -> true
         * doorl1 - location -> false
         */
        bool isMostSpecificType(string obj, string type) const;

    protected:
        /// Map from type to objects of this same type
        /**
         * should also contain entries for all supertypes, e.g.:
         * l1_door1 has an entry for door_location and for location (given addSuperType was inserted) 
         * */
        multimap<string, string> _typedObjects;

        multimap<string, string> _superTypes;      ///< Type -> Supertypes (including type)

        // The only possible parameters for the following predicates should be those in the _typedObjects list.
        // matched strings in _objectFluents should also only be from _typedObjects and of the correct type
        map<Predicate, bool> _booleanPredicates;
        map<Predicate, double> _numericalFluents;
        map<Predicate, string> _objectFluents;
        // foreach statements
        typedef multimap<string, pair<string, bool> > ForEachGoalStatements;
        ForEachGoalStatements _forEachGoalStatements;
        string _directGoalStatement;

        typedef map<Predicate, bool>::value_type BooleanPredicateEntry;
        typedef map<Predicate, double>::value_type NumericalFluentEntry;
        typedef map<Predicate, string>::value_type ObjectFluentEntry;
};

std::ostream & operator<<(std::ostream & os, const SymbolicState & ss);

#endif

