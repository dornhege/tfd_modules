#ifndef SYMBOLIC_STATE_H
#define SYMBOLIC_STATE_H

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH
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

        SymbolicState();
        virtual ~SymbolicState();

        typedef multimap<string,string>::iterator TypedObjectIterator;
        typedef multimap<string,string>::const_iterator TypedObjectConstIterator;

        void clear() {
            _typedObjects.clear();
            _booleanPredicates.clear();
            _numericalFluents.clear();
        }

        // Setter/Getter

        /// Add an object to the list of objects.
        void addObject(string obj, string type); 

        /// Get a map from type -> objects of that type.
        const multimap<string, string> & getTypedObjects() const { return _typedObjects; }


        /// Set a boolean predicate in the state. If it does not exist it will be created.
        void setBooleanPredicate(string name, vector<string> parameters, bool value);
        /// Set a boolean predicate using handcoded parameters.
        /**
         * The parameters string is a space-separated string of the form: "robot0 x1 test"
         * and will be parsed into the single parameters.
         */
        void setBooleanPredicate(string name, string parameters, bool value);

        /// Set a numerical fluent. If it does not exist it will be created.
        void setNumericalFluent(string name, vector<string> parameters, double value);
        /// Set a numerical fluent using handcoded parameters.
        void setNumericalFluent(string name, string parameters, double value);

        /// Determine if this state has the given predicate.
        /**
         * \param [out] value, if != NULL and the hasBooleanPredicate is true, the value will be filled.
         */
        bool hasBooleanPredicate(const Predicate & p, bool* value) const;
        bool hasNumericalFluent(const Predicate & p, double* value) const;

        // state matching

        /// Does state fulfill this state?
        /**
         * Check if state fulfills this state, i.e. for each predicate in this state, there is a 
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
        /// Determine if all fluents in this state are the same as in other and vice-versa.
        bool equals(const SymbolicState & other) const;

        // conversion/writing

        /// Give a PDDL notation as required for writing a PDDL problem.
        virtual void toPDDLProblem(std::ostream & ss) const;
        /// Output a goal condition of all predicates.
        virtual void toPDDLGoal(std::ostream & ss) const;

    protected:
        /// split string at " "
        vector<string> buildParameterList(string params); 

    protected:
        multimap<string, string> _typedObjects;    ///< Map from type to it objects of this same type.

        // The only possible parameters for the following predicates should be those in the _typedObjects list.
        map<Predicate, bool> _booleanPredicates;
        map<Predicate, double> _numericalFluents;

        typedef pair<Predicate, bool> BooleanPredicateEntry;
        typedef pair<Predicate, double> NumericalFluentEntry;
};

std::ostream & operator<<(std::ostream & os, const SymbolicState & ss);

#endif

