/*
 * Predicate.h
 *
 *  Created on: Aug 25, 2017
 *      Author: andreas
 */

#ifndef INCLUDE_CONTINUAL_PLANNING_EXECUTIVE_PREDICATE_H_
#define INCLUDE_CONTINUAL_PLANNING_EXECUTIVE_PREDICATE_H_

#include <string>
#include <vector>
using std::string;
using std::vector;

vector<string> splitOnWhitespace(string params);

/// Any predicate specification.
/** e.g. (on cup1 table0) (connected a b) (capacity robot1)  */
class Predicate
{
public:
	string name;
	vector<string> parameters;
	Predicate(const string& name, const string& params);
	Predicate(const string& name, const vector<string>& params);

	friend std::ostream & operator<<(std::ostream & os, const Predicate & p);
	bool operator<(const Predicate & p) const;
};
std::ostream & operator<<(std::ostream & os, const Predicate & p);


#endif /* INCLUDE_CONTINUAL_PLANNING_EXECUTIVE_PREDICATE_H_ */
