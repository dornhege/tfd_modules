/*
 * Predicate.cpp
 *
 *  Created on: Aug 25, 2017
 *      Author: andreas
 */

#include <continual_planning_executive/predicate.h>

Predicate::Predicate(const string& name, const string& params):name(name)
{
	parameters = splitOnWhitespace(params);
}

Predicate::Predicate(const string& name, const vector<string>& params):name(name), parameters(params)
{
}

vector<string> splitOnWhitespace(string params)
{
	vector<string> word_list;

	size_t word_start = 0;
	size_t word_end = 0;
	const string whitespace = " \t";
	do
	{
		word_start = params.find_first_not_of(whitespace, word_end);
		word_end = params.find_first_of(whitespace, word_start);
		if (word_end == string::npos)
		{
			word_end = params.size();
		}
		word_list.push_back(params.substr(word_start, word_end - word_start));
	} while (word_start < params.size() && word_end < params.size());

	return word_list;
}


