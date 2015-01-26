/*
 * ProblemStructure.h
 *
 *  Created on: May 10, 2011
 *      Author: Andreas Hertle
 */

#ifndef PROBLEMSTRUCTURE_H_
#define PROBLEMSTRUCTURE_H_

#include "ElementStructure.h"

namespace opl
{

class ProblemStructure: public opl::ElementStructure
{
public:
    ProblemStructure();
    virtual ~ProblemStructure();

    Element* identify(const ParseUnit* unit) const;
};

}

#endif /* PROBLEMSTRUCTURE_H_ */
