/*
 * NumberStructure.h
 *
 *  Created on: May 9, 2011
 *      Author: Andreas Hertle
 */

#ifndef NUMBERSTRUCTURE_H_
#define NUMBERSTRUCTURE_H_

#include "ElementStructure.h"

namespace opl
{

class NumberStructure: public opl::ElementStructure
{
public:
    NumberStructure();
    virtual ~NumberStructure();

    Element* identify(const ParseUnit* unit) const;
};

}

#endif /* NUMBERSTRUCTURE_H_ */
