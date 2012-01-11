/*
 * StringStructure.h
 *
 *  Created on: Apr 28, 2011
 *      Author: Andreas Hertle
 */

#ifndef STRINGSTRUCTURE_H_
#define STRINGSTRUCTURE_H_

#include "ElementStructure.h"

namespace opl
{

class StringStructure: public opl::ElementStructure
{
public:
    StringStructure();
    virtual ~StringStructure();

    Element* identify(const ParseUnit* unit) const;
};

}

#endif /* STRINGSTRUCTURE_H_ */
