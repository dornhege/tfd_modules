/*
 * ObjectStructure.h
 *
 *  Created on: May 10, 2011
 *      Author: Andreas Hertle
 */

#ifndef OBJECTSTRUCTURE_H_
#define OBJECTSTRUCTURE_H_

#include "ElementStructure.h"

namespace opl
{

class ObjectStructure: public opl::ElementStructure
{
public:
    ObjectStructure();
    virtual ~ObjectStructure();

    Element* identify(const ParseUnit* unit) const;
};

}

#endif /* OBJECTSTRUCTURE_H_ */
