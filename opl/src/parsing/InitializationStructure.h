/*
 * InitializationStructure.h
 *
 *  Created on: May 10, 2011
 *      Author: Andreas Hertle
 */

#ifndef INITIALIZATIONSTRUCTURE_H_
#define INITIALIZATIONSTRUCTURE_H_

#include "ElementStructure.h"

namespace opl
{

class InitializationStructure: public opl::ElementStructure
{
public:
    InitializationStructure();
    virtual ~InitializationStructure();

    Element* identify(const ParseUnit* unit) const;
};

}

#endif /* INITIALIZATIONSTRUCTURE_H_ */
