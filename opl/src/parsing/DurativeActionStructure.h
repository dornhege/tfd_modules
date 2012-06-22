/*
 * DurativeActionStructure.h
 *
 *  Created on: May 4, 2011
 *      Author: Andreas Hertle
 */

#ifndef DURATIVEACTIONSTRUCTURE_H_
#define DURATIVEACTIONSTRUCTURE_H_

#include "ElementStructure.h"

namespace opl
{

class DurativeActionStructure: public opl::ElementStructure
{
public:
    DurativeActionStructure();
    virtual ~DurativeActionStructure();

    Element* identify(const ParseUnit* unit) const;
};

}

#endif /* DURATIVEACTIONSTRUCTURE_H_ */
