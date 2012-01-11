/*
 * OPLSemantic.h
 *
 *  Created on: Aug 5, 2011
 *      Author: Andreas Hertle
 */

#ifndef OPLSEMANTIC_H_
#define OPLSEMANTIC_H_

#include "Type.h"
#include "Domain.h"
#include "Element.h"
#include "parsing/Parser.h"
#include "parsing/ParseUnit.h"
#include "parsing/ElementDetector.h"
#include "parsing/DomainStructure.h"
#include "parsing/TypeStructure.h"
#include "parsing/FluentStructure.h"
#include "parsing/ModuleStructure.h"
#include "parsing/StringStructure.h"
#include "parsing/NumberStructure.h"
#include "parsing/SymbolDefinitionStructure.h"
#include "parsing/NamedListStructure.h"
#include "parsing/SymbolAccessStructure.h"
#include "parsing/DurativeActionStructure.h"
#include "parsing/FunctionStructure.h"
#include "parsing/ProblemStructure.h"
#include "parsing/ObjectStructure.h"
#include "parsing/InitializationStructure.h"

namespace opl
{

class OPLSemantic
{
private:
    ElementDetector domainDetector;
    ElementDetector problemDetector;

public:
    OPLSemantic();
    virtual ~OPLSemantic();

    const ElementDetector& getDomainDetector() {return domainDetector;}
    const ElementDetector& getProblemDetector() {return problemDetector;}
};

}

#endif /* OPLSEMANTIC_H_ */
