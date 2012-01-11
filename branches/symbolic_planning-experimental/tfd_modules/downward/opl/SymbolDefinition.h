/*
 * SymbolDefinition.h
 *
 *  Created on: Apr 28, 2011
 *      Author: Andreas Hertle
 */

#ifndef SYMBOLDEFINITION_H_
#define SYMBOLDEFINITION_H_

#include "Element.h"

namespace opl
{

class SymbolDefinition: public opl::Element
{
public:
    SymbolDefinition(const QString& type, const QString& name);
    virtual ~SymbolDefinition();

    void customEvaluation();
    QString toPDDL() const;
    bool definesSymbol() const;
};

}

#endif /* SYMBOLDEFINITION_H_ */
