/*
 * SymbolDefinition.cpp
 *
 *  Created on: Apr 28, 2011
 *      Author: Andreas Hertle
 */

#include "SymbolDefinition.h"

namespace opl
{

SymbolDefinition::SymbolDefinition(const QString& type, const QString& name)
: Element(type, name)
{

}

SymbolDefinition::~SymbolDefinition()
{
}

void SymbolDefinition::customEvaluation()
{
    // look up if symbol type exists
    const Element* symbolType = findType(type);
    if (symbolType == NULL)
    {
        std::cout << "Type::evaluate ERROR: Type " << qPrintable(type) << " of symbol " << qPrintable(name) << " not found" << std::endl << std::flush;
        assert(symbolType != NULL);
    }
}

QString SymbolDefinition::toPDDL() const
{
    QString symbol(" ?");
    symbol.append(name).append(" - ").append(type);
    return symbol;
}

bool SymbolDefinition::definesSymbol() const
{
    return true;
}

}
