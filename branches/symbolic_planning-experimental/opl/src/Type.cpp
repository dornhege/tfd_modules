/*
 * Type.cpp
 *
 *  Created on: Nov 23, 2010
 *      Author: Andreas Hertle
 */

#include <assert.h>
#include "Type.h"

namespace opl
{

Type::Type(const QString& type, const QString& name, const QString& superName)
: Element(type, name), superName(superName)
{
    evaluated = false;
    superType = NULL;
}

Type::~Type()
{
}

const Element* Type::findThis() const
{
    return this;
}

void Type::customEvaluation()
{
    evaluated = true;
    if (superName != "Object")
    {
        // look up if superType exists
        const Element* type = findType(superName);
        if (type == NULL)
        {
            std::cout << "Type::evaluate ERROR: Super type " << qPrintable(superName) << " of type " << qPrintable(name) << " not found" << std::endl << std::flush;
            assert(type != NULL);
        }
        superType = reinterpret_cast<const Type*>(type);
    }
}

const Element* Type::findSymbolDefinition(const QString& symbol) const
{
    const Element* element = Element::findSymbolDefinition(symbol);
    if (element != NULL)
    {
        return element;
    }
    //std::cout << "Type::findSymbolDefinition INFO: looking in Super type " << qPrintable(superName) << " for symbol " << qPrintable(symbol) << std::endl << std::flush;
    if (element == NULL && superType != NULL)
    {
        return superType->findSymbolDefinition(symbol);
    }
    return element;
}

QString Type::toPDDL() const
{
    QString predicate = "    ";
    predicate.append(name).append(" - ").append(superName);
    predicate.append("\n");
    return predicate;
}

}
