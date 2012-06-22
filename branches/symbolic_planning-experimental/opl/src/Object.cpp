/*
 * Object.cpp
 *
 *  Created on: Feb 4, 2011
 *      Author: Andreas Hertle
 */

#include "Object.h"

namespace opl
{

Object::Object(const QString& type, const QString& name)
: Element(type, name)
{

}

Object::~Object()
{
}

bool Object::definesSymbol() const
{
    return true;
}

void Object::customEvaluation()
{
    // look up if symbol type exists
    const Element* symbolType = findType(type);
    if (symbolType == NULL)
    {
        std::cout << "Type::evaluate ERROR: Type " << qPrintable(type) << " of symbol " << qPrintable(name) << " not found" << std::endl << std::flush;
        assert(symbolType != NULL);
    }
}

QString Object::toPDDL() const
{
    QString object = "    ";
    object.append(name).append(" - ");
    object.append(type);
    object.append("\n");
    return object;
}

}
