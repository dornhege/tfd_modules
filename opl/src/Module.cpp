/*
 * Module.cpp
 *
 *  Created on: Mar 16, 2011
 *      Author: Andreas Hertle
 */

#include <iostream>
#include <assert.h>
#include "Module.h"

namespace opl
{

Module::Module(const QString& type, const QString& name)
: Element(type, name)
{
    libraryCall = NULL;
    effects = NULL;
}

Module::~Module()
{
}

void Module::customEvaluation()
{
    QListIterator<Element*> bodyIterator(body);
    while (bodyIterator.hasNext())
    {
        Element* element = bodyIterator.next();
        if (element->getType() == "EffectList" && element->getName() == "Effects")
        {
            effects = element;
        }
        else if (element->getType() == "StringConstant")
        {
            libraryCall = element;
        }
        else
        {
            std::cout << "Module::customEvaluation ERROR: unknown element in module body: " << qPrintable(element->getType()) << " " << qPrintable(element->getName()) << std::endl << std::flush;
            assert(NULL);
        }
    }
    if (libraryCall == NULL)
    {
        std::cout << "Module::customEvaluation ERROR: library call was not defined in module: " << qPrintable(type) << " " << qPrintable(name) << std::endl << std::flush;
        assert(libraryCall != NULL);
    }
}

bool Module::definesSymbol() const
{
    return true;
}

QString Module::toPDDL() const
{
    QString module = "    (";
    const Element* thisElement = findThis();
    if (thisElement != NULL)
    {
        module.append(thisElement->getName()).append("_");
        module.append(name);
        module.append(" ?this - ").append(thisElement->getName());
    }
    else
    {
        module.append(name);
    }
    QListIterator<Element*> parameterIterator(arguments);
    while (parameterIterator.hasNext())
    {
        module.append(" ");
        const Element* parameter = parameterIterator.next();
        module.append(parameter->toPDDL());
    }
    QString keyword = "cost";
    if (type == "EffectModule")
    {
        keyword = "effect";
        // find and add effect numerical fluents
        keyword.append("\n      ");
        QListIterator<Element*> effectIterator(effects->getBody());
        while (effectIterator.hasNext())
        {
            const Element* effect = effectIterator.next();
            std::cout << "Module::toPDDL recursive call: " << qPrintable(effect->getName()) << std::endl << std::flush;
            const Element* innerMost = effect;
            while (innerMost->getScopedAccess() != NULL)
            {
                innerMost = innerMost->getScopedAccess();
            }
            std::cout << "Module::toPDDL INFO: effect parameter: " << qPrintable(effect->getType()) << " " << qPrintable(effect->getName()) << std::endl << std::flush;
//            keyword.append(" ").append(effect->toPDDL());
            addNumericsToPDDL(keyword, innerMost, effect->toPDDL());
        }
        keyword.append("\n    ");
    }
    else if (type == "ConditionModule")
    {
        keyword = "conditionchecker";
    }
    module.append(" ").append(keyword);
    QString plannerLibraryCall = libraryCall->getName();
    if (thisElement != NULL)
    {
        plannerLibraryCall.prepend(thisElement->getName()+ "_");
    }
    plannerLibraryCall.replace("@", "_plannerCall@");
    module.append(" ").append(plannerLibraryCall);
    module.append(")\n");
    return module;
}

void Module::addNumericsToPDDL(QString& pddlString, const Element* element, const QString& objectParameter) const
{
    std::cout << "Module::addNumericsToPDDL: " << qPrintable(element->getType()) << " " << qPrintable(element->getName()) << " " << qPrintable(objectParameter) << std::endl << std::flush;
    const Element* symbol = element->findSymbolDefinition(element->getName());
    const Element* type =  NULL;
    if (symbol != NULL)
    {
        type = findType(symbol->getType());
    }
    if (symbol != NULL && symbol->getType() == "float")
    {
        std::cout << "Module::addNumericsToPDDL numeric: " << qPrintable(element->getType()) << " " << qPrintable(element->getName()) << std::endl << std::flush;
        // if numeric: add to list
        pddlString.append(objectParameter).append(" ");
//        pddlString.append(element->toPDDL());
    }
    else if (type != NULL)
    {
        std::cout << "Module::addNumericsToPDDL object: " << qPrintable(element->getType()) << std::endl << std::flush;
        // if object: recursive call for child elements
        QListIterator<Element*> it(type->getBody());
        while (it.hasNext())
        {
            const Element* child = it.next();
            QString fluentString = "(";
            fluentString.append(type->getName()).append("_").append(child->getName()).append(" ");
            fluentString.append(objectParameter).append(")");
            addNumericsToPDDL(pddlString, child, fluentString);
        }
    }
    else
    {
        std::cout << "Module::addNumericsToPDDL double NULL: " << qPrintable(element->getType()) << " " << qPrintable(element->getName()) << std::endl << std::flush;
    }
}

}
