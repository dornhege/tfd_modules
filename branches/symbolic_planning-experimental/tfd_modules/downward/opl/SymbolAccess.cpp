/*
 * SymbolAccess.cpp
 *
 *  Created on: May 4, 2011
 *      Author: Andreas Hertle
 */

#include "SymbolAccess.h"

namespace opl
{

SymbolAccess::SymbolAccess(const QString& type, const QString& name)
: Element(type, name)
{
    symbol = NULL;
    symbolType = NULL;
    pddlNamePrefix = "";
    isGlobal = false;
    isModuleAccess = false;
    isCostModuleAccess = false;
    isFluentAccess = false;
    isGrounded = false;
}

SymbolAccess::~SymbolAccess()
{
}

const Element* SymbolAccess::findSymbolDefinition(const QString& name) const
{
//    std::cout << "SymbolAccess::findSymbolDefinition: symbol " << qPrintable(name) << std::endl << std::flush;
    if (scopedAcceess != NULL)
    {
//        std::cout << "SymbolAccess::findSymbolDefinition: scopedAccess " << qPrintable(scopedAcceess->getName()) << std::endl << std::flush;
        if (name == scopedAcceess->getName())
        {
//            std::cout << "SymbolAccess::findSymbolDefinition: lookup in type " << qPrintable(symbolType->getName()) << std::endl << std::flush;
            return symbolType->findSymbolDefinition(name);
        }
//        std::cout << "SymbolAccess::findSymbolDefinition: symbol " << qPrintable(name) << std::endl << std::flush;
    }
    return Element::findSymbolDefinition(name);
}

void SymbolAccess::customEvaluation()
{
    if (name == "this")
    {
        symbolType = findThis();
        isGlobal = false;
        pddlNamePrefix = "?";
    }
    else
    {
        symbol = parent->findSymbolDefinition(name);
        if (symbol == NULL)
        {
            std::cout << "SymbolAccess::customEvaluation ERROR: symbol " << qPrintable(name) << " not found" << std::endl << std::flush;
            assert(symbol != NULL);
        }
        else
        {
            symbolType = parent->findType(symbol->getType());
            if (symbol->findThis() != NULL)
            {
                isGlobal = false;
                pddlNamePrefix = symbol->getParent()->getName();
                pddlNamePrefix.append("_");
            }
            else
            {
                isGlobal = true;
            }
            if (symbol->getType() == "CostModule" || symbol->getType() == "ConditionModule" || symbol->getType() == "EffectModule")
            {
                if (symbol->getType() == "CostModule")
                {
                    isCostModuleAccess = true;
                }
//                std::cout << "SymbolAccess::customEvaluation INFO: symbol " << qPrintable(name) << " is " << qPrintable(symbol->getType()) << " " << qPrintable(symbol->getName()) << std::endl << std::flush;
                isModuleAccess = true;
            }
            else if (symbol->getParent()->getType() == "Domain" || symbol->getParent()->getType() == "Type")
            {
//                std::cout << "SymbolAccess::customEvaluation INFO: symbol " << qPrintable(name) << " is fluent " << qPrintable(symbol->getType()) << " " << qPrintable(symbol->getName()) << std::endl << std::flush;
                isFluentAccess = true;
            }
            else if (symbol->getParent()->getType() == "Problem")
            {
//                std::cout << "SymbolAccess::customEvaluation INFO: symbol " << qPrintable(name) << " is grounded" << std::endl << std::flush;
                isGrounded = true;
            }
            else
            {
                pddlNamePrefix = "?";
            }
        }
    }
}

QString SymbolAccess::toPDDL() const
{
    QString access = "";
    access.append(pddlNamePrefix).append(name);
    if (isFluentAccess)
    {
        access.prepend("(");
        QListIterator<Element*> argumentIterator(arguments);
        while (argumentIterator.hasNext())
        {
            access.append(" ").append(argumentIterator.next()->toPDDL());
        }
        access.append(")");
    }
    else if (isModuleAccess)
    {
        QListIterator<Element*> argumentIterator(arguments);
        while (argumentIterator.hasNext())
        {
            access.append(" ").append(argumentIterator.next()->toPDDL());
        }
        access.prepend("[");
        access.append("]");
        if (!isCostModuleAccess)
        {
            access.prepend("(");
            access.append(")");
        }
    }
    if (scopedAcceess != NULL)
    {
        return scopedAcceess->toPDDL(access);
    }
    else
    {
        return access;
    }

}

QString SymbolAccess::toPDDL(const QString& objectString) const
{
    if (isModuleAccess)
    {
        QString pddlString;
        pddlString.append(pddlNamePrefix).append(name);
        pddlString.append(" ").append(objectString);
        QListIterator<Element*> it(arguments);
        while (it.hasNext())
        {
            const Element* parameter = it.next();
            pddlString.append(" ").append(parameter->toPDDL());
        }
        pddlString.prepend("[");
        pddlString.append("]");
        if (!isCostModuleAccess)
        {
            pddlString.prepend("(");
            pddlString.append(")");
        }
        return pddlString;
    }
    if (isFluentAccess)
    {
        QString pddlString = "(";
        pddlString.append(pddlNamePrefix).append(name);
        pddlString.append(" ").append(objectString);
        QListIterator<Element*> it(arguments);
        while (it.hasNext())
        {
            const Element* parameter = it.next();
            pddlString.append(" ").append(parameter->toPDDL());
        }
        pddlString.append(")");
        if (scopedAcceess != NULL)
        {
            return scopedAcceess->toPDDL(pddlString);
        }
        else
        {
            return pddlString;
        }
    }
    return ">>Leftover<<";
}

}
