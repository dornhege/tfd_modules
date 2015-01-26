/*
 * Element.cpp
 *
 *  Created on: Nov 29, 2010
 *      Author: Andreas Hertle
 */

#include <QList>
#include "Element.h"

namespace opl
{

QString Element::scopeSeparator = ".";

std::ostream& operator<<(std::ostream& output, const Element& element)
{
    // defined names
    output << qPrintable(element.type) << " ";
//    output << qPrintable(element.scope) << qPrintable(Element::scopeSeparator);
    output << qPrintable(element.name) << std::endl;
    QListIterator<Element* > it(element.arguments);
    while (it.hasNext())
    {
        output << *(it.next());
    }
    it = QListIterator<Element* >(element.body);
    while (it.hasNext())
    {
        output << *(it.next());
    }
    if (element.scopedAcceess != NULL)
    {
        output << *(element.scopedAcceess);
    }
    return output;
}

Element::Element(const QString& type, const QString& name) :
        type(type), name(name)
{
    parent = NULL;
    scopedAcceess = NULL;
}

Element::~Element()
{
    QListIterator<Element*> it(arguments);
    while (it.hasNext())
    {
        delete it.next();
    }
    QListIterator<Element*> it2(body);
    while (it2.hasNext())
    {
        delete it2.next();
    }
    delete scopedAcceess;
}

void Element::evaluate()
{
    // check for duplicate names
    checkForNameConflicts();
    customEvaluation();

    QListIterator<Element*> it(arguments);
    while (it.hasNext())
    {
        it.next()->evaluate();
    }
    QListIterator<Element*> it2(body);
    while (it2.hasNext())
    {
        it2.next()->evaluate();
    }
    if (scopedAcceess != NULL)
    {
        scopedAcceess->evaluate();
    }
}

void Element::customEvaluation()
{

}

void Element::addArgument(Element* element)
{
//    assert(nameAvailable(element->getName()));
    element->setParent(this);
    arguments.append(element);
}

void Element::addToBody(Element* element)
{
//    assert(nameAvailable(element->getName()));
    element->setParent(this);
    body.append(element);
}

void Element::setSocpedAccess(Element* element)
{
    element->setParent(this);
    scopedAcceess = element;
}

bool Element::definesSymbol() const
{
    return false;
}

bool Element::definesSymbol(const QString& name) const
{
    return definesSymbol() && name == this->name;
}

const Element* Element::findSymbolDefinition(const QString& symbol) const
{
//    std::cout << "Element::findSymbolDefinition: INFO: Entity: " << qPrintable(type) << " "  << qPrintable(name) << std::endl << std::flush;
    QListIterator<Element*> it(arguments);
    while (it.hasNext())
    {
        Element* child = it.next();
        if (child->definesSymbol(symbol))
        {
            return child;
        }
    }
    it = QListIterator<Element*>(body);
    while (it.hasNext())
    {
        Element* child = it.next();
        if (child->definesSymbol(symbol))
        {
            return child;
        }
    }
    if (parent != NULL)
    {
        return parent->findSymbolDefinition(symbol);
    }
    return NULL;
}

const Element* Element::findThis() const
{
    if (parent != NULL)
    {
        return parent->findThis();
    }
    else
    {
        return NULL;
    }
}

const Element* Element::findType(const QString& type) const
{
    if (parent != NULL)
    {
        return parent->findType(type);
    }
    else
    {
        return NULL;
    }
}

void Element::checkForNameConflicts() const
{
    //std::cout << "Element::checkForNameConflicts" << std::endl;
    QListIterator<Element*> it(arguments);
    while (it.hasNext())
    {
        Element* symbol = it.next();
        if (symbol->definesSymbol())
        {
            const Element* conflict = findNameConflict(symbol);
            if (conflict != NULL)
            {
                std::cout << "Element::checkForNameConflicts: ERROR: name conflict detected: " << qPrintable(symbol->getName()) << " has multiple difintions in " << *this << std::endl << std::flush;
            }
        }
    }
    it = QListIterator<Element*>(body);
    while (it.hasNext())
    {
        Element* symbol = it.next();
        if (symbol->definesSymbol())
        {
            const Element* conflict = findNameConflict(symbol);
            if (conflict != NULL)
            {
                std::cout << "Element::checkForNameConflicts: ERROR: name conflict detected: " << qPrintable(symbol->getName()) << " has multiple difintions in " << *this << std::endl << std::flush;
            }
        }
    }
}

const Element* Element::findNameConflict(const Element* symbol) const
{
//    std::cout << "Element::findNameConflict for " << *symbol << std::endl;
    QListIterator<Element*> it(arguments);
    while (it.hasNext())
    {
        Element* element = it.next();
        if (element->definesSymbol() && element != symbol && element->getName() == symbol->getName())
        {
            return element;
        }
    }
    it = QListIterator<Element*>(body);
    while (it.hasNext())
    {
        Element* element = it.next();
        if (element->definesSymbol() && element != symbol && element->getName() == symbol->getName())
        {
            return element;
        }
    }
    return NULL;
}

QString Element::toPDDL(const QString& additionalArgument) const
{
    return additionalArgument;
}

}

