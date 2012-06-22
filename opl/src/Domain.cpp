/*
 * Type.cpp
 *
 *  Created on: Nov 23, 2010
 *      Author: Andreas Hertle
 */

#include <iostream>
#include <assert.h>
#include "Domain.h"

namespace opl
{

Domain::Domain(const QString& type, const QString& name)
: Element(type, name)
{

}

Domain::~Domain()
{
}

QString Domain::toPDDL() const
{
    QString domain = "(define (domain ";
    domain.append(name);
    domain.append(")\n  (:requirements :strips :typing :durative-actions :numeric-fluents :object-fluents :modules)\n");
    domain.append("  (:oplinit initCallbackInterface@libtfd_opl_").append(name).append(".so)\n");

    QString types = "  (:types\n";
    QString modules = "  (:modules\n";
    QString predicates = "  (:predicates\n";
    QString fluents = "  (:functions\n";
    QString durativeActions;

    QListIterator<Element*> elementIterator(body);
    while (elementIterator.hasNext())
    {
        const Element* element = elementIterator.next();
        if (element->getType() == "Type")
        {
            types.append(element->toPDDL());

            // process subelements
            QListIterator<Element*> elementsChildrenIterator(element->getBody());
            while (elementsChildrenIterator.hasNext())
            {
                const Element* child = elementsChildrenIterator.next();
                if (child->getType() == "boolean")
                {
                    predicates.append(child->toPDDL());
                }
                else if (child->getType() == "float")
                {
                    fluents.append(child->toPDDL());
                }
                else if (child->getType() == "DurativeAction")
                {
                    durativeActions.append(child->toPDDL());
                }
                else if (child->getType() == "CostModule" || child->getType() == "ConditionModule" || child->getType() == "EffectModule")
                {
                    modules.append(child->toPDDL());
                }
                else
                {
                    fluents.append(child->toPDDL());
                }
            }
        }
        else if (element->getType() == "boolean")
        {
            predicates.append(element->toPDDL());
        }
        else if (element->getType() == "float")
        {
            fluents.append(element->toPDDL());
        }
        else if (element->getType() == "CostModule" || element->getType() == "ConditionModule" || element->getType() == "EffectModule")
        {
            modules.append(element->toPDDL());
        }
        else
        {
            fluents.append(element->toPDDL());
        }

    }

    types.append("  )\n");
    modules.append("  )\n");
    predicates.append("  )\n");
    fluents.append("  )\n");
    domain.append(types);
    domain.append(modules);
    domain.append(predicates);
    domain.append(fluents);
    domain.append(durativeActions);
    domain.append(")");

    return domain;
}

const Element* Domain::findType(const QString& type) const
{
    if (type == this->type)
    {
        return this;
    }
    QListIterator<Element*> it(body);
    while (it.hasNext())
    {
        Element* element = it.next();
        if (element->getName() == type && element->getType() == "Type")
        {
            return element;
        }
    }
    return NULL;
}

void Domain::customEvaluation()
{
    QListIterator<Element*> it(body);
    while (it.hasNext())
    {
        Element* element = it.next();
        if (element->getType() == "Type")
        {
            element->customEvaluation();
        }
    }
}

const Element* Domain::findSymbolDefinition(const QString& symbol) const
{
    const Element* element = Element::findSymbolDefinition(symbol);
    return element;
}

}
