/*
 * ElementDetector.h
 *
 *  Created on: Apr 27, 2011
 *      Author: Andreas Hertle
 */

#ifndef ELEMENTDETECTOR_H_
#define ELEMENTDETECTOR_H_

#include <QHash>
#include "ElementStructure.h"

namespace opl
{

class ElementDetector
{
    QHash<QString, ElementStructure*> structureDefinitions;
public:
    ElementDetector();
    virtual ~ElementDetector();

    void defineStructure(ElementStructure* structure){structureDefinitions.insert(structure->getKeyword(), structure);}
    Element* detect(const ParseUnit* unit, const QStringList possibleStructures) const;

};

}

#endif /* ELEMENTDETECTOR_H_ */
