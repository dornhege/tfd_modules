/*
 * Type.h
 *
 *  Created on: Nov 23, 2010
 *      Author: Andreas Hertle
 */

#ifndef DOMAIN_H_
#define DOMAIN_H_

//#include "parsing/Parser.h"
#include "Element.h"

namespace opl
{

class Domain : public Element
{
public:
    Domain(const QString& type, const QString& name);
    virtual ~Domain();


    const Element* findType(const QString& type) const;
    void customEvaluation();
    const Element* findSymbolDefinition(const QString& symbol) const;
    QString toPDDL() const;
};

}

#endif /* DOMAIN_H_ */
