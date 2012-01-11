/*
 * Type.h
 *
 *  Created on: Nov 23, 2010
 *      Author: Andreas Hertle
 */

#ifndef TYPE_H_
#define TYPE_H_

#include "Element.h"

namespace opl
{

class Type : public Element
{
private:
    bool evaluated;
    QString superName;
    const Type* superType;

public:
    Type(const QString& type, const QString& name, const QString& superName);
    virtual ~Type();

    virtual const Element* findSymbolDefinition(const QString& name) const;
    const Element* findThis() const ;
    void customEvaluation();
    QString toPDDL() const;

    const Type* getSuperType() const {return superType;}
};

}

#endif /* TYPE_H_ */
