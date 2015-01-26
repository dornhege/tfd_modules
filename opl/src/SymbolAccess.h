/*
 * SymbolAccess.h
 *
 *  Created on: May 4, 2011
 *      Author: Andreas Hertle
 */

#ifndef SYMBOLACCESS_H_
#define SYMBOLACCESS_H_

#include "Element.h"

namespace opl
{

class SymbolAccess: public opl::Element
{
    const Element* symbol;
    const Element* symbolType;

    bool isGrounded;
    bool isCostModuleAccess;
    bool isModuleAccess;
    bool isFluentAccess;
    bool isGlobal;
    QString pddlNamePrefix;
public:
    SymbolAccess(const QString& type, const QString& name);
    virtual ~SymbolAccess();

    const Element* findSymbolDefinition(const QString& name) const;
    void customEvaluation();
    QString toPDDL() const;
    QString toPDDL(const QString& additionalArgument) const;
};

}


#endif /* SYMBOLACCESS_H_ */
