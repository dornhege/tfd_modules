/*
 * ElementStructure.h
 *
 *  Created on: Apr 26, 2011
 *      Author: Andreas Hertle
 */

#ifndef ELEMENTSTRUCTURE_H_
#define ELEMENTSTRUCTURE_H_

#include <QStringList>
#include "ParseUnit.h"
#include "Element.h"

namespace opl
{

class ElementStructure
{
protected:
    QString keyWord;
    QStringList argumentConfigurationKeywords;
    QStringList bodyConfigurationKeywords;
    QStringList scopedAccessConfigurationKeywords;

public:
    virtual Element* identify(const ParseUnit* unit) const = 0;

    const QString& getKeyword() const {return keyWord;}
    const QStringList& getArgumentConfigurationKeywords() const {return argumentConfigurationKeywords;}
    const QStringList& getBodyConfigurationKeywords() const {return bodyConfigurationKeywords;}
    const QStringList& getScopedAccessConfigurationKeywords() const {return scopedAccessConfigurationKeywords;}
    virtual ~ElementStructure(){;}
};

}

#endif /* ELEMENTSTRUCTURE_H_ */
