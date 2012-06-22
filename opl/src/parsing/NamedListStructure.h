/*
 * NamedListStructure.h
 *
 *  Created on: May 2, 2011
 *      Author: Andreas Hertle
 */

#ifndef NAMEDLISTSTRUCTURE_H_
#define NAMEDLISTSTRUCTURE_H_

#include "ElementStructure.h"

namespace opl
{

class NamedListStructure: public opl::ElementStructure
{
    QString name;

public:
    NamedListStructure();
    virtual ~NamedListStructure();

    Element* identify(const ParseUnit* unit) const;

    void setName(const QString& name) {this->name = name;}
    void setKeyWord(const QString& keyWord) {this->keyWord = keyWord;}
    void addListContentStructure(const QString& keyWord) {this->bodyConfigurationKeywords.append(keyWord);}
};

}

#endif /* NAMEDLISTSTRUCTURE_H_ */
