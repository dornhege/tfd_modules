/*
 * Translator.h
 *
 *  Created on: Nov 26, 2010
 *      Author: Andreas Hertle
 */

#ifndef TRANSLATOR_H_
#define TRANSLATOR_H_

//#include "parsing2/Element.h"

namespace opl
{

class Translator
{
public:
    Translator();
    virtual ~Translator();

//    Element* readDomain(const QString& domainFileName);
    void preprocessFile(const QString& domainFileName, QStringList& tokens);

};

}

#endif /* TRANSLATOR_H_ */
