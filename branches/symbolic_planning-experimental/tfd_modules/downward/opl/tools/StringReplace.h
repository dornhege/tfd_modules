/*
 * StringReplace.h
 *
 *  Created on: Aug 10, 2011
 *      Author: Andreas Hertle
 */

#ifndef STRINGREPLACE_H_
#define STRINGREPLACE_H_

#include <QString>
#include <QList>
#include <QPair>

namespace opl
{

namespace tools
{

class StringReplace
{
private:
    QList<QPair<QString, QString> > replacements;

public:
    StringReplace();
    virtual ~StringReplace();

    void loadReplacementDefinitionFile(const QString& fileName);
    void replaceInFile(const QRegExp& fileNamePattern, const QString& sourceDirectory, const QString& destinationDirectory);
};

}

}

#endif /* STRINGREPLACE_H_ */
